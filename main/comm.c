#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"  // esp_rom_delay_us

#include "comm.h"

static const char *TAG = "comm";

/* ---------- Estado interno compartido ---------- */
static mb_role_t      s_role;
static uint8_t         s_own_addr;         // solo rol ESCLAVO
static QueueHandle_t   s_request_queue;    // solo rol MAESTRO (mb_request_t entrantes)
static QueueHandle_t   s_notify_queue;     // ambos roles (mb_notify_t salientes hacia app)

// Banco de registros: en rol ESCLAVO es el propio; en rol MAESTRO se usa
// como caché simple del último esclavo consultado (alcanza para 1 esclavo
// activo a la vez desde este maestro; para varios, ver nota en el header).
static modbus_holding_registers_t s_registers = {0};
static SemaphoreHandle_t          s_reg_mutex;

/* =======================================================
 *  CRC16 Modbus (compartido por ambos roles)
 * ======================================================= */
uint16_t modbus_crc16(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t pos = 0; pos < len; pos++) {
        crc ^= (uint16_t)buf[pos];
        for (int i = 8; i != 0; i--) {
            if (crc & 0x0001) { crc >>= 1; crc ^= 0xA001; }
            else               { crc >>= 1; }
        }
    }
    return crc;
}

/* =======================================================
 *  Transmisión por UART con control de dirección RS485
 *  (idéntica para ambos roles: transmitir es transmitir)
 * ======================================================= */
#if MB_RS485_MANUAL_DE_RE
static inline void mb_rs485_tx_mode(void) { gpio_set_level(MB_UART_RTS_PIN, 1); esp_rom_delay_us(50); }
static inline void mb_rs485_rx_mode(void) { gpio_set_level(MB_UART_RTS_PIN, 0); }
#endif

static mb_result_t mb_transmit_frame(const uint8_t *frame, uint16_t len)
{
    uart_flush_input(MB_UART_PORT);
#if MB_RS485_MANUAL_DE_RE
    mb_rs485_tx_mode();
#endif
    int written = uart_write_bytes(MB_UART_PORT, (const char *)frame, len);
    if (written != len) {
        ESP_LOGE(TAG, "Error al transmitir (%d/%d bytes)", written, len);
#if MB_RS485_MANUAL_DE_RE
        mb_rs485_rx_mode();
#endif
        return MB_ERR_TX;
    }
    uart_wait_tx_done(MB_UART_PORT, pdMS_TO_TICKS(50));
#if MB_RS485_MANUAL_DE_RE
    mb_rs485_rx_mode();
#endif
    return MB_OK;
}

static void mb_publish_notify(const mb_notify_t *n)
{
    // No bloqueante: si la app no está consumiendo, se descarta en vez de
    // frenar la tarea de comunicación (que tiene requisitos de timing).
    xQueueSend(s_notify_queue, n, 0);
}

/* =======================================================================
 *  ============ ROL MAESTRO: generar, transmitir, esperar, procesar ======
 * ======================================================================= */

static uint16_t mb_master_build_frame(const mb_request_t *req, uint8_t *frame)
{
    uint16_t idx = 0;
    frame[idx++] = req->slave_addr;
    frame[idx++] = req->function_code;

    switch (req->function_code) {
        case MB_FC_READ_HOLDING_REGISTERS:
        case MB_FC_READ_INPUT_REGISTERS:
            frame[idx++] = (req->start_addr >> 8) & 0xFF;
            frame[idx++] = req->start_addr & 0xFF;
            frame[idx++] = (req->quantity >> 8) & 0xFF;
            frame[idx++] = req->quantity & 0xFF;
            break;
        case MB_FC_WRITE_SINGLE_REGISTER:
            frame[idx++] = (req->start_addr >> 8) & 0xFF;
            frame[idx++] = req->start_addr & 0xFF;
            frame[idx++] = (req->write_data[0] >> 8) & 0xFF;
            frame[idx++] = req->write_data[0] & 0xFF;
            break;
        case MB_FC_WRITE_MULTIPLE_REGISTERS: {
            frame[idx++] = (req->start_addr >> 8) & 0xFF;
            frame[idx++] = req->start_addr & 0xFF;
            frame[idx++] = (req->quantity >> 8) & 0xFF;
            frame[idx++] = req->quantity & 0xFF;
            uint8_t byte_count = req->quantity * 2;
            frame[idx++] = byte_count;
            for (int i = 0; i < req->quantity; i++) {
                frame[idx++] = (req->write_data[i] >> 8) & 0xFF;
                frame[idx++] = req->write_data[i] & 0xFF;
            }
            break;
        }
        default:
            return 0;
    }
    uint16_t crc = modbus_crc16(frame, idx);
    frame[idx++] = crc & 0xFF;
    frame[idx++] = (crc >> 8) & 0xFF;
    return idx;
}

// Espera respuesta y notifica el resultado registro por registro (Lectura)
// o confirma la escritura (Escritura), publicando en s_notify_queue.
static void mb_master_wait_and_process(const mb_request_t *req)
{
    uint8_t rx_frame[MB_MAX_FRAME_LEN];
    mb_result_t result = MB_ERR_TIMEOUT;
    int rx_len = 0;

    for (int attempt = 0; attempt <= MB_MAX_RETRIES; attempt++) {
        rx_len = uart_read_bytes(MB_UART_PORT, rx_frame, sizeof(rx_frame),
                                  pdMS_TO_TICKS(MB_RESPONSE_TIMEOUT_MS));
        if (rx_len <= 0) {
            ESP_LOGW(TAG, "[MAESTRO] Timeout esclavo %d (intento %d/%d)",
                     req->slave_addr, attempt + 1, MB_MAX_RETRIES + 1);
            result = MB_ERR_TIMEOUT;
            continue;
        }
        if (rx_len < 5) { result = MB_ERR_INVALID_FRAME; continue; }

        uint16_t crc_calc = modbus_crc16(rx_frame, rx_len - 2);
        uint16_t crc_recv = rx_frame[rx_len - 2] | (rx_frame[rx_len - 1] << 8);
        if (crc_calc != crc_recv) { result = MB_ERR_CRC; continue; }

        if (rx_frame[0] != req->slave_addr) { result = MB_ERR_INVALID_FRAME; continue; }
        if (rx_frame[1] & 0x80) { result = MB_ERR_EXCEPTION; continue; }
        if (rx_frame[1] != req->function_code) { result = MB_ERR_INVALID_FRAME; continue; }

        result = MB_OK;
        break;
    }

    if (result != MB_OK) {
        mb_notify_t n = { .slave_addr = req->slave_addr, .reg_addr = req->start_addr,
                           .value = 0, .is_write = (req->function_code != MB_FC_READ_HOLDING_REGISTERS &&
                                                     req->function_code != MB_FC_READ_INPUT_REGISTERS),
                           .result = result };
        mb_publish_notify(&n);
        return;
    }

    switch (req->function_code) {
        case MB_FC_READ_HOLDING_REGISTERS:
        case MB_FC_READ_INPUT_REGISTERS: {
            uint8_t byte_count = rx_frame[2];
            uint16_t n_regs = byte_count / 2;
            xSemaphoreTake(s_reg_mutex, portMAX_DELAY);
            for (uint16_t i = 0; i < n_regs && (req->start_addr + i) < MB_REG_COUNT; i++) {
                uint16_t val = (rx_frame[3 + i * 2] << 8) | rx_frame[4 + i * 2];
                ((uint16_t *)&s_registers)[req->start_addr + i] = val;
                xSemaphoreGive(s_reg_mutex);

                mb_notify_t n = { .slave_addr = req->slave_addr, .reg_addr = (uint16_t)(req->start_addr + i),
                                   .value = val, .is_write = false, .result = MB_OK };
                mb_publish_notify(&n);

                xSemaphoreTake(s_reg_mutex, portMAX_DELAY);
            }
            xSemaphoreGive(s_reg_mutex);
            break;
        }
        case MB_FC_WRITE_SINGLE_REGISTER:
        case MB_FC_WRITE_MULTIPLE_REGISTERS: {
            mb_notify_t n = { .slave_addr = req->slave_addr, .reg_addr = req->start_addr,
                               .value = req->write_data[0], .is_write = true, .result = MB_OK };
            mb_publish_notify(&n);
            break;
        }
        default:
            break;
    }
}

static void mb_master_loop(void)
{
    uint8_t tx_frame[MB_MAX_FRAME_LEN];
    mb_request_t req;

    for (;;) {
        // 1) Generar solicitud: bloquea hasta que la app encole algo
        if (xQueueReceive(s_request_queue, &req, portMAX_DELAY) != pdTRUE) continue;

        uint16_t frame_len = mb_master_build_frame(&req, tx_frame);
        if (frame_len == 0) {
            ESP_LOGE(TAG, "[MAESTRO] Function code 0x%02X no soportado", req.function_code);
            mb_notify_t n = { .slave_addr = req.slave_addr, .reg_addr = req.start_addr,
                               .value = 0, .is_write = false, .result = MB_ERR_INVALID_FRAME };
            mb_publish_notify(&n);
            continue;
        }

        // 2) Transmitir trama
        if (mb_transmit_frame(tx_frame, frame_len) != MB_OK) {
            mb_notify_t n = { .slave_addr = req.slave_addr, .reg_addr = req.start_addr,
                               .value = 0, .is_write = false, .result = MB_ERR_TX };
            mb_publish_notify(&n);
            continue;
        }

        // 3) Esperar respuesta + 4) Procesar + 5) Detectar error/timeout
        mb_master_wait_and_process(&req);
    }
}

/* =======================================================================
 *  ============ ROL ESCLAVO: escuchar, responder, notificar ==============
 * ======================================================================= */

static uint16_t mb_slave_build_response(uint8_t func_code, uint16_t start_addr,
                                         uint16_t quantity, const uint16_t *echo_value,
                                         uint8_t *frame)
{
    uint16_t idx = 0;
    frame[idx++] = s_own_addr;
    frame[idx++] = func_code;

    if (func_code == MB_FC_READ_HOLDING_REGISTERS || func_code == MB_FC_READ_INPUT_REGISTERS) {
        uint8_t byte_count = quantity * 2;
        frame[idx++] = byte_count;
        xSemaphoreTake(s_reg_mutex, portMAX_DELAY);
        for (uint16_t i = 0; i < quantity && (start_addr + i) < MB_REG_COUNT; i++) {
            uint16_t val = ((uint16_t *)&s_registers)[start_addr + i];
            frame[idx++] = (val >> 8) & 0xFF;
            frame[idx++] = val & 0xFF;
        }
        xSemaphoreGive(s_reg_mutex);
    } else { // write single/multiple: se responde con eco de dirección + valor/cantidad
        frame[idx++] = (start_addr >> 8) & 0xFF;
        frame[idx++] = start_addr & 0xFF;
        frame[idx++] = (echo_value[0] >> 8) & 0xFF;
        frame[idx++] = echo_value[0] & 0xFF;
    }

    uint16_t crc = modbus_crc16(frame, idx);
    frame[idx++] = crc & 0xFF;
    frame[idx++] = (crc >> 8) & 0xFF;
    return idx;
}

static void mb_slave_handle_frame(const uint8_t *rx, int rx_len)
{
    if (rx_len < 5) return; // trama muy corta, se descarta

    uint16_t crc_calc = modbus_crc16(rx, rx_len - 2);
    uint16_t crc_recv = rx[rx_len - 2] | (rx[rx_len - 1] << 8);
    if (crc_calc != crc_recv) {
        ESP_LOGW(TAG, "[ESCLAVO %d] CRC inválido, trama descartada", s_own_addr);
        return; // en Modbus RTU, ante CRC inválido el esclavo simplemente no responde
    }

    uint8_t addr = rx[0];
    if (addr != s_own_addr) return; // la trama es para otro esclavo del bus

    uint8_t func_code = rx[1];
    uint8_t tx_frame[MB_MAX_FRAME_LEN];
    uint16_t tx_len = 0;

    switch (func_code) {
        case MB_FC_READ_HOLDING_REGISTERS:
        case MB_FC_READ_INPUT_REGISTERS: {
            uint16_t start_addr = (rx[2] << 8) | rx[3];
            uint16_t quantity   = (rx[4] << 8) | rx[5];
            if (start_addr + quantity > MB_REG_COUNT) return; // fuera de rango -> se podría responder excepción 0x02

            tx_len = mb_slave_build_response(func_code, start_addr, quantity, NULL, tx_frame);

            // Avisar a la Tarea de adquisición que se pidió(n) estos registros
            // (p. ej. útil si la adquisición quiere refrescar el dato justo antes de responder)
            for (uint16_t i = 0; i < quantity; i++) {
                mb_notify_t n = { .slave_addr = s_own_addr, .reg_addr = (uint16_t)(start_addr + i),
                                   .value = ((uint16_t *)&s_registers)[start_addr + i],
                                   .is_write = false, .result = MB_OK };
                mb_publish_notify(&n);
            }
            break;
        }
        case MB_FC_WRITE_SINGLE_REGISTER: {
            uint16_t reg_addr = (rx[2] << 8) | rx[3];
            uint16_t value    = (rx[4] << 8) | rx[5];
            if (reg_addr >= MB_REG_COUNT) return;

            xSemaphoreTake(s_reg_mutex, portMAX_DELAY);
            ((uint16_t *)&s_registers)[reg_addr] = value;
            xSemaphoreGive(s_reg_mutex);

            uint16_t echo[1] = { value };
            tx_len = mb_slave_build_response(func_code, reg_addr, 1, echo, tx_frame);

            // Avisar a la Tarea de aplicación que debe actuar sobre el hardware
            // (ej: reg 4004 -> estado del LED)
            mb_notify_t n = { .slave_addr = s_own_addr, .reg_addr = reg_addr,
                               .value = value, .is_write = true, .result = MB_OK };
            mb_publish_notify(&n);
            break;
        }
        default:
            ESP_LOGW(TAG, "[ESCLAVO %d] Function code 0x%02X no soportado", s_own_addr, func_code);
            return;
    }

    // Responder al maestro
    mb_transmit_frame(tx_frame, tx_len);
}

static void mb_slave_loop(void)
{
    uint8_t rx_frame[MB_MAX_FRAME_LEN];

    for (;;) {
        // Escuchar el bus: bloquea (con timeout corto) esperando una trama dirigida a este esclavo.
        int rx_len = uart_read_bytes(MB_UART_PORT, rx_frame, sizeof(rx_frame),
                                      pdMS_TO_TICKS(MB_SLAVE_LISTEN_TIMEOUT_MS));
        if (rx_len <= 0) continue; // nada llegó en esta ventana, seguir escuchando

        mb_slave_handle_frame(rx_frame, rx_len);
    }
}

/* =======================================================================
 *  ============ Tarea única: despacha según el rol configurado ===========
 * ======================================================================= */
static void modbus_comm_task(void *arg)
{
    if (s_role == MB_ROLE_MASTER) {
        ESP_LOGI(TAG, "Tarea Modbus iniciada en rol MAESTRO");
        mb_master_loop();
    } else {
        ESP_LOGI(TAG, "Tarea Modbus iniciada en rol ESCLAVO, dirección %d", s_own_addr);
        mb_slave_loop();
    }
    vTaskDelete(NULL); // nunca debería llegar acá (los loops son infinitos)
}

/* =======================================================================
 *  Inicialización común (UART + colas + la única tarea)
 * ======================================================================= */
void modbus_comm_task_start(mb_role_t role, uint8_t own_addr)
{
    s_role = role;
    s_own_addr = own_addr;

    uart_config_t uart_config = {
        .baud_rate = MB_UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(MB_UART_PORT, MB_UART_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(MB_UART_PORT, &uart_config));

#if MB_RS485_MANUAL_DE_RE
    ESP_ERROR_CHECK(uart_set_pin(MB_UART_PORT, MB_UART_TX_PIN, MB_UART_RX_PIN,
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_set_mode(MB_UART_PORT, UART_MODE_UART));
    gpio_config_t de_re_conf = {
        .pin_bit_mask = (1ULL << MB_UART_RTS_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&de_re_conf));
    mb_rs485_rx_mode();
#else
    ESP_ERROR_CHECK(uart_set_pin(MB_UART_PORT, MB_UART_TX_PIN, MB_UART_RX_PIN,
                                  MB_UART_RTS_PIN, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_set_mode(MB_UART_PORT, UART_MODE_RS485_HALF_DUPLEX));
#endif

    s_reg_mutex = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(s_reg_mutex != NULL ? ESP_OK : ESP_ERR_NO_MEM);
    s_notify_queue = xQueueCreate(16, sizeof(mb_notify_t));
    ESP_ERROR_CHECK(s_notify_queue != NULL ? ESP_OK : ESP_ERR_NO_MEM);

    if (role == MB_ROLE_MASTER) {
        s_request_queue = xQueueCreate(10, sizeof(mb_request_t));
        ESP_ERROR_CHECK(s_request_queue != NULL ? ESP_OK : ESP_ERR_NO_MEM);
    }

    // Misma función de tarea para ambos roles; el rol ya quedó guardado en s_role.
    BaseType_t result = xTaskCreate(modbus_comm_task, "modbus_comm_task", 4096, NULL, 10, NULL);
    ESP_ERROR_CHECK(result == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
}

int modbus_master_enqueue_request(const mb_request_t *req)
{
    if (s_role != MB_ROLE_MASTER || s_request_queue == NULL) return 0;
    return xQueueSend(s_request_queue, req, pdMS_TO_TICKS(100)) == pdTRUE;
}

void modbus_slave_get_registers(modbus_holding_registers_t *out)
{
    if (!out || !s_reg_mutex) return;
    xSemaphoreTake(s_reg_mutex, portMAX_DELAY);
    *out = s_registers;
    xSemaphoreGive(s_reg_mutex);
}

void modbus_slave_set_register(uint16_t reg_offset, uint16_t value)
{
    if (reg_offset >= MB_REG_COUNT || !s_reg_mutex) return;
    xSemaphoreTake(s_reg_mutex, portMAX_DELAY);
    ((uint16_t *)&s_registers)[reg_offset] = value;
    xSemaphoreGive(s_reg_mutex);
}

QueueHandle_t modbus_get_notify_queue(void)
{
    return s_notify_queue;
} 
