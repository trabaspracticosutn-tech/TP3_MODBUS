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

static TickType_t mb_timeout_ticks(uint32_t timeout_ms)
{
    return (timeout_ms + portTICK_PERIOD_MS - 1U) / portTICK_PERIOD_MS;
}

/* UART entrega bytes, no tramas. Acumular exactamente una solicitud o
 * respuesta evita partirlas en ventanas de 50 ms o juntar varias. */
static mb_result_t mb_receive_frame(uint8_t *frame, int *rx_len, bool response,
                                    uint32_t timeout_ms)
{
    TickType_t start = xTaskGetTickCount();
    TickType_t timeout = mb_timeout_ticks(timeout_ms);
    int expected = 0;
    *rx_len = 0;

    while (*rx_len < MB_MAX_FRAME_LEN) {
        TickType_t elapsed = xTaskGetTickCount() - start;
        if (elapsed >= timeout) {
            return *rx_len == 0 ? MB_ERR_TIMEOUT : MB_ERR_INVALID_FRAME;
        }
        TickType_t wait = timeout - elapsed;
        TickType_t gap = mb_timeout_ticks(MB_FRAME_GAP_MS) + 1U;
        /* Una trama truncada debe descartarse antes del reintento remoto,
         * para que ese reintento no se agregue a los bytes anteriores. */
        if (*rx_len > 0 && wait > gap) wait = gap;
        int received = uart_read_bytes(MB_UART_PORT, frame + *rx_len, 1, wait);
        if (received < 0) return MB_ERR_INVALID_FRAME;
        if (received == 0) {
            return *rx_len == 0 ? MB_ERR_TIMEOUT : MB_ERR_INVALID_FRAME;
        }
        ++*rx_len;

        /* El timeout corto del esclavo solo corresponde a la espera inicial.
         * Una trama que empezo al final de esa espera debe completarse. */
        if (*rx_len == 1 && !response) {
            start = xTaskGetTickCount();
            timeout = mb_timeout_ticks((MB_MAX_FRAME_LEN * MB_UART_BITS_PER_CHAR *
                                        1000U + MB_UART_BAUDRATE - 1U) /
                                       MB_UART_BAUDRATE + MB_SLAVE_LISTEN_TIMEOUT_MS);
        }

        if (*rx_len == 2) {
            uint8_t function = frame[1];
            if (response && (function & 0x80)) {
                expected = 5;
            } else if (function == MB_FC_READ_HOLDING_REGISTERS ||
                       function == MB_FC_READ_INPUT_REGISTERS) {
                expected = response ? 0 : 8;
            } else if (function == MB_FC_WRITE_SINGLE_REGISTER ||
                       function == MB_FC_WRITE_MULTIPLE_REGISTERS) {
                expected = !response && function == MB_FC_WRITE_MULTIPLE_REGISTERS ? 0 : 8;
            } else {
                /* Las solicitudes de funcion no soportada se delimitan por
                 * silencio para poder responder una excepcion. */
                if (response) return MB_ERR_INVALID_FRAME;
                while (*rx_len < MB_MAX_FRAME_LEN) {
                    received = uart_read_bytes(MB_UART_PORT, frame + *rx_len, 1,
                                                 mb_timeout_ticks(MB_FRAME_GAP_MS) + 1U);
                    if (received <= 0) return MB_OK;
                    ++*rx_len;
                }
                return MB_ERR_INVALID_FRAME;
            }
        }
        if (*rx_len == 3 && response &&
            (frame[1] == MB_FC_READ_HOLDING_REGISTERS ||
             frame[1] == MB_FC_READ_INPUT_REGISTERS)) {
            expected = 5 + frame[2];
        }
        if (*rx_len == 7 && !response && frame[1] == MB_FC_WRITE_MULTIPLE_REGISTERS) {
            expected = 9 + frame[6];
        }
        if (expected > MB_MAX_FRAME_LEN) return MB_ERR_INVALID_FRAME;
        if (expected != 0 && *rx_len == expected) return MB_OK;
    }
    return MB_ERR_INVALID_FRAME;
}

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
    /* Un tick extra garantiza t3.5 incluso si estamos cerca del siguiente tick.
     * No vaciar RX aqui: el esclavo podria tener otra solicitud pendiente. */
    vTaskDelay(mb_timeout_ticks(MB_FRAME_GAP_MS) + 1U);
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
    uint32_t tx_timeout_ms = (len * MB_UART_BITS_PER_CHAR * 1000U +
                              MB_UART_BAUDRATE - 1U) / MB_UART_BAUDRATE + 20U;
    esp_err_t tx_result = uart_wait_tx_done(MB_UART_PORT, mb_timeout_ticks(tx_timeout_ms));
#if MB_RS485_MANUAL_DE_RE
    mb_rs485_rx_mode();
#endif
    if (tx_result != ESP_OK) {
        ESP_LOGE(TAG, "Timeout esperando fin de transmision: %s", esp_err_to_name(tx_result));
        return MB_ERR_TX;
    }
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
    if (!req || req->slave_addr < 1 || req->slave_addr > 247 ||
        req->quantity == 0 || req->start_addr >= MB_REG_COUNT ||
        req->quantity > MB_REG_COUNT - req->start_addr) return 0;
    if (req->function_code == MB_FC_WRITE_SINGLE_REGISTER && req->quantity != 1) return 0;
    if (req->function_code == MB_FC_WRITE_MULTIPLE_REGISTERS &&
        req->quantity > sizeof(req->write_data) / sizeof(req->write_data[0])) return 0;

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

static mb_result_t mb_master_validate_response(const mb_request_t *req,
                                                const uint8_t *rx_frame, int rx_len)
{
    if (rx_len < 5 || rx_len > MB_MAX_FRAME_LEN) return MB_ERR_INVALID_FRAME;
    uint16_t crc_calc = modbus_crc16(rx_frame, rx_len - 2);
    uint16_t crc_recv = rx_frame[rx_len - 2] | (rx_frame[rx_len - 1] << 8);
    if (crc_calc != crc_recv) return MB_ERR_CRC;
    if (rx_frame[0] != req->slave_addr) return MB_ERR_INVALID_FRAME;
    if (rx_frame[1] == (req->function_code | 0x80)) {
        if (rx_len != 5) return MB_ERR_INVALID_FRAME;
        ESP_LOGW(TAG, "[MAESTRO] Esclavo %u: excepcion Modbus 0x%02X",
                 req->slave_addr, rx_frame[2]);
        return MB_ERR_EXCEPTION;
    }
    if (rx_frame[1] != req->function_code) return MB_ERR_INVALID_FRAME;

    if (req->function_code == MB_FC_READ_HOLDING_REGISTERS ||
        req->function_code == MB_FC_READ_INPUT_REGISTERS) {
        if (rx_frame[2] != req->quantity * 2U || rx_len != 5 + rx_frame[2]) {
            return MB_ERR_INVALID_FRAME;
        }
    } else {
        if (rx_len != 8) return MB_ERR_INVALID_FRAME;
        uint16_t echoed_addr = (rx_frame[2] << 8) | rx_frame[3];
        uint16_t echoed_value = (rx_frame[4] << 8) | rx_frame[5];
        uint16_t expected_value = req->function_code == MB_FC_WRITE_SINGLE_REGISTER
                                  ? req->write_data[0] : req->quantity;
        if (echoed_addr != req->start_addr || echoed_value != expected_value) {
            return MB_ERR_INVALID_FRAME;
        }
    }
    return MB_OK;
}

// Cada reintento vuelve a transmitir la solicitud antes de esperar respuesta.
static void mb_master_execute_request(const mb_request_t *req,
                                      const uint8_t *tx_frame, uint16_t frame_len)
{
    uint8_t rx_frame[MB_MAX_FRAME_LEN];
    mb_result_t result = MB_ERR_TIMEOUT;
    int rx_len = 0;

    for (int attempt = 0; attempt <= MB_MAX_RETRIES; attempt++) {
        uart_flush_input(MB_UART_PORT);
        result = mb_transmit_frame(tx_frame, frame_len);
        if (result == MB_OK) {
            result = mb_receive_frame(rx_frame, &rx_len, true, MB_RESPONSE_TIMEOUT_MS);
            if (result == MB_OK) result = mb_master_validate_response(req, rx_frame, rx_len);
        }
        if (result == MB_OK || result == MB_ERR_EXCEPTION) break;
        ESP_LOGW(TAG, "[MAESTRO] Esclavo %u: error %d, intento %d/%d",
                 req->slave_addr, result, attempt + 1, MB_MAX_RETRIES + 1);
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
            for (uint16_t i = 0; i < req->quantity; i++) {
                uint16_t val = (rx_frame[3 + i * 2] << 8) | rx_frame[4 + i * 2];
                xSemaphoreTake(s_reg_mutex, portMAX_DELAY);
                ((uint16_t *)&s_registers)[req->start_addr + i] = val;
                xSemaphoreGive(s_reg_mutex);

                mb_notify_t n = { .slave_addr = req->slave_addr, .reg_addr = (uint16_t)(req->start_addr + i),
                                   .value = val, .is_write = false, .result = MB_OK };
                mb_publish_notify(&n);
            }
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
            ESP_LOGE(TAG, "[MAESTRO] Solicitud invalida (FC=0x%02X, registro=%u, cantidad=%u)",
                     req.function_code, req.start_addr, req.quantity);
            mb_notify_t n = { .slave_addr = req.slave_addr, .reg_addr = req.start_addr,
                               .value = 0, .is_write = false, .result = MB_ERR_INVALID_FRAME };
            mb_publish_notify(&n);
            continue;
        }

        mb_master_execute_request(&req, tx_frame, frame_len);
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

static void mb_slave_send_exception(uint8_t function, uint8_t exception)
{
    uint8_t frame[5] = {s_own_addr, function | 0x80, exception};
    uint16_t crc = modbus_crc16(frame, 3);
    frame[3] = crc & 0xFF;
    frame[4] = crc >> 8;
    mb_transmit_frame(frame, sizeof(frame));
}

static void mb_slave_handle_frame(const uint8_t *rx, int rx_len)
{
    if (rx_len < 4 || rx_len > MB_MAX_FRAME_LEN) return;

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
            if (rx_len != 8) return;
            uint16_t start_addr = (rx[2] << 8) | rx[3];
            uint16_t quantity   = (rx[4] << 8) | rx[5];
            if (quantity == 0 || quantity > 125) {
                mb_slave_send_exception(func_code, 0x03);
                return;
            }
            if (start_addr >= MB_REG_COUNT || quantity > MB_REG_COUNT - start_addr) {
                mb_slave_send_exception(func_code, 0x02);
                return;
            }

            tx_len = mb_slave_build_response(func_code, start_addr, quantity, NULL, tx_frame);

            // Avisar a la Tarea de adquisición que se pidió(n) estos registros
            // (p. ej. útil si la adquisición quiere refrescar el dato justo antes de responder)
            for (uint16_t i = 0; i < quantity; i++) {
                mb_notify_t n = { .slave_addr = s_own_addr, .reg_addr = (uint16_t)(start_addr + i),
                                   .value = (uint16_t)((tx_frame[3 + i * 2] << 8) |
                                                       tx_frame[4 + i * 2]),
                                   .is_write = false, .result = MB_OK };
                mb_publish_notify(&n);
            }
            break;
        }
        case MB_FC_WRITE_SINGLE_REGISTER: {
            if (rx_len != 8) return;
            uint16_t reg_addr = (rx[2] << 8) | rx[3];
            uint16_t value    = (rx[4] << 8) | rx[5];
            if (reg_addr >= MB_REG_COUNT) {
                mb_slave_send_exception(func_code, 0x02);
                return;
            }

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
        case MB_FC_WRITE_MULTIPLE_REGISTERS: {
            if (rx_len < 9) return;
            uint16_t start_addr = (rx[2] << 8) | rx[3];
            uint16_t quantity = (rx[4] << 8) | rx[5];
            if (quantity == 0 || quantity > 123 || rx[6] != quantity * 2U ||
                rx_len != 9 + rx[6]) {
                mb_slave_send_exception(func_code, 0x03);
                return;
            }
            if (start_addr >= MB_REG_COUNT || quantity > MB_REG_COUNT - start_addr) {
                mb_slave_send_exception(func_code, 0x02);
                return;
            }

            xSemaphoreTake(s_reg_mutex, portMAX_DELAY);
            for (uint16_t i = 0; i < quantity; i++) {
                ((uint16_t *)&s_registers)[start_addr + i] =
                    (rx[7 + i * 2] << 8) | rx[8 + i * 2];
            }
            xSemaphoreGive(s_reg_mutex);
            tx_len = mb_slave_build_response(func_code, start_addr, quantity, &quantity, tx_frame);

            for (uint16_t i = 0; i < quantity; i++) {
                mb_notify_t n = {
                    .slave_addr = s_own_addr,
                    .reg_addr = (uint16_t)(start_addr + i),
                    .value = (uint16_t)((rx[7 + i * 2] << 8) | rx[8 + i * 2]),
                    .is_write = true,
                    .result = MB_OK,
                };
                mb_publish_notify(&n);
            }
            break;
        }
        default:
            ESP_LOGW(TAG, "[ESCLAVO %d] Function code 0x%02X no soportado", s_own_addr, func_code);
            mb_slave_send_exception(func_code, 0x01);
            return;
    }

    // Responder al maestro
    mb_transmit_frame(tx_frame, tx_len);
}

static void mb_slave_loop(void)
{
    uint8_t rx_frame[MB_MAX_FRAME_LEN];

    for (;;) {
        int rx_len;
        mb_result_t result = mb_receive_frame(rx_frame, &rx_len, false,
                                              MB_SLAVE_LISTEN_TIMEOUT_MS);
        if (result == MB_ERR_TIMEOUT) continue;
        if (result != MB_OK) {
            ESP_LOGW(TAG, "[ESCLAVO %u] Trama incompleta o invalida (%d bytes)",
                     s_own_addr, rx_len);
            uart_flush_input(MB_UART_PORT);
            continue;
        }
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
    ESP_ERROR_CHECK((role == MB_ROLE_MASTER || role == MB_ROLE_SLAVE) &&
                    (role != MB_ROLE_SLAVE || (own_addr >= 1 && own_addr <= 247))
                    ? ESP_OK : ESP_ERR_INVALID_ARG);
    s_role = role;
    s_own_addr = own_addr;

    uart_config_t uart_config = {
        .baud_rate = MB_UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_EVEN,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(MB_UART_PORT, MB_UART_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(MB_UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_rx_timeout(MB_UART_PORT, 4));
    ESP_ERROR_CHECK(uart_set_rx_full_threshold(MB_UART_PORT, 1));
    ESP_LOGI(TAG, "UART%d: %d baudios, 8N1, TX=%d RX=%d DE/RE=%d",
             MB_UART_PORT, MB_UART_BAUDRATE, MB_UART_TX_PIN, MB_UART_RX_PIN, MB_UART_RTS_PIN);

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
    uint8_t frame[MB_MAX_FRAME_LEN];
    if (mb_master_build_frame(req, frame) == 0) {
        ESP_LOGE(TAG, "Solicitud Modbus invalida; no se encolo");
        return 0;
    }
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
