#ifndef MODBUS_COMM_H
#define MODBUS_COMM_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* ============================================================
 *  Configuración UART / RS485 (común a maestro y esclavo)
 * ============================================================ */
#define MB_UART_PORT          UART_NUM_1
#define MB_UART_TX_PIN        17
#define MB_UART_RX_PIN        16
#define MB_UART_RTS_PIN       18   // control DE/RE del MAX485
#define MB_UART_BAUDRATE      9600
#define MB_UART_BUF_SIZE      256

// 1 = manejo manual del DE/RE por GPIO (recomendado con módulos MAX485 genéricos)
// 0 = modo automático UART_MODE_RS485_HALF_DUPLEX del driver
#define MB_RS485_MANUAL_DE_RE 1

#define MB_MAX_FRAME_LEN         256
#define MB_RESPONSE_TIMEOUT_MS   200   // usado solo en rol MAESTRO
#define MB_MAX_RETRIES              3  // usado solo en rol MAESTRO
#define MB_SLAVE_LISTEN_TIMEOUT_MS  50 // usado solo en rol ESCLAVO (poleo de UART)

/* ============================================================
 *  Rol de la tarea: la MISMA tarea sirve para ambos, cambia
 *  únicamente el parámetro que se pasa a modbus_comm_task_start()
 * ============================================================ */
typedef enum {
    MB_ROLE_MASTER = 0,
    MB_ROLE_SLAVE  = 1,
} mb_role_t;

/* ---------- Function codes soportados ---------- */
typedef enum {
    MB_FC_READ_HOLDING_REGISTERS   = 0x03,
    MB_FC_READ_INPUT_REGISTERS     = 0x04,
    MB_FC_WRITE_SINGLE_REGISTER    = 0x06,
    MB_FC_WRITE_MULTIPLE_REGISTERS = 0x10,
} mb_function_code_t;

/* ---------- Resultado de una transacción ---------- */
typedef enum {
    MB_OK = 0,
    MB_ERR_TIMEOUT,
    MB_ERR_CRC,
    MB_ERR_EXCEPTION,
    MB_ERR_INVALID_FRAME,
    MB_ERR_TX,
} mb_result_t;

/* ---------- Mapa de registros Holding (40001-40007) ----------
 * Usado por el rol ESCLAVO como su banco de registros local, y
 * por el rol MAESTRO como caché de lo último leído de cada esclavo
 * (en el maestro hay un mapa por dirección de esclavo, ver abajo). */
typedef struct {
    uint16_t valor_analogico_1;     // REG 40001
    uint16_t valor_analogico_2;     // REG 40002
    uint16_t contador;              // REG 40003
    uint16_t estado_dispositivo;    // REG 40004
    uint16_t setpoint;              // REG 40005
    uint16_t tiempo;                // REG 40006
    uint16_t modo_operacion;        // REG 40007
} modbus_holding_registers_t;

#define MB_REG_COUNT  (sizeof(modbus_holding_registers_t) / sizeof(uint16_t))
#define MB_MAX_SLAVES 4   // cuántos esclavos puede trackear un maestro (direcciones 1..N)

/* ============================================================
 *  Rol MAESTRO: solicitudes salientes
 * ============================================================ */
typedef struct {
    uint8_t  slave_addr;
    uint8_t  function_code;
    uint16_t start_addr;       // offset dentro del mapa (0=4001 ... 6=4007)
    uint16_t quantity;
    uint16_t write_data[8];
} mb_request_t;

/* ============================================================
 *  Notificación hacia las tareas de aplicación / adquisición.
 *  - Rol ESCLAVO: el comm task la publica cuando el maestro remoto
 *    pidió leer o escribir un registro (cola_lectura / cola_escritura
 *    del diagrama).
 *  - Rol MAESTRO: el comm task la publica con el resultado de cada
 *    solicitud que hizo (equivalente a las QUEUE 4001 / QUEUE 4004).
 * ============================================================ */
typedef struct {
    uint8_t     slave_addr;    // relevante en rol MAESTRO (de qué esclavo vino/fue)
    uint16_t    reg_addr;      // offset de registro (0..6)
    uint16_t    value;         // valor leído o escrito
    bool        is_write;      // true = escritura, false = lectura
    mb_result_t result;        // MB_OK si todo salió bien
} mb_notify_t;

/* ============================================================
 *  API pública
 * ============================================================ */

// Inicializa UART, colas y arranca LA MISMA tarea configurada según 'role'.
// - MB_ROLE_MASTER: own_addr se ignora (pasar 0).
// - MB_ROLE_SLAVE:  own_addr es la dirección Modbus de ESTE esclavo (1, 2, ...).
void modbus_comm_task_start(mb_role_t role, uint8_t own_addr);

// --- Uso en rol MAESTRO ---
// Encola una solicitud saliente hacia un esclavo. El resultado llega de forma
// asincrónica por mb_get_notify_queue().
int modbus_master_enqueue_request(const mb_request_t *req);

// --- Uso en rol ESCLAVO ---
// Acceso thread-safe al banco de registros local de este esclavo.
void modbus_slave_get_registers(modbus_holding_registers_t *out);
void modbus_slave_set_register(uint16_t reg_offset, uint16_t value);

// --- Común a ambos roles ---
// Cola de notificaciones que consumen la Tarea de aplicación / Tarea de
// adquisición (ver diagrama). Se crea en modbus_comm_task_start().
void *modbus_get_notify_queue(void); // devuelve QueueHandle_t (void* para no obligar a incluir freertos acá)

// Calcula CRC16 Modbus sobre un buffer.
uint16_t modbus_crc16(const uint8_t *buf, uint16_t len);

#endif // MODBUS_COMM_H
