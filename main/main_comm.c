#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_err.h"
#include "esp_log.h"

#include "comm.h"

#define APP_MODBUS_ROLE MB_ROLE_SLAVE

#define APP_MODBUS_ADDRESS 1


#define ADC_MUESTRAS 16
#define LONGITUD_COLAS 10

_Static_assert(APP_MODBUS_ROLE == MB_ROLE_MASTER ||
               APP_MODBUS_ROLE == MB_ROLE_SLAVE, "Rol Modbus invalido");
_Static_assert(APP_MODBUS_ADDRESS >= 1 && APP_MODBUS_ADDRESS <= 247,
               "Direccion Modbus invalida");

static const char *TAG = "app";
static uint32_t muestras = ADC_MUESTRAS;
static QueueHandle_t notificaciones;
static QueueHandle_t lecturas_analogicas_maestro;
static QueueHandle_t niveles_dac_esclavo;

static QueueHandle_t crear_cola(UBaseType_t longitud, UBaseType_t tamano)
{
    QueueHandle_t cola = xQueueCreate(longitud, tamano);
    ESP_ERROR_CHECK(cola != NULL ? ESP_OK : ESP_ERR_NO_MEM);
    return cola;
}

static void crear_tarea(TaskFunction_t funcion, const char *nombre,
                        void *parametro, TaskHandle_t *handle)
{
    BaseType_t resultado = xTaskCreate(funcion, nombre, 4096, parametro, 5, handle);
    ESP_ERROR_CHECK(resultado == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
}

static void encolar_solicitud(const mb_request_t *solicitud)
{
    while (!modbus_master_enqueue_request(solicitud)) {
        vTaskDelay(1);
    }
}

/* Un solo consumidor distribuye las notificaciones a cada modulo. */
static void tarea_notificaciones(void *arg)
{
    (void)arg;
    mb_notify_t notificacion;

    for (;;) {
        if (xQueueReceive(notificaciones, &notificacion, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        if (notificacion.result != MB_OK) {
            ESP_LOGW(TAG, "Esclavo %u, registro %u: error %d",
                     (unsigned)notificacion.slave_addr,
                     (unsigned)(40001 + notificacion.reg_addr),
                     notificacion.result);
            continue;
        }

        if (APP_MODBUS_ROLE == MB_ROLE_MASTER) {
            if (!notificacion.is_write && notificacion.reg_addr == MB_REG_ADC_0_OFFSET) {
                ESP_LOGI(TAG, "[MAESTRO] Esclavo %u, ADC0 (GPIO36, pin fisico 3): "
                         "registro %u (offset %u), valor leido=%u",
                         (unsigned)notificacion.slave_addr,
                         (unsigned)(40001 + notificacion.reg_addr),
                         (unsigned)notificacion.reg_addr,
                         (unsigned)notificacion.value);
                xQueueSend(lecturas_analogicas_maestro, &notificacion, portMAX_DELAY);
            } else if (notificacion.is_write &&
                       notificacion.reg_addr == MB_REG_DAC_2_OFFSET) {
                ESP_LOGI(TAG, "[MAESTRO] Esclavo %u, DAC2 (GPIO26, pin fisico 10): "
                         "respuesta de escritura recibida para registro %u (offset %u), "
                         "nivel DAC=%u/255",
                         (unsigned)notificacion.slave_addr,
                         (unsigned)(40001 + notificacion.reg_addr),
                         (unsigned)notificacion.reg_addr,
                         (unsigned)notificacion.value);
            } else {
                ESP_LOGI(TAG, "Esclavo %u, registro %u = %u (%s)",
                         (unsigned)notificacion.slave_addr,
                         (unsigned)(40001 + notificacion.reg_addr),
                         (unsigned)notificacion.value,
                         notificacion.is_write ? "escritura" : "lectura");
            }

            if (!notificacion.is_write && notificacion.reg_addr == 3) {
                mb_device_state_t estado = {
                    .slave_addr = notificacion.slave_addr,
                    .value = notificacion.value,
                };
                xQueueSend(estado_dipositivos, &estado, portMAX_DELAY);
            }
        } else if (notificacion.is_write && notificacion.reg_addr == 3) {
            uint16_t estado_aplicado;
            xQueueSend(Estado_registro, &notificacion.value, portMAX_DELAY);
            xQueueReceive(Informacion_de_aplicacion, &estado_aplicado, portMAX_DELAY);

            /* El banco ya contiene la escritura. Una confirmacion atrasada
             * no debe sobrescribir una orden Modbus mas reciente. */
            if (estado_aplicado != notificacion.value) {
                ESP_LOGW(TAG, "Estado aplicado distinto del solicitado");
            }
        } else if (notificacion.is_write &&
                   notificacion.reg_addr == MB_REG_DAC_2_OFFSET) {
            mb_dac_value_t dato = {
                .canal = 1, // DAC2: segundo canal del driver.
                .valor_dac = notificacion.value,
            };
            xQueueSend(niveles_dac_esclavo, &dato, portMAX_DELAY);
        }
    }
}

static void tarea_polling_maestro(void *arg)
{
    (void)arg;

    for (;;) {
        mb_request_t solicitud = {
            .slave_addr = APP_MODBUS_ADDRESS,
            .function_code = MB_FC_READ_HOLDING_REGISTERS,
            .start_addr = 0,
            .quantity = 4,
        };
        encolar_solicitud(&solicitud);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

static void tarea_escrituras_maestro(void *arg)
{
    (void)arg;
    mb_device_state_t estado;

    for (;;) {
        if (xQueueReceive(setear_dispositivos, &estado, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        mb_request_t solicitud = {
            .slave_addr = estado.slave_addr,
            .function_code = MB_FC_WRITE_SINGLE_REGISTER,
            .start_addr = 3,
            .quantity = 1,
            .write_data = {estado.value},
        };
        encolar_solicitud(&solicitud);
    }
}

static uint16_t leer_analogica(void)
{
    int canal = 0;
    uint16_t valor;
    xQueueSend(Com_to_adq, &canal, portMAX_DELAY);
    xQueueReceive(REG40001_to_Com, &valor, portMAX_DELAY);
    return valor;
}

/* comm.c responde desde su banco; el ADC debe refrescarlo antes de las lecturas. */
static void tarea_datos_esclavo(void *arg)
{
    (void)arg;
    for (;;) {
        uint16_t valor = leer_analogica();
        modbus_slave_set_register(MB_REG_ADC_0_OFFSET, valor);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Configuracion: %s, direccion del %s=%u",
             APP_MODBUS_ROLE == MB_ROLE_MASTER ? "MAESTRO" : "ESCLAVO",
             APP_MODBUS_ROLE == MB_ROLE_MASTER ? "esclavo consultado" : "esclavo local",
             (unsigned)APP_MODBUS_ADDRESS);
    if (APP_MODBUS_ROLE == MB_ROLE_MASTER) {
        estado_dipositivos = crear_cola(LONGITUD_COLAS, sizeof(mb_device_state_t));
        setear_dispositivos = crear_cola(LONGITUD_COLAS, sizeof(mb_device_state_t));
        lecturas_analogicas_maestro = crear_cola(LONGITUD_COLAS, sizeof(mb_notify_t));

        modbus_comm_task_start(MB_ROLE_MASTER, 0);
        notificaciones = modbus_get_notify_queue();
        ESP_ERROR_CHECK(notificaciones != NULL ? ESP_OK : ESP_ERR_NO_MEM);

        crear_tarea(app_dac_master_task, "control_dac", lecturas_analogicas_maestro, NULL);
        crear_tarea(tarea_notificaciones, "notificaciones", NULL, NULL);
        crear_tarea(app_master_task, "app_maestro", NULL, &Tarea_de_aplicacion_maestro);
        crear_tarea(tarea_escrituras_maestro, "escrituras", NULL, NULL);
        crear_tarea(tarea_polling_maestro, "polling", NULL, NULL);
    } else {
        Com_to_adq = crear_cola(LONGITUD_COLAS, sizeof(int));
        REG40001_to_Com = crear_cola(LONGITUD_COLAS, sizeof(uint16_t));
        Estado_registro = crear_cola(LONGITUD_COLAS, sizeof(uint16_t));
        Informacion_de_aplicacion = crear_cola(LONGITUD_COLAS, sizeof(uint16_t));
        niveles_dac_esclavo = crear_cola(LONGITUD_COLAS, sizeof(mb_dac_value_t));

        app_gpio_inicializacion();
        ADC1_inicializacion();
        ADC_calibracion();
        crear_tarea(ADC_leer_task, "ADC", &muestras, NULL);

        uint16_t valor = leer_analogica();

        modbus_comm_task_start(MB_ROLE_SLAVE, APP_MODBUS_ADDRESS);
        modbus_slave_set_register(MB_REG_ADC_0_OFFSET, valor);
        notificaciones = modbus_get_notify_queue();
        ESP_ERROR_CHECK(notificaciones != NULL ? ESP_OK : ESP_ERR_NO_MEM);

        crear_tarea(app_slave_task, "app_esclavo", NULL, &Tarea_de_aplicacion_esclavo);
        crear_tarea(app_dac_slave_task, "dac_esclavo", niveles_dac_esclavo, NULL);
        crear_tarea(tarea_notificaciones, "notificaciones", NULL, NULL);
        crear_tarea(tarea_datos_esclavo, "datos_esclavo", NULL, NULL);
    }
}
