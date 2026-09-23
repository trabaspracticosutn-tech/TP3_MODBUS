#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/gpio.h"
//#include "driver/adc.h"

#include "comm.h"

static const char *TAG = "app";

/* =========================================================================
 *  EJEMPLO A) Este ESP32 es el MAESTRO
 *  - Encola lecturas periódicas a los esclavos 1 y 2.
 *  - Una "Tarea de aplicación" consume la cola de notificaciones y hace
 *    algo con cada dato (ej: loguear, actualizar un dashboard, etc.)
 * ========================================================================= */
static void tarea_aplicacion_maestro(void *arg)
{
    QueueHandle_t notify_q = (QueueHandle_t)modbus_get_notify_queue();
    mb_notify_t n;

    while (1) {
        if (xQueueReceive(notify_q, &n, portMAX_DELAY) == pdTRUE) {
            if (n.result != MB_OK) {
                ESP_LOGW(TAG, "Esclavo %d, reg %d: error %d", n.slave_addr, n.reg_addr, n.result);
                continue;
            }
            // reg_addr 0 = 4001 (valor_analogico_1), reg_addr 3 = 4004 (estado_dispositivo), etc.
            ESP_LOGI(TAG, "Esclavo %d, reg 400%d = %d (%s)",
                     n.slave_addr, n.reg_addr + 1, n.value, n.is_write ? "escritura" : "lectura");
        }
    }
}

static void tarea_polling_maestro(void *arg)
{
    while (1) {
        // Leer 4001-4004 (valor_analogico_1, valor_analogico_2, contador, estado_dispositivo)
        // del esclavo 1
        mb_request_t req1 = {
            .slave_addr = 1,
            .function_code = MB_FC_READ_HOLDING_REGISTERS,
            .start_addr = 0,
            .quantity = 4,
        };
        modbus_master_enqueue_request(&req1);
        vTaskDelay(pdMS_TO_TICKS(500));

        // Lo mismo para el esclavo 2
        mb_request_t req2 = {
            .slave_addr = 2,
            .function_code = MB_FC_READ_HOLDING_REGISTERS,
            .start_addr = 0,
            .quantity = 4,
        };
        modbus_master_enqueue_request(&req2);
        vTaskDelay(pdMS_TO_TICKS(1500));
    }
}

void app_main_maestro(void)
{
    modbus_comm_task_start(MB_ROLE_MASTER, /*own_addr=*/0); // own_addr no aplica al maestro
    xTaskCreate(tarea_aplicacion_maestro, "tarea_app_maestro", 4096, NULL, 5, NULL);
    xTaskCreate(tarea_polling_maestro, "tarea_polling", 4096, NULL, 5, NULL);
}

/* =========================================================================
 *  EJEMPLO B) Este ESP32 es un ESCLAVO (por ejemplo, ESCLAVO 1)
 *  - "Tarea de aplicación": consume escrituras (ej: reg 4004 -> LED)
 *  - "Tarea de adquisición": lee el potenciómetro y actualiza 4001 en loop,
 *    así el comm task siempre responde con un valor fresco.
 * ========================================================================= */
#define LED_GPIO          2
#define POT_ADC_CHANNEL    ADC1_CHANNEL_6  // GPIO34, ejemplo

//static void tarea_aplicacion_esclavo(void *arg)
//{
//    QueueHandle_t notify_q = (QueueHandle_t)modbus_get_notify_queue();
//    mb_notify_t n;
//
//    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
//
//    while (1) {
//        if (xQueueReceive(notify_q, &n, portMAX_DELAY) == pdTRUE) {
//            if (!n.is_write) continue; // esta tarea solo actúa ante escrituras

//            if (n.reg_addr == 3) { // offset 3 = REG 40004 = estado_dispositivo
//                gpio_set_level(LED_GPIO, n.value ? 1 : 0);
//                ESP_LOGI(TAG, "LED actualizado por Modbus: %s", n.value ? "ON" : "OFF");
//            }
            // reg_addr == 4 (setpoint), 5 (tiempo), 6 (modo_operacion) ya quedaron
            // guardados en el mapa por el comm task; acá se podría reaccionar también
            // ante esos cambios si hiciera falta.
//        }
//    }
//}

//static void tarea_adquisicion_esclavo(void *arg)
//{
//    adc1_config_width(ADC_WIDTH_BIT_12);
//    adc1_config_channel_atten(POT_ADC_CHANNEL, ADC_ATTEN_DB_12);

//    while (1) {
//        int lectura = adc1_get_raw(POT_ADC_CHANNEL);
//        modbus_slave_set_register(0, (uint16_t)lectura); // offset 0 = REG 40001 = valor_analogico_1
//        vTaskDelay(pdMS_TO_TICKS(100)); // refresco periódico, independiente de cuándo pregunte el maestro
//    }
//}

//void app_main_esclavo(void)
//{
//    modbus_comm_task_start(MB_ROLE_SLAVE, /*own_addr=*/1); // dirección Modbus de ESTE esclavo
//    xTaskCreate(tarea_aplicacion_esclavo, "tarea_app_esclavo", 4096, NULL, 5, NULL);
//    xTaskCreate(tarea_adquisicion_esclavo, "tarea_adq_esclavo", 4096, NULL, 5, NULL);
//}

/* Descomentar solo uno de los dos según qué placa se está compilando: */
void app_main(void)
{
    app_main_maestro();
    // app_main_esclavo();
}
