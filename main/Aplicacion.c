#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/dac_oneshot.h"
#include "esp_err.h"



#define GPIO_LED GPIO_NUM_26
#define GPIO_MOTOR GPIO_NUM_23
#define ADC_VALOR_MAXIMO 4095U
#define DAC_VALOR_MAXIMO 255U

void app_slave_task(void *pvParameters);
void app_master_task(void *pvParameters);

#include "comm.h"

static const char *TAG = "control_led";

TaskHandle_t Tarea_de_aplicacion_esclavo;
TaskHandle_t Tarea_de_aplicacion_maestro;
// Esta funcion es solo la tarea de aplicacion del esclavo, unicamente realiza la lectura del registro 4004 y configura las GPIO


QueueHandle_t Informacion_de_aplicacion ; // cola para enviar la informacion de los registro 40004 al maestro
QueueHandle_t Estado_registro           ; // cola para leer el estado del registro 40004 desde el esclavo
QueueHandle_t estado_dipositivos        ; //cola para ver el estado de los dispositivos en el maestro
QueueHandle_t setear_dispositivos       ; //cola para setear el estado de los dispositivos desde el maestro al esclavo

void app_gpio_inicializacion(void)
{
    ESP_ERROR_CHECK(gpio_set_level(GPIO_LED, 0));
    ESP_ERROR_CHECK(gpio_set_level(GPIO_MOTOR, 0));
    ESP_ERROR_CHECK(gpio_set_direction(GPIO_LED, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_direction(GPIO_MOTOR, GPIO_MODE_OUTPUT));
}


void app_master_task(void *pvParameters)
{
    while (1)
    {
        mb_device_state_t estado;
        if (xQueueReceive(estado_dipositivos, &estado, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        estado.value = 0x03; // Estado deseado: encender el LED y el motor.
        xQueueSend(setear_dispositivos, &estado, portMAX_DELAY);

    }
}

void app_dac_master_task(void *pvParameters)
{
    QueueHandle_t valores_analogicos = (QueueHandle_t)pvParameters;
    ESP_ERROR_CHECK(valores_analogicos != NULL ? ESP_OK : ESP_ERR_INVALID_ARG);

    for (;;) {
        mb_notify_t dato;
        if (xQueueReceive(valores_analogicos, &dato, portMAX_DELAY) != pdTRUE ||
            dato.result != MB_OK || dato.is_write || dato.reg_addr != MB_REG_ADC_0_OFFSET) {
            continue;
        }

        uint32_t valor_adc = dato.value;
        if (valor_adc > ADC_VALOR_MAXIMO) {
            valor_adc = ADC_VALOR_MAXIMO;
        }
        uint16_t nivel_dac = (valor_adc * DAC_VALOR_MAXIMO) / ADC_VALOR_MAXIMO;
        unsigned porcentaje_nivel = (nivel_dac * 100U) / DAC_VALOR_MAXIMO;
        // El maestro calcula el nivel y lo escribe en el esclavo que envio el ADC.
        mb_request_t solicitud = {
            .slave_addr = dato.slave_addr,
            .function_code = MB_FC_WRITE_SINGLE_REGISTER,
            .start_addr = MB_REG_DAC_2_OFFSET,
            .quantity = 1,
            .write_data = {nivel_dac},
        };
        ESP_LOGI(TAG, "[MAESTRO] Esclavo %u, LED en DAC2 (GPIO26, pin fisico 10): "
                 "ADC0 usado=%u/%u -> nivel DAC=%u/%u (%u%%); "
                 "solicitud de escritura FC=0x%02X, registro %u "
                 "(offset %u), valor=0x%04X",
                 (unsigned)dato.slave_addr,
                 (unsigned)valor_adc, (unsigned)ADC_VALOR_MAXIMO,
                 (unsigned)nivel_dac, (unsigned)DAC_VALOR_MAXIMO,
                 porcentaje_nivel, (unsigned)solicitud.function_code,
                 (unsigned)(40001 + solicitud.start_addr),
                 (unsigned)solicitud.start_addr, (unsigned)nivel_dac);
        while (!modbus_master_enqueue_request(&solicitud)) {
            vTaskDelay(1);
        }
    }
}

void app_dac_slave_task(void *pvParameters)
{
    QueueHandle_t niveles_dac = (QueueHandle_t)pvParameters;
    ESP_ERROR_CHECK(niveles_dac != NULL ? ESP_OK : ESP_ERR_INVALID_ARG);
    dac_oneshot_handle_t salida;
    dac_oneshot_config_t configuracion = {
        // DAC2: GPIO26, pin fisico 10 del conector J2 de la DevKitC.
        .chan_id = DAC_CHAN_1,
    };
    ESP_ERROR_CHECK(dac_oneshot_new_channel(&configuracion, &salida));
    ESP_ERROR_CHECK(dac_oneshot_output_voltage(salida, 0));

    for (;;) {
        mb_dac_value_t dato;
        if (xQueueReceive(niveles_dac, &dato, portMAX_DELAY) != pdTRUE ||
            dato.canal != DAC_CHAN_1) {
            continue;
        }

        // La orden ya contiene el nivel de 8 bits calculado por el maestro.
        uint8_t valor_dac = dato.valor_dac > DAC_VALOR_MAXIMO
                           ? DAC_VALOR_MAXIMO : dato.valor_dac;
        ESP_ERROR_CHECK(dac_oneshot_output_voltage(salida, valor_dac));
    }
}

void app_slave_task(void *pvParameters)
{
    while (1)
    {
        uint16_t estado_dispositivo;
        if (xQueueReceive(Estado_registro, &estado_dispositivo, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        // Leer el registro 40004 y actualizar el estado de los GPIO
        ESP_ERROR_CHECK(gpio_set_level(GPIO_LED, (estado_dispositivo & 0x01) != 0)); // Configura el estado del LED según el registro 40004
        ESP_ERROR_CHECK(gpio_set_level(GPIO_MOTOR, (estado_dispositivo & 0x02) != 0)); // Configura el estado del motor según el registro 40004

        xQueueSend(Informacion_de_aplicacion, &estado_dispositivo, portMAX_DELAY); // Enviar el estado del dispositivo a la cola de información de aplicación

    }
}
