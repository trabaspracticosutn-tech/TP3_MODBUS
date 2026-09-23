#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_err.h"



#define GPIO_LED GPIO_NUM_22
#define GPIO_MOTOR GPIO_NUM_23

void app_slave_task(void *pvParameters);
void app_master_task(void *pvParameters);

#include "comm.h"

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


