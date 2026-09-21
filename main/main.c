#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

#include "rs485_uart.h"
#include "modbus_master.h"
#include "modbus_slave.h"

#define GPIO_LED 2
#define GIPIO_MOTOR 4


typedef struct {
    uint16_t valor_analogico_1;     // REG 40001 
    uint16_t valor_analogico_2;     // REG 40002     
    uint16_t contador;              // REG 40003
    uint16_t estado_dispositivo;    // REG 40004
    uint16_t setpoint;              // REG 40005
    uint16_t tiempo;                // REG 40006
    uint16_t modo_operacion;        // REG 40007
} modbus_holding_registers_t;

modbus_holding_registers_t mapa_registros = {0};

task_handle_t Tarea_de_aplicacion_esclavo;
task_handle_t Tarea_de_aplicacion_maestro;
// Esta funcion es solo la tarea de aplicacion del esclavo, unicamente realiza la lectura del registro 4004 y configura las GPIO


QueueHandle_t Informacion_de_aplicacion = xQueueCreate(5, sizeof(uint16_t)); // cola del esclavo para enviar el estado del dispositivo al maestro
QueueHandle_t Estado_registro           = xQueueCreate(5, sizeof(uint16_t)); // cola del esclavo para escribir el estado deseado por el maestro en el mapa de registros
QueueHandle_t estado_dipositivos        = xQueueCreate(5, sizeof(uint16_t)); //cola del maestro para recibir el estado del esclavo
QueueHandle_t setear dispositivos       = xQueueCreate(5, sizeof(uint16_t)); //cola del esclavo para recibir el estado del maestro


void app_main(void)
{   


    
    tarea_de_aplicacion_esclavo = xTaskCreate(app_slave_task, "app_slave_task", 2048, NULL, 5, NULL);
    tarea_de_aplicacion_maestro = xTaskCreate(app_master_task, "app_master_task", 2048, NULL, 5, NULL);

    


    GPIO_config();
    GPIO_set_direction(GPIO_LED, GPIO_MODE_OUTPUT);
    GPIO_set_direction(GPIO_MOTOR, GPIO_MODE_OUTPUT);

}

void app_master_task(void *pvParameters)
{
    while (1)
    {
        // Leer el registro 40004 del esclavo
        xqueue_receive(estado_dipositivos, &mapa_registros.estado_dispositivo, portMAX_DELAY);
        if (modbus_master_read_holding_registers(1, 40004, 1, &estado_dispositivo) == ESP_OK)
        {
            // Enviar el estado del dispositivo a la cola de aplicación
            xQueueSend(estado_dipositivos, &estado_dispositivo, portMAX_DELAY);
        }
        else
        {
            ESP_LOGE("MODBUS_MASTER", "Error al leer el registro 40004");
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // Espera 100 ms antes de la siguiente lectura
    }
}

void app_master_task(void *pvParameters)
{
    while (1)
    {
        xqueue_receive(Estado_dipositivos, &mapa_registros.estado_dispositivo, portMAX_DELAY);
        uint16_t estado_dispositivo = 0x03; // Estado deseado del dispositivo (encendido del led y el motor)
        mapa_estado_dispositivo.estado_dispositivo = estado_dispositivo;
        xqueue_send(setear_dispositivos, &mapa_registros.estado_dispositivo, portMAX_DELAY); // Enviar el estado del dispositivo al esclavo

    }
}

void app_slave_task(void *pvParameters)
{
    while (1)
    {   xqueue_receive(Estado_registro, &mapa_registros.estado_dispositivo, portMAX_DELAY);
        // Leer el registro 40004 y actualizar el estado de los GPIO
        gpio_set_level(GPIO_LED, mapa_registros.estado_dispositivo & 0x01); // Configura el estado del LED según el registro 40004
        gpio_set_level(GPIO_MOTOR, mapa_registros.estado_dispositivo & 0x02); // Configura el estado del motor según el registro 40004

        xqueue_send(Informacion_de_aplicacion, &estado_dispositivo, portMAX_DELAY); // Enviar el estado del dispositivo a la cola de información de aplicación

        vTaskDelay(pdMS_TO_TICKS(100)); // Espera 100 ms antes de la siguiente lectura
    }
}


