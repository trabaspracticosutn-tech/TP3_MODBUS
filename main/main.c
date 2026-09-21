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
// Esta funcion es solo la tarea de aplicacion del esclavo, unicamente realiza la lectura del registro 4004 y configura las GPIO


QueueHandle_t Informacion_de_aplicacion = xQueueCreate(5, sizeof(uint16_t));
QueueHandle_t Aplication_queue          = xQueueCreate(5, sizeof(uint16_t));



void app_main(void)
{   


    
    tarea_de_aplicacion_esclavo = xTaskCreate(app_slave_task, "app_slave_task", 2048, NULL, 5, NULL);
    
    modbus_holding_registers.estado_dispositivo = Aplication_queue;


    GPIO_config();
    GPIO_set_direction(GPIO_LED, GPIO_MODE_OUTPUT);
    GPIO_set_direction(GPIO_MOTOR, GPIO_MODE_OUTPUT);

}


void app_slave_task(void *pvParameters)
{
    while (1)
    {   xqueue_receive(Aplication_queue, &mapa_registros.estado_dispositivo, portMAX_DELAY);
        // Leer el registro 40004 y actualizar el estado de los GPIO
        uint16_t estado_dispositivo = mapa_registros.estado_dispositivo;
        gpio_set_level(GPIO_LED, estado_dispositivo & 0x01); // Configura el estado del LED según el registro 40004
        gpio_set_level(GPIO_MOTOR, estado_dispositivo & 0x02); // Configura el estado del motor según el registro 40004

        xqueue_send(Informacion_de_aplicacion, &estado_dispositivo, portMAX_DELAY); // Enviar el estado del dispositivo a la cola de información de aplicación

        vTaskDelay(pdMS_TO_TICKS(100)); // Espera 100 ms antes de la siguiente lectura
    }
}


