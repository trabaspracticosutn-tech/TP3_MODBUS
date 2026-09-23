#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/gpio.h"



#define GPIO_LED GPIO_NUM_22
#define GPIO_MOTOR GPIO_NUM_23

void app_slave_task(void *pvParameters);
void app_master_task(void *pvParameters);

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

TaskHandle_t Tarea_de_aplicacion_esclavo;
TaskHandle_t Tarea_de_aplicacion_maestro;
// Esta funcion es solo la tarea de aplicacion del esclavo, unicamente realiza la lectura del registro 4004 y configura las GPIO


QueueHandle_t Informacion_de_aplicacion ; // cola para enviar la informacion de los registro 40004 al maestro
QueueHandle_t Estado_registro           ; // cola para leer el estado del registro 40004 desde el esclavo
QueueHandle_t estado_dipositivos        ; //cola para ver el estado de los dispositivos en el maestro
QueueHandle_t setear_dispositivos       ; //cola para setear el estado de los dispositivos desde el maestro al esclavo

void app_main(void)
{   


    
    xTaskCreate(app_slave_task, "Tarea_de_aplicacion_esclavo", 2048, NULL, 5, NULL);
    xTaskCreate(app_master_task, "Tarea_de_aplicacion_maestro", 2048, NULL, 5, NULL);
    Informacion_de_aplicacion = xQueueCreate(10, sizeof(uint16_t));
    Estado_registro = xQueueCreate(10, sizeof(uint16_t));
    estado_dipositivos = xQueueCreate(10, sizeof(uint16_t));
    setear_dispositivos = xQueueCreate(10, sizeof(uint16_t));
    


    gpio_set_direction(GPIO_LED, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_MOTOR, GPIO_MODE_OUTPUT);

}


void app_master_task(void *pvParameters)
{
    while (1)
    {
         xQueueReceive(estado_dipositivos, &mapa_registros.estado_dispositivo, portMAX_DELAY);
        uint16_t estado_dispositivo = 0x03; // Estado deseado del dispositivo (encendido del led y el motor)
        mapa_registros.estado_dispositivo = estado_dispositivo;
        xQueueSend(setear_dispositivos, &mapa_registros.estado_dispositivo, portMAX_DELAY); // Enviar el estado del dispositivo al esclavo

    }
}

void app_slave_task(void *pvParameters)
{
    while (1)
    {   xQueueReceive(Estado_registro, &mapa_registros.estado_dispositivo, portMAX_DELAY);
        // Leer el registro 40004 y actualizar el estado de los GPIO
        gpio_set_level(GPIO_LED, mapa_registros.estado_dispositivo & 0x01); // Configura el estado del LED según el registro 40004
        gpio_set_level(GPIO_MOTOR, mapa_registros.estado_dispositivo & 0x02); // Configura el estado del motor según el registro 40004

        xQueueSend(Informacion_de_aplicacion, &mapa_registros.estado_dispositivo, portMAX_DELAY); // Enviar el estado del dispositivo a la cola de información de aplicación

        vTaskDelay(pdMS_TO_TICKS(100)); // Espera 100 ms antes de la siguiente lectura
    }
}


