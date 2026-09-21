#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

#include "rs485_uart.h"
#include "modbus_master.h"
#include "modbus_slave.h"

#include "esp_adc/adc_oneshot.h"          // Libreria para el uso del ADC
#include "esp_log.h"                      // Libreria para la calibración del ADC
#include "esp_adc/adc_cali.h"             // Libreria para la calibración del ADC
#include "esp_adc/adc_cali_scheme.h"      // Libreria para la calibración del ADC

#include <stdint.h>

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

// --------------------------- TAREA DE ADQUISICIÓN -----------------------------------

#define ADC_MUESTRAS 16                   // Cantidad de muestras a promediar
#define N_CANALES 2

adc_oneshot_unit_handle_t adc1_handle;    // Se crea una variable global
adc_cali_handle_t adc_cali_handle = NULL; // Manejador para la calibración del ADC
bool do_calibration = false;              // Bandera para indicar si se ha calibrado el ADC

adc_channel_t canales[N_CANALES] = {
    ADC_CHANNEL_0, // GPIO36
    ADC_CHANNEL_1, // GPIO37
};

void ADC1_inicializacion(                  ); // Función para inicializar el ADC1
void ADC_calibracion    (                  ); // Función para calibrar el ADC con Line Fitting Scheme
void ADC_leer_task      (void *pvParameters); // Tarea para leer el ADC en una tarea de FreeRTOS

QueueHandle_t Com_to_adq      = xQueueCreate(10, sizeof(       1)); // Cola de Tarea de Comunicación a Tarea de Adquisición
QueueHandle_t REG40001_to_Com = xQueueCreate(10, sizeof(uint16_t)); // Cola desde REG40001 a Tarea de Comunicación 
QueueHandle_t REG40002_to_Com = xQueueCreate(10, sizeof(uint16_t)); // Cola desde REG40002 a Tarea de Comunicación

// ------------------------------------------------------------------------------------

void app_main(void)
{
    ADC_calibracion     ();   // Se calibra el ADC con Line Fitting Scheme
    ADC1_inicializacion ();   // Se inicializa el ADC1

    uint32_t muestras = ADC_MUESTRAS;     // Se define la cantidad de muestras a promediar

    xTaskCreate(ADC_leer_task, "ADC", 4096, &muestras, 5, NULL);

}

void ADC_calibracion() // Función para calibrar el ADC con Line Fitting Scheme
{
    // Será para todos los canales del ADC1
    adc_cali_line_fitting_config_t cali_cfg = {
        .unit_id  = ADC_UNIT_1    ,   // ADC1
        .atten    = ADC_ATTEN_DB_12,  // Atenuación de 12 dB
        .bitwidth = ADC_BITWIDTH_12,  // Resolución de 12 bits
    };

    // El esquema disponible para mí ESP32 es el Line Fitting scheme
    // En caso de ser Curve Fitting, solo hay que reemplazar "line" por "curve"
    if (adc_cali_create_scheme_line_fitting(&cali_cfg, &adc_cali_handle) == ESP_OK) 
    {
        printf("Calibración del ADC realizada con éxito.\n");
        do_calibration = true; // Se ha calibrado el ADC
    } 
    else 
    {
        printf("Error al calibrar el ADC.\n");
    }
}

void ADC1_inicializacion(void) // Función para inicializar el ADC1
{
    // Configuración de la unidad ADC1
    adc_oneshot_unit_init_cfg_t adc1_config = { .unit_id = ADC_UNIT_1, };

    ESP_ERROR_CHECK( adc_oneshot_new_unit(&adc1_config, &adc1_handle) );

    // Configuración común para todos los canales
    adc_oneshot_chan_cfg_t adc1_channel_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };

    // Configuración de los canales 0 y 1
    ESP_ERROR_CHECK(
        adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_0, &adc1_channel_config)
    );

    ESP_ERROR_CHECK(
        adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_1, &adc1_channel_config)
    );
}

void ADC_leer_task(void *pvParameters)
{
    uint32_t n_muestras = *((uint32_t *)pvParameters);
    while(1)
    {
        uint32_t suma[N_CANALES] = {0};
        uint32_t promedio[N_CANALES];
        int lectura;

        for(int i = 0; i < n_muestras; i++)
        {
            for(int ch = 0; ch < N_CANALES; ch++)
            {
                ESP_ERROR_CHECK(
                    adc_oneshot_read(adc1_handle, canales[ch], &lectura)
                );
                suma[ch] += lectura;
            }
        }

        for(int ch = 0; ch < N_CANALES; ch++)
        {
            promedio[ch] = suma[ch] / n_muestras;
        }

        int comando;
        if(xQueueReceive(Com_to_adq, &comando, 0) == pdTRUE)
        {
            uint16_t valor_promedio;

            if(comando == 0)
            {
                valor_promedio = promedio[0];
                mapa_registros.valor_analogico_1 = valor_promedio;
                xQueueSend(REG40001_to_Com, &valor_promedio, 0);
            }
            else if(comando == 1)
            {
                valor_promedio = promedio[1];
                mapa_registros.valor_analogico_2 = valor_promedio;
                xQueueSend(REG40002_to_Com, &valor_promedio, 0);
            }
        }
    }
}

