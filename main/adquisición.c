#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

#include "esp_adc/adc_oneshot.h"          // Libreria para el uso del ADC
#include "esp_log.h"                      // Libreria para la calibración del ADC
#include "esp_adc/adc_cali.h"             // Libreria para la calibración del ADC
#include "esp_adc/adc_cali_scheme.h"      // Libreria para la calibración del ADC

#include <stdint.h>

#include "comm.h"

// --------------------------- TAREA DE ADQUISICIÓN -----------------------------------

adc_oneshot_unit_handle_t adc1_handle;    // Se crea una variable global
adc_cali_handle_t adc_cali_handle = NULL; // Manejador para la calibración del ADC
bool do_calibration = false;              // Bandera para indicar si se ha calibrado el ADC

// ADC0: ADC1 canal 0, GPIO36, pin fisico 3 del conector J2 de la DevKitC.
static const adc_channel_t canal_adc = ADC_CHANNEL_0;

void ADC1_inicializacion(void); // Función para inicializar el ADC1
void ADC_calibracion    (void); // Función para calibrar el ADC con Line Fitting Scheme
void ADC_leer_task      (void *pvParameters); // Tarea para leer el ADC en una tarea de FreeRTOS

QueueHandle_t Com_to_adq;
QueueHandle_t REG40001_to_Com;

// ------------------------------------------------------------------------------------

void ADC_calibracion(void) // Función para calibrar el ADC con Line Fitting Scheme
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

    // Configuracion del ADC0 solicitado.
    ESP_ERROR_CHECK(
        adc_oneshot_config_channel(adc1_handle, canal_adc, &adc1_channel_config)
    );
}

void ADC_leer_task(void *pvParameters)
{
    uint32_t n_muestras = *((uint32_t *)pvParameters);
    ESP_ERROR_CHECK(n_muestras > 0 && n_muestras <= UINT32_MAX / 4095U
                    ? ESP_OK : ESP_ERR_INVALID_ARG);
    while(1)
    {
        int comando;
        if (xQueueReceive(Com_to_adq, &comando, portMAX_DELAY) != pdTRUE ||
            comando != 0) {
            continue;
        }
        uint32_t suma = 0;
        int lectura;

        for(uint32_t i = 0; i < n_muestras; i++)
        {
            ESP_ERROR_CHECK(
                adc_oneshot_read(adc1_handle, canal_adc, &lectura)
            );
            suma += lectura;
        }

        uint16_t valor_promedio = suma / n_muestras;
        xQueueSend(REG40001_to_Com, &valor_promedio, portMAX_DELAY);
    }
}

