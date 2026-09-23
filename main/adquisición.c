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

#define N_CANALES 2

adc_oneshot_unit_handle_t adc1_handle;    // Se crea una variable global
adc_cali_handle_t adc_cali_handle = NULL; // Manejador para la calibración del ADC
bool do_calibration = false;              // Bandera para indicar si se ha calibrado el ADC

adc_channel_t canales[N_CANALES] = {
    ADC_CHANNEL_0, // GPIO36
    ADC_CHANNEL_1, // GPIO37
};

void ADC1_inicializacion(void); // Función para inicializar el ADC1
void ADC_calibracion    (void); // Función para calibrar el ADC con Line Fitting Scheme
void ADC_leer_task      (void *pvParameters); // Tarea para leer el ADC en una tarea de FreeRTOS

QueueHandle_t Com_to_adq;
QueueHandle_t REG40001_to_Com;
QueueHandle_t REG40002_to_Com;

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
    ESP_ERROR_CHECK(n_muestras > 0 && n_muestras <= UINT32_MAX / 4095U
                    ? ESP_OK : ESP_ERR_INVALID_ARG);
    while(1)
    {
        int comando;
        if (xQueueReceive(Com_to_adq, &comando, portMAX_DELAY) != pdTRUE ||
            (comando != 0 && comando != 1)) {
            continue;
        }
        uint32_t suma[N_CANALES] = {0};
        uint32_t promedio[N_CANALES];
        int lectura;

        for(uint32_t i = 0; i < n_muestras; i++)
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

        uint16_t valor_promedio = promedio[comando];
        QueueHandle_t respuesta = comando == 0 ? REG40001_to_Com : REG40002_to_Com;
        xQueueSend(respuesta, &valor_promedio, portMAX_DELAY);
    }
}

