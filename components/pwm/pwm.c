#include <stdio.h>
#include <math.h>
#include "pwm.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "sdkconfig.h"

// Definições de constantes para configuração do PWM
#define PWM_CHANNEL_DEFAULT LEDC_CHANNEL_0  // Canal padrão do PWM
#define PWM_FREQ_HZ_DEFAULT 1000            // Frequência padrão de 1kHz
#define PWM_RESOLUTION_DEFAULT LED_TIMER_10_BIT  // Resolução de 10 bits (valores de 0 a 1023)
#define LEDC_CHANNEL_IDLE_LOW 0   // Estado do canal PWM quando o LEDC está inativo (LOW)
#define LEDC_CHANNEL_IDLE_HIGH 1  // Estado do canal PWM quando o LEDC está inativo (HIGH)

// Definição de uma tag para logs
static const char *TAG = "PWM";

/**
 * @brief Inicializa o PWM no ESP32.
 *
 * Configura o temporizador e o canal PWM para o pino especificado.
 *
 * @param pwm Ponteiro para a estrutura PWM.
 * @param pin Pino GPIO onde o PWM será gerado.
 * @param channel Canal do LEDC que será usado.
 * @param resolution Resolução do PWM (ex: 10 bits).
 * @param frequency Frequência do PWM (ex: 1kHz).
 * @return esp_err_t Retorna ESP_OK se a inicialização for bem-sucedida.
 */
esp_err_t pwm_init(PWM *pwm, uint8_t pin, uint8_t channel, uint8_t resolution, uint16_t frequency)
{
    ESP_LOGW(TAG, "pwm_init() called");

    ledc_timer_config_t cfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,  // Modo de baixa velocidade (baixo consumo de energia)
        .duty_resolution = resolution,      // Define resolução
        .timer_num = LEDC_TIMER_0,          // Usa o Timer 0
        .freq_hz = frequency,               // Define a frequência do PWM
        .clk_cfg = LEDC_AUTO_CLK            // Configuração automática do clock
    };
    ledc_timer_config(&cfg);                // Aplica a configuração do temporizador

    ledc_channel_config_t ledc_channel = {
        .gpio_num = pin,                    // Define o pino GPIO para saída do PWM
        .speed_mode = LEDC_LOW_SPEED_MODE,  // Modo de baixa velocidade
        .channel = channel,                 // Define o canal de saída
        .timer_sel = LEDC_TIMER_0,          // Associa ao Timer 0 configurado anteriormente
        .duty = 0,                          // Inicializa com ciclo de trabalho 0%
        .hpoint = 0                         // Ponto de início do PWM
    };
    ledc_channel_config(&ledc_channel);     // Aplica a configuração do canal PWM

    // Armazena os valores da configuração na estrutura PWM
    pwm->pin = pin;
    pwm->channel = channel;
    pwm->resolution = resolution;
    pwm->frequency = frequency;
    pwm->max_duty = pow(2, resolution) - 1;

    return ESP_OK; // Retorna sucesso
}

/**
 * @brief Define o ciclo de trabalho do PWM.
 *
 * @param pwm Ponteiro para a estrutura PWM.
 * @param duty_cycle Valor do ciclo de trabalho (0 a 1023 para 10 bits).
 * @return esp_err_t Retorna ESP_OK se bem-sucedido, ou ESP_ERR_INVALID_ARG se o valor for inválido.
 */
esp_err_t pwm_set_duty_cycle(PWM *pwm, uint16_t duty_cycle)
{
    ESP_LOGW(TAG, "pwm_setPWM() called, duty_cycle: %d", duty_cycle);

    // Verifica se o valor do duty cycle é válido
    if (duty_cycle > pwm->max_duty)
    {
        ESP_LOGE(TAG, "pwm_set_duty_cycle() called with invalid duty_cycle value: %d", duty_cycle);
        return ESP_ERR_INVALID_ARG;
    }
    // Se o ciclo de trabalho for 0, desliga o PWM
    else if (duty_cycle == 0)
    {
        ESP_LOGI(TAG, "ESP32");
        ledc_stop(LEDC_LOW_SPEED_MODE, pwm->channel, LEDC_CHANNEL_IDLE_HIGH);
    }
    // Caso contrário, define o ciclo de trabalho normalmente
    else
    {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, pwm->channel, duty_cycle);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, pwm->channel);
    }
    return ESP_OK;
}

/**
 * @brief Desliga o PWM.
 *
 * Interrompe a geração do sinal PWM no canal configurado.
 *
 * @param pwm Ponteiro para a estrutura PWM.
 * @return esp_err_t Retorna ESP_OK se bem-sucedido.
 */
esp_err_t pwm_turn_off(PWM *pwm)
{
    ESP_LOGW(TAG, "pwm_turn_off called");
    ledc_stop(LEDC_LOW_SPEED_MODE, pwm->channel, LEDC_CHANNEL_IDLE_HIGH);
    return ESP_OK;
}
