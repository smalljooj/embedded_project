#pragma once

#ifndef __PWM_H__
#define __PWM_H__

#include <esp_err.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief Estrutura para armazenar informações da configuração do PWM.
     *
     * Contém os parâmetros necessários para configurar e manipular um sinal PWM no ESP32.
     */
    typedef struct
    {
        uint8_t pin;         /**< Pino GPIO onde o PWM será gerado */
        uint8_t channel;     /**< Canal LEDC utilizado para o PWM */
        uint8_t resolution;  /**< Resolução do PWM em bits (ex: 10 bits -> valores de 0 a 1023) */
        uint16_t frequency;  /**< Frequência do sinal PWM em Hertz */
    } PWM;

    /**
     * @brief Inicializa o PWM no pino especificado.
     *
     * Configura o temporizador e o canal PWM com os parâmetros desejados.
     *
     * @param pwm Ponteiro para a estrutura PWM onde será armazenada a configuração.
     * @param pin Pino GPIO onde o PWM será gerado.
     * @param channel Canal do LEDC a ser utilizado.
     * @param resolution Resolução do PWM em bits.
     * @param frequency Frequência do PWM em Hz.
     * @return esp_err_t Retorna ESP_OK se a configuração for bem-sucedida.
     */
    esp_err_t pwm_init(PWM *pwm, uint8_t pin, uint8_t channel, uint8_t resolution, uint16_t frequency);

    /**
     * @brief Define o ciclo de trabalho do PWM.
     *
     * Ajusta o duty cycle do PWM, permitindo controlar a intensidade ou nível do sinal gerado.
     *
     * @param pwm Ponteiro para a estrutura PWM.
     * @param duty_cycle Valor do ciclo de trabalho (0 a 1023 para 10 bits de resolução).
     * @return esp_err_t Retorna ESP_OK se bem-sucedido, ou ESP_ERR_INVALID_ARG se o valor for inválido.
     */
    esp_err_t pwm_set_duty_cycle(PWM *pwm, uint16_t duty_cycle);

    /**
     * @brief Desliga o PWM no canal configurado.
     *
     * Para a geração do sinal PWM e define o canal no estado inativo.
     *
     * @param pwm Ponteiro para a estrutura PWM.
     * @return esp_err_t Retorna ESP_OK se bem-sucedido.
     */
    esp_err_t pwm_turn_off(PWM *pwm);

#ifdef __cplusplus
}
#endif

#endif
