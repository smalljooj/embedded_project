# Biblioteca PWM para Relé de Estado Sólido

Esta biblioteca fornece uma interface para controle de um Relé de Estado Sólido (SSR) via PWM utilizando o ESP32. Com ela, é possível inicializar o PWM, configurar a frequência e o ciclo de trabalho, e desativar o relé.

## Funcionalidades
- Inicializar o PWM para controle do Relé de Estado Sólido.
- Ajustar o ciclo de trabalho do PWM.
- Desligar o relé.

## Dependências
A biblioteca utiliza os seguintes componentes:

- `esp_err.h`: Para tratamento de erros.
- `driver/gpio.h`: Para controle de GPIO.
- `driver/ledc.h`: Para geração de PWM com o LEDC.

## Como Usar a Biblioteca

### Passo 1: Inicializar o PWM no ESP32
Antes de utilizar o relé, é necessário configurar corretamente o driver PWM no ESP32. Abaixo está um exemplo de inicialização:

```c
#include "pwm.h"

void app_main() {
    PWM pwm1;
    pwm_init(&pwm1, GPIO_NUM_2, LEDC_CHANNEL_0, LEDC_TIMER_10_BIT, 1000);
}
```

### Passo 2: Ajustar o Ciclo de Trabalho do PWM
Após a inicialização, é possível ajustar o duty cycle para controlar o estado do relé.

```c
#include "pwm.h"

void app_main() {
    PWM pwm1;
    pwm_init(&pwm1, GPIO_NUM_2, LEDC_CHANNEL_0, LEDC_TIMER_10_BIT, 1000);
    
    for (size_t i = 0; i < 1024; i++) {
        pwm_set_duty_cycle(&pwm1, i);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
```

### Passo 3: Desligar o PWM
Caso seja necessário desligar o relé, utilize a função `pwm_turn_off`.

```c
#include "pwm.h"

void app_main() {
    PWM pwm1;
    pwm_init(&pwm1, GPIO_NUM_2, LEDC_CHANNEL_0, LEDC_TIMER_10_BIT, 1000);
    pwm_turn_off(&pwm1);
}
```

## Definições

### Pino do Relé
O pino utilizado no ESP32 para controle do relé deve ser especificado no momento da inicialização. Neste exemplo, estamos utilizando o `GPIO_NUM_2`.

```c
#define PWM_GPIO_PIN GPIO_NUM_2
```

### Estrutura de Dados

```c
typedef struct {
    uint8_t pin;        // Pino GPIO utilizado para o PWM
    uint8_t channel;    // Canal do LEDC
    uint8_t resolution; // Resolução do PWM (bits)
    uint16_t frequency; // Frequência do PWM (Hz)
} PWM;
```

Essa estrutura é usada para armazenar as configurações do PWM.

## Funções

### `pwm_init`
Inicializa o PWM e configura os parâmetros necessários.

#### Protótipo
```c
esp_err_t pwm_init(PWM *pwm, uint8_t pin, uint8_t channel, uint8_t resolution, uint16_t frequency);
```

#### Parâmetros
- `pwm`: Ponteiro para a estrutura `PWM`.
- `pin`: Pino GPIO utilizado para o PWM.
- `channel`: Canal do LEDC utilizado.
- `resolution`: Resolução do PWM.
- `frequency`: Frequência do PWM.

#### Retorno
- `ESP_OK`: Sucesso.
- Outros valores: Erro durante a inicialização.

### `pwm_set_duty_cycle`
Ajusta o ciclo de trabalho do PWM.

#### Protótipo
```c
esp_err_t pwm_set_duty_cycle(PWM *pwm, uint16_t duty_cycle);
```

#### Parâmetros
- `pwm`: Ponteiro para a estrutura `PWM`.
- `duty_cycle`: Valor do ciclo de trabalho (0 a 1023 para 10 bits).

#### Retorno
- `ESP_OK`: Sucesso.
- Outros valores: Erro ao definir o duty cycle.

### `pwm_turn_off`
Desativa o PWM e desliga o relé.

#### Protótipo
```c
esp_err_t pwm_turn_off(PWM *pwm);
```

#### Parâmetros
- `pwm`: Ponteiro para a estrutura `PWM`.

#### Retorno
- `ESP_OK`: Sucesso.

## Modo de Uso
O código abaixo exemplifica o uso da biblioteca para controle do Relé de Estado Sólido:

```c
#include <stdio.h>
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "pwm.h"    // Componente PWM

#define PWM1_PSSR_GPIO_PIN GPIO_NUM_2 // Definição do pino GPIO usado para PWM
#define STACK_SIZE 4096               // Tamanho da pilha para a tarefa PWM

static const char *TAG = "PWM";

void pwm_task(void *pvParameter)
{
    ESP_LOGW(TAG, "pwm_task() called");
    
    #ifdef CONFIG_IDF_TARGET_ESP32
    ESP_LOGI(TAG, "ESP32 detected");
    #else
    ESP_LOGI(TAG, "ESP32 not detected");
    #endif

    // Inicializa o PWM no pino definido
    PWM pwm1;
    pwm_init(&pwm1, PWM1_PSSR_GPIO_PIN, LEDC_CHANNEL_0, LEDC_TIMER_10_BIT, 1000);
    
    uint16_t pwm1_value = 0; // Variável para armazenar o ciclo de trabalho do PWM
    
    while (1)
    {
        // Loop para aumentar gradualmente o ciclo de trabalho
        for (size_t i = 0; i < 1024; i++)
        {
            pwm1_value = i; // Define o ciclo de trabalho
            pwm_set_duty_cycle(&pwm1, pwm1_value);
            vTaskDelay(10 / portTICK_PERIOD_MS); // Pequeno atraso para visualização gradual
        }
        vTaskDelay(100 / portTICK_PERIOD_MS); // Pausa para evitar consumo excessivo da CPU
    }
}

void app_main(void)
{
    ESP_LOGW(TAG, "app_main() called");

    // Configuração do temporizador para o PWM
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE, // Modo de baixa velocidade
        .duty_resolution  = LEDC_TIMER_10_BIT,   // Resolução de 10 bits
        .timer_num        = LEDC_TIMER_0,        // Timer 0
        .freq_hz          = 1000,                // Frequência de 1kHz
        .clk_cfg          = LEDC_AUTO_CLK        // Configuração automática de clock
    };

    if (ledc_timer_config(&ledc_timer) != ESP_OK) {
        ESP_LOGE(TAG, "Erro ao configurar o temporizador do LEDC!");
    }

    xTaskCreate(pwm_task, "pwm_task", STACK_SIZE, NULL, 1, NULL);
}
```


