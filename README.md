# embedded_project
Final project of embedded discipline, in which the main idea was to create drivers of OLED display, MPU, PWM and DS18B20 for the ESP32, and with them make a project.

## Biblioteca para o MPU6050

### Descrição Geral

Esta biblioteca foi projetada para controlar e ler dados do sensor MPU6050 (um acelerômetro e giroscópio de 6 eixos) usando comunicação I²C. Ela fornece funções para inicializar o sensor, configurar os registradores, calcular erros iniciais, ler valores brutos de aceleração e giroscópio e calcular ângulos usando o filtro de Kalman.

### Funções
1. mpu6050_init()
Descrição: Inicializa o MPU6050, configurando o registrador de gerenciamento de energia (PWR_MGMT_1) para desativar o modo de suspensão.
Entradas: Nenhuma.
Saída: Nenhuma.
Log: Exibe mensagens indicando sucesso ou falha na inicialização.

2. mpu6050_configure_sensors()

Descrição: Configura o acelerômetro (±2g) e o giroscópio (±250°/s).
Entradas: Nenhuma.
Saída: Nenhuma.
Log: Exibe mensagens indicando sucesso ou falha na configuração de cada sensor.

3. init_mpu6050()
Descrição: Chama as funções de inicialização e configuração do sensor. Calcula os erros iniciais do acelerômetro e do giroscópio com base em 500 amostras.
Entradas: Nenhuma.
Saída: Nenhuma.
Log: Exibe mensagens indicando o progresso e a conclusão do cálculo dos erros iniciais.

4. kalman_init(Kalman *kalman)
Descrição: Inicializa a estrutura de filtro de Kalman com valores padrão.
Entradas:
kalman: Ponteiro para uma estrutura Kalman.
Saída: Nenhuma.

5. kalman_update(Kalman *kalman, float new_angle, float new_rate, float dt)
Descrição: Atualiza o filtro de Kalman com novos dados para estimar o ângulo.
Entradas:
kalman: Ponteiro para a estrutura Kalman.
new_angle: Ângulo estimado a partir do acelerômetro.
new_rate: Taxa angular medida pelo giroscópio.
dt: Intervalo de tempo desde a última atualização.
Saída: Retorna o ângulo atualizado estimado pelo filtro.

6. calculate_angles_task()
Descrição: Calcula os ângulos de inclinação lateral (roll) e arfagem (pitch) usando dados do acelerômetro e giroscópio, aplicando o filtro de Kalman.
Entradas: Nenhuma.
Saída:
Estrutura Angles, contendo os ângulos roll e pitch.

7. Funções de Leitura do Sensor
a) mpu6050_read_gyro_x()
Descrição: Lê e retorna o valor do eixo X do giroscópio.
Saída: Valor do giroscópio no eixo X, em graus por segundo (°/s).
b) mpu6050_read_gyro_y()
Descrição: Lê e retorna o valor do eixo Y do giroscópio.
Saída: Valor do giroscópio no eixo Y, em graus por segundo (°/s).
c) mpu6050_read_gyro_z()
Descrição: Lê e retorna o valor do eixo Z do giroscópio.
Saída: Valor do giroscópio no eixo Z, em graus por segundo (°/s).
d) mpu6050_read_accel_x()
Descrição: Lê e retorna o valor do eixo X do acelerômetro.
Saída: Valor do acelerômetro no eixo X, em g (aceleração gravitacional).
e) mpu6050_read_accel_y()
Descrição: Lê e retorna o valor do eixo Y do acelerômetro.
Saída: Valor do acelerômetro no eixo Y, em g (aceleração gravitacional).
f) mpu6050_read_accel_z()
Descrição: Lê e retorna o valor do eixo Z do acelerômetro.
Saída: Valor do acelerômetro no eixo Z, em g (aceleração gravitacional).
Constantes Importantes
MPU6050_ADDR: Endereço do dispositivo MPU6050 no barramento I²C.
PWR_MGMT_1: Registrador para gerenciamento de energia.
ACCEL_CONFIG: Registrador de configuração do acelerômetro.
GYRO_CONFIG: Registrador de configuração do giroscópio.
RAD2DEG: Fator de conversão de radianos para graus.

Exemplo de Uso
c
void app_main() {
    // Inicializar o MPU6050
    init_mpu6050();

    // Calcular ângulos continuamente
    while (true) {
        Angles angles = calculate_angles_task();
        printf("Roll: %.2f, Pitch: %.2f\n", angles.roll, angles.pitch);
        vTaskDelay(pdMS_TO_TICKS(100)); // Atualiza a cada 100ms
    }
}

# Notas
O cálculo dos erros iniciais (accError e gyrError) deve ser feito com o sensor estático para minimizar ruídos.
A biblioteca utiliza o FreeRTOS para tarefas assíncronas (vTaskDelay).
Verifique as conexões do barramento I²C antes de utilizar o código.
