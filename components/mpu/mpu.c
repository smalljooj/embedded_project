#include <stdio.h>
#include "mpu.h"

#define PWR_MGMT_1 0x6B
#define MPU6050_ADDR 0x68

#define ACCEL_CONFIG 0x1C
#define GYRO_CONFIG 0x1B

#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44

#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46

#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48

#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C

#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E

#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40

#define TAG "MPU6050"
#define RAD2DEG (180.0 / M_PI)

float Q_angle = 0.001;  // Incerteza do modelo do ângulo
float Q_bias = 0.003;   // Incerteza do viés do giroscópio
float R_measure = 0.03; // Incerteza do acelerômetro

float accRaw[3], accAngle[3], accError[3];
float gyrRaw[3], gyrAngle[3], gyrError[3];
float yaw = 0, pitch = 0, roll = 0;
uint32_t prevtime = 0, time = 0;

void scan_i2c_bus() {
    printf("Scanning I2C bus...\n");
    for (uint8_t addr = 1; addr < 127; addr++) {
        if (i2c_master_write_to_device(I2C_NUM_0, addr, NULL, 0, pdMS_TO_TICKS(100)) == ESP_OK) {
            printf("Device found at address 0x%02X\n", addr);
        }
    }
    printf("I2C scan complete.\n");
}


esp_err_t  mpu6050_init(void) {
    // Create I2C command link
    i2c_cmd_handle_t link = i2c_cmd_link_create();

    // Start I2C communication
    i2c_master_start(link);

    // Send MPU6050 address with write flag
    i2c_master_write_byte(link, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);

    // Send register address for PWR_MGMT_1
    i2c_master_write_byte(link, PWR_MGMT_1, true);

    // Send data to disable sleep mode (value 0x00)
    i2c_master_write_byte(link, 0x00, true);

    // Stop I2C communication
    i2c_master_stop(link);

    // Execute I2C command and check for success
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, link, pdMS_TO_TICKS(1000));

    // Delete I2C command link
    i2c_cmd_link_delete(link);

    if (ret == ESP_OK) {
        ESP_LOGI("mpu6050", "Wake-up successful");
    } else {
        ESP_LOGE("mpu6050", "Wake-up failed: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

esp_err_t mpu6050_configure_sensors(void) {
    // Create I2C command link
    i2c_cmd_handle_t link = i2c_cmd_link_create();

    // Configure accelerometer (ACCEL_CONFIG register)
    i2c_master_start(link);
    i2c_master_write_byte(link, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(link, ACCEL_CONFIG, true);
    i2c_master_write_byte(link, 0x00, true); // Set accelerometer to ±2g
    i2c_master_stop(link);

    // Execute I2C command for accelerometer and check for success
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, link, pdMS_TO_TICKS(1000));
    if (ret == ESP_OK) {
        ESP_LOGI("mpu6050", "Accelerometer configured successfully.");
    } else {
        ESP_LOGE("mpu6050", "Failed to configure accelerometer: %s", esp_err_to_name(ret));
    }

    // Delete I2C command link
    i2c_cmd_link_delete(link);

    // Create a new I2C command link for gyroscope configuration
    link = i2c_cmd_link_create();

    // Configure gyroscope (GYRO_CONFIG register)
    i2c_master_start(link);
    i2c_master_write_byte(link, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(link, GYRO_CONFIG, true);
    i2c_master_write_byte(link, 0x00, true); // Set gyroscope to ±250°/s
    i2c_master_stop(link);

    // Execute I2C command for gyroscope and check for success
    ret = i2c_master_cmd_begin(I2C_NUM_0, link, pdMS_TO_TICKS(1000));

    // Delete I2C command link
    i2c_cmd_link_delete(link);

    if (ret == ESP_OK) {
        ESP_LOGI("mpu6050", "Gyroscope configured successfully.");
    } else {
        ESP_LOGE("mpu6050", "Failed to configure gyroscope: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

// Inicialização do MPU6050
esp_err_t init_mpu6050() {
    esp_err_t ret = mpu6050_init();
    if(ret  != ESP_OK ){
        return ret;
    }
    ret = mpu6050_configure_sensors();
    if(ret  != ESP_OK ){
        return ret;
    }

    ESP_LOGI(TAG, "Calculando erros iniciais...");
    // Calcular erros de acelerômetro -> dispositivo deve estar parado
    // media de 500 amostras para minimizar efeito dos ruídos
    for (int i = 0; i < 500; i++) {
        accRaw[0] = mpu6050_read_accel_x();
        accRaw[1] = mpu6050_read_accel_y();
        accRaw[2] = mpu6050_read_accel_z();

        // calculando erros do angulo de roll (inclinação lateral)
        accError[0] += atan(accRaw[1] / sqrt(pow(accRaw[0], 2) + pow(accRaw[2], 2))) * RAD2DEG;
        // calculando erros do angulo de pitch (arfagem) (aviao levantando o bico)
        accError[1] += atan(-accRaw[0] / sqrt(pow(accRaw[1], 2) + pow(accRaw[2], 2))) * RAD2DEG;
    }

    // calculando os erros médios, assim removemos as imprecisões   
    accError[0] /= 500.0;
    accError[1] /= 500.0;

    // Calcular erros de giroscópio
    for (int i = 0; i < 500; i++) {
        gyrRaw[0] = mpu6050_read_gyro_x();
        gyrRaw[1] = mpu6050_read_gyro_y();
        gyrRaw[2] = mpu6050_read_gyro_z();

        gyrError[0] += gyrRaw[0];
        gyrError[1] += gyrRaw[1];
        gyrError[2] += gyrRaw[2];
    }
    // fazemos a média dos erros médios em cada eixo
    gyrError[0] /= 500.0;
    gyrError[1] /= 500.0;
    gyrError[2] /= 500.0;

    ESP_LOGI(TAG, "Erros iniciais calculados.");
    return ESP_OK;
}

// Funções para inicializar e atualizar o filtro de Kalman
void kalman_init(Kalman *kalman) {
    kalman->angle = 0.0;
    kalman->bias = 0.0;
    kalman->rate = 0.0;
    kalman->P[0][0] = 1.0;
    kalman->P[0][1] = 0.0;
    kalman->P[1][0] = 0.0;
    kalman->P[1][1] = 1.0;
}

float kalman_update(Kalman *kalman, float new_angle, float new_rate, float dt) {
    // Estágio de predição
    kalman->rate = new_rate - kalman->bias;
    kalman->angle += dt * kalman->rate;

    kalman->P[0][0] += dt * (dt * kalman->P[1][1] - kalman->P[0][1] - kalman->P[1][0] + Q_angle);
    kalman->P[0][1] -= dt * kalman->P[1][1];
    kalman->P[1][0] -= dt * kalman->P[1][1];
    kalman->P[1][1] += Q_bias * dt;

    // Estágio de atualização
    float S = kalman->P[0][0] + R_measure;
    float K[2];
    K[0] = kalman->P[0][0] / S;
    K[1] = kalman->P[1][0] / S;

    float y = new_angle - kalman->angle;
    kalman->angle += K[0] * y;
    kalman->bias += K[1] * y;

    float P00_temp = kalman->P[0][0];
    float P01_temp = kalman->P[0][1];

    kalman->P[0][0] -= K[0] * P00_temp;
    kalman->P[0][1] -= K[0] * P01_temp;
    kalman->P[1][0] -= K[1] * P00_temp;
    kalman->P[1][1] -= K[1] * P01_temp;

    return kalman->angle;
}

// Tarefa principal adaptada para o filtro de Kalman
Angles calculate_angles_task() {
    static Kalman kalman_roll, kalman_pitch;
    static bool kalman_initialized = false;

// Se os filtros ainda não foram inicializados ou os valores são inválidos, reinicializa
    if (!kalman_initialized || isnan(kalman_roll.angle) || isnan(kalman_pitch.angle)) {
        kalman_init(&kalman_roll);
        kalman_init(&kalman_pitch);
        kalman_initialized = true;
    }

    Angles angles;

    prevtime = time;
    time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    float dt = (time - prevtime) / 1000.0;

    if (dt == 0) {
    dt = 0.001; // Pequeno valor para evitar divisão por zero
    }

    // Leitura do acelerômetro
    accRaw[0] = mpu6050_read_accel_x();
    accRaw[1] = mpu6050_read_accel_y();
    accRaw[2] = mpu6050_read_accel_z();

    accAngle[0] = atan(accRaw[1] / sqrt(pow(accRaw[0], 2) + pow(accRaw[2], 2))) * RAD2DEG - accError[0];
    accAngle[1] = atan(-accRaw[0] / sqrt(pow(accRaw[1], 2) + pow(accRaw[2], 2))) * RAD2DEG - accError[1];

    // Leitura do giroscópio
    gyrRaw[0] = mpu6050_read_gyro_x() - gyrError[0];
    gyrRaw[1] = mpu6050_read_gyro_y() - gyrError[1];

    // Atualização dos ângulos usando o filtro de Kalman
    roll = kalman_update(&kalman_roll, accAngle[0], gyrRaw[0], dt);
    pitch = kalman_update(&kalman_pitch, accAngle[1], gyrRaw[1], dt);

    angles.roll = roll;
    angles.pitch = pitch;

    ESP_LOGI(TAG, "Roll: %.2f | Pitch: %.2f", roll, pitch);

    return angles;
}


float mpu6050_read_gyro_x(void) {
    uint8_t data[2];  // Para armazenar os dois bytes de GYRO_XOUT_H e GYRO_XOUT_L
    int16_t gyro_x;

    // Cria o comando I²C
    i2c_cmd_handle_t link = i2c_cmd_link_create();

    // Envia o endereço do registrador inicial (GYRO_XOUT_H)
    i2c_master_start(link);
    i2c_master_write_byte(link, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(link, GYRO_XOUT_H, true);

    // Repeated Start para leitura
    i2c_master_start(link);
    i2c_master_write_byte(link, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);

    // Lê dois bytes consecutivos (GYRO_XOUT_H e GYRO_XOUT_L)
    i2c_master_read(link, data, sizeof(data), I2C_MASTER_LAST_NACK);

    // Finaliza a comunicação
    i2c_master_stop(link);

    // Executa o comando
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, link, pdMS_TO_TICKS(1000));
    if (ret == ESP_OK) {
        // Combina os bytes lidos
        gyro_x = (int16_t)((data[0] << 8) | data[1]);
        ESP_LOGI("mpu6050", "Gyro X: %f",(float) gyro_x / (131.0));

        // Libera o comando I²C
        i2c_cmd_link_delete(link);

        return (float) gyro_x / (131.0);
    } else {

        ESP_LOGE("mpu6050", "Failed to read gyro data: %s", esp_err_to_name(ret));

        // Libera o comando I²C
        i2c_cmd_link_delete(link);

        
        return 0;
    }


}

float mpu6050_read_gyro_y(void) {
    uint8_t data[2];  // Para armazenar os dois bytes de GYRO_YOUT_H e GYRO_YOUT_L
    int16_t gyro_y;

    // Cria o comando I²C
    i2c_cmd_handle_t link = i2c_cmd_link_create();

    // Envia o endereço do registrador inicial (GYRO_YOUT_H)
    i2c_master_start(link);
    i2c_master_write_byte(link, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(link, GYRO_YOUT_H, true);

    // Repeated Start para leitura
    i2c_master_start(link);
    i2c_master_write_byte(link, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);

    // Lê dois bytes consecutivos (GYRO_YOUT_H e GYRO_YOUT_L)
    i2c_master_read(link, data, sizeof(data), I2C_MASTER_LAST_NACK);

    // Finaliza a comunicação
    i2c_master_stop(link);

    // Executa o comando
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, link, pdMS_TO_TICKS(1000));
    if (ret == ESP_OK) {
        // Combina os bytes lidos
        gyro_y = (int16_t)((data[0] << 8) | data[1]);
        ESP_LOGI("mpu6050", "Gyro Y: %f",(float) gyro_y / (131.0));

        // Libera o comando I²C
        i2c_cmd_link_delete(link);

        return (float) gyro_y / (131.0);
    } else {
        ESP_LOGE("mpu6050", "Failed to read gyro data: %s", esp_err_to_name(ret));

        // Libera o comando I²C
        i2c_cmd_link_delete(link);

        return 0;
    }

}

float mpu6050_read_gyro_z(void) {
    uint8_t data[2];  // Para armazenar os dois bytes de GYRO_ZOUT_H e GYRO_ZOUT_L
    int16_t gyro_z;

    // Cria o comando I²C
    i2c_cmd_handle_t link = i2c_cmd_link_create();

    // Envia o endereço do registrador inicial (GYRO_ZOUT_H)
    i2c_master_start(link);
    i2c_master_write_byte(link, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(link, GYRO_ZOUT_H, true);

    // Repeated Start para leitura
    i2c_master_start(link);
    i2c_master_write_byte(link, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);

    // Lê dois bytes consecutivos (GYRO_ZOUT_H e GYRO_ZOUT_L)
    i2c_master_read(link, data, sizeof(data), I2C_MASTER_LAST_NACK);

    // Finaliza a comunicação
    i2c_master_stop(link);

    // Executa o comando
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, link, pdMS_TO_TICKS(1000));
    if (ret == ESP_OK) {
        // Combina os bytes lidos
        gyro_z = (int16_t)((data[0] << 8) | data[1]);
        ESP_LOGI("mpu6050", "Gyro Z: %f", (float) gyro_z / (131.0));

        // Libera o comando I²C
        i2c_cmd_link_delete(link);

        return  (float) gyro_z / (131.0);
    } else {
        ESP_LOGE("mpu6050", "Failed to read gyro data: %s", esp_err_to_name(ret));

        // Libera o comando I²C
        i2c_cmd_link_delete(link);

        return 0;
    }


}


float mpu6050_read_accel_x(void) {
    uint8_t data[2];  // Para armazenar os dois bytes de ACCEL_XOUT_H e ACCEL_XOUT_L
    int16_t accel_x;

    // Cria o comando I²C
    i2c_cmd_handle_t link = i2c_cmd_link_create();

    // Envia o endereço do registrador inicial (ACCEL_XOUT_H)
    i2c_master_start(link);
    i2c_master_write_byte(link, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(link, ACCEL_XOUT_H, true);

    // Repeated Start para leitura
    i2c_master_start(link);
    i2c_master_write_byte(link, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);

    // Lê dois bytes consecutivos (ACCEL_XOUT_H e ACCEL_XOUT_L)
    i2c_master_read(link, data, sizeof(data), I2C_MASTER_LAST_NACK);

    // Finaliza a comunicação
    i2c_master_stop(link);

    // Executa o comando
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, link, pdMS_TO_TICKS(1000));
    if (ret == ESP_OK) {
        // Combina os bytes lidos
        accel_x = (int16_t)((data[0] << 8) | data[1]);
        ESP_LOGI("mpu6050", "Accel X: %f", (float) accel_x/ (16384));

        // Libera o comando I²C
        i2c_cmd_link_delete(link);

        return (float) accel_x / (16384);
    } else {
        ESP_LOGE("mpu6050", "Failed to read accel data: %s", esp_err_to_name(ret));

        // Libera o comando I²C
        i2c_cmd_link_delete(link);

        return 0;
    }


}

float mpu6050_read_accel_y(void) {
    uint8_t data[2];  // Para armazenar os dois bytes de ACCEL_YOUT_H e ACCEL_YOUT_L
    int16_t accel_y;

    // Cria o comando I²C
    i2c_cmd_handle_t link = i2c_cmd_link_create();

    // Envia o endereço do registrador inicial (ACCEL_YOUT_H)
    i2c_master_start(link);
    i2c_master_write_byte(link, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(link, ACCEL_YOUT_H, true);

    // Repeated Start para leitura
    i2c_master_start(link);
    i2c_master_write_byte(link, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);

    // Lê dois bytes consecutivos (ACCEL_YOUT_H e ACCEL_YOUT_L)
    i2c_master_read(link, data, sizeof(data), I2C_MASTER_LAST_NACK);

    // Finaliza a comunicação
    i2c_master_stop(link);

    // Executa o comando
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, link, pdMS_TO_TICKS(1000));
    if (ret == ESP_OK) {
        // Combina os bytes lidos
        accel_y = (int16_t)((data[0] << 8) | data[1]);
        ESP_LOGI("mpu6050", "Accel Y: %f", (float) accel_y/ (16384));

        // Libera o comando I²C
        i2c_cmd_link_delete(link);

        return (float) accel_y / (16384);
    } else {
        ESP_LOGE("mpu6050", "Failed to read accel data: %s", esp_err_to_name(ret));

        // Libera o comando I²C
        i2c_cmd_link_delete(link);

        return 0;
    }


}

float mpu6050_read_accel_z(void) {
    uint8_t data[2];  // Para armazenar os dois bytes de ACCEL_ZOUT_H e ACCEL_ZOUT_L
    int16_t accel_z;

    // Cria o comando I²C
    i2c_cmd_handle_t link = i2c_cmd_link_create();

    // Envia o endereço do registrador inicial (ACCEL_ZOUT_H)
    i2c_master_start(link);
    i2c_master_write_byte(link, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(link, ACCEL_ZOUT_H, true);

    // Repeated Start para leitura
    i2c_master_start(link);
    i2c_master_write_byte(link, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);

    // Lê dois bytes consecutivos (ACCEL_ZOUT_H e ACCEL_ZOUT_L)
    //i2c_master_read(link, data, sizeof(data), I2C_MASTER_LAST_NACK);
    i2c_master_read(link, data, sizeof(data), 0x1);


    // Finaliza a comunicação
    i2c_master_stop(link);

    // Executa o comando
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, link, pdMS_TO_TICKS(1000));
    if (ret == ESP_OK) {
        // Combina os bytes lidos
        accel_z = (int16_t)((data[0] << 8) | data[1]);
        ESP_LOGI("mpu6050", "Accel Z: %f", (float) accel_z/ (16384));

        // Libera o comando I²C
        i2c_cmd_link_delete(link);

        return (float) accel_z / (16384);
    } else {
        ESP_LOGE("mpu6050", "Failed to read accel data: %s", esp_err_to_name(ret));

        // Libera o comando I²C
        i2c_cmd_link_delete(link);

        return 0;
    }

    
}