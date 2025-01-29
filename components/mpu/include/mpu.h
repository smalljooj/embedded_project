#ifndef MPU_H
#define MPU_H

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <esp_log.h>
#include <driver/i2c.h>

#include <esp_log.h>

// Estrutura para os ângulos
typedef struct {
    float roll;
    float pitch;
} Angles;

// Estrutura para manter os estados do filtro de Kalman
typedef struct {
    float angle;   // Ângulo estimado
    float bias;    // Viés estimado
    float rate;    // Taxa de giro sem viés
    float P[2][2]; // Matriz de covariância de erro
} Kalman;

void func(void);
void mpu6050_init(void);
void mpu6050_configure_sensors(void);
void init_mpu6050(void);
Angles calculate_angles_task();
void kalman_init(Kalman *kalman);
float kalman_update(Kalman *kalman, float new_angle, float new_rate, float dt);
float mpu6050_read_gyro_x(void);
float mpu6050_read_gyro_y(void);
float mpu6050_read_gyro_z(void);
float mpu6050_read_accel_x(void);
float mpu6050_read_accel_y(void);
float mpu6050_read_accel_z(void);

#endif