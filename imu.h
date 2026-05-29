#ifndef IMU_MPU6050_H
#define IMU_MPU6050_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "pico/time.h"
#include <stdint.h>
#include <stdbool.h>

/* ── Pines I2C ────────────────────────────────────────────────────────────── */
#define IMU_I2C_PORT    i2c1
#define IMU_PIN_SDA     2
#define IMU_PIN_SCL     3
#define IMU_I2C_FREQ    400000

/* ── Periodo de muestreo ─────────────────────────────────────────────────── */
#define IMU_SAMPLE_US   10000   /* 10 000 µs = 10 ms = 100 Hz */

/* ── Dirección I2C ───────────────────────────────────────────────────────── */
#define MPU6050_ADDR    0x68    /* AD0 a GND */

/* ── Datos que expone el driver ──────────────────────────────────────────── */
typedef struct {
    float gyro_x;   /* rad/s */
    float gyro_y;   /* rad/s */
    float gyro_z;   /* yaw rate — el que usa el filtro complementario */
    float accel_x;  /* m/s²  */
    float accel_y;  /* m/s²  */
    float accel_z;  /* m/s²  */
    float temp;     /* °C    */
} imu_data_t;

/* ── API pública ─────────────────────────────────────────────────────────── */
bool imu_init(void);        /* Inicializa I2C, configura chip, arranca alarma */
bool imu_data_ready(void);  /* true si la alarma disparó desde la última lectura */
bool imu_read(void);        /* Lee 14 bytes I2C y convierte — llamar desde main */
void imu_get_data(imu_data_t *out);

#endif