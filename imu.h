/**
 * @file imu.h
 * @author Robinson Correa Morales (robinson.corream@udea.edu.co)
 * @brief Controlador de una unidad de aceleración inercial 
 * MPU6050 por interfaz I2C
 * @version 0.1
 * @date 2026-06-09
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef IMU_MPU6050_H
#define IMU_MPU6050_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "pico/time.h"
#include <stdint.h>
#include <stdbool.h>

// Comunicación I2C
#define IMU_I2C_PORT    i2c1    /**< Instancia I2C */
#define IMU_PIN_SDA     2       /**< GPIO SDA */
#define IMU_PIN_SCL     3       /**< GPIO SLC */
#define IMU_I2C_FREQ    400000  /**< Frecuencia de comunicación */
#define IMU_SAMPLE_US   10000   /**< Periodo de Muestreo */
#define MPU6050_ADDR    0x68    /**< Direccion I2C de la IMU AD0 a GND */

//Registros
#define REG_SMPLRT_DIV   0x19   /**< Divisor de frecuencia de muestreo. */
#define REG_CONFIG       0x1A   /**< Configuración general y DLPF. */
#define REG_GYRO_CONFIG  0x1B   /**< Configuración del giroscopio. */
#define REG_ACCEL_CONFIG 0x1C   /**< Configuración del acelerómetro. */
#define REG_INT_ENABLE   0x38   /**< Habilitación de interrupciones. */
#define REG_ACCEL_XOUT_H 0x3B   /**< Byte alto de Accel X. */
#define REG_PWR_MGMT_1   0x6B   /**< Gestión de energía y reloj. */
#define REG_WHO_AM_I     0x75   /**< Identificador del dispositivo. */

//Conversiones lineales
#define GYRO_SCALE   (1.0f / 131.0f)        /**< Conversión de LSB (cuentas crudad  raw) a °/s (±250 °/s). */
#define DEG_TO_RAD   (3.14159265f / 180.0f) /**< Conversión de grados a radianes. */
#define ACCEL_SCALE  (9.80665f / 16384.0f)  /**< Conversión de LSB (cuentas crudad  raw) a m/s² (±2 g). */


/**
 * @brief Estructura de datos que expone el driver
 * 
 */
typedef struct 
{
    float gyro_x;   /** Velocidad angular en x rad/s */
    float gyro_y;   /** Velocidad angular en y rad/s */
    float gyro_z;   /** Velocidad angular en z yaw rate — el que usa el filtro complementario */
    float accel_x;  /** aceleración en x m/s² */
    float accel_y;  /** Aceleración en y m/s² */
    float accel_z;  /** Aceleración en z m/s² */
    float temp;     /** Temperatura °C */
} imu_data_t;

//-----------------── API pública ───────────────────//

/**
 * @brief Inicializa I2C, configura chip, arranca la alarma
 * 
 * @return true Se inició la IMU correctamente
 * @return false Falló la inicialización de la IMU
 */
bool imu_init();

/**
 * @brief 
 * 
 * @return true true si la alarma disparó desde la última lectura 
 * @return false 
 */
bool imu_data_ready();  /* */

/**
 * @brief Lee la IMU y entrega los resultados en unidades físicas
 * 
 * @return true La IMU se leyó correctamente
 * @return false Hubo un error en la lectura
 */
bool imu_read();

/**
 * @brief Retorna la estructura de mediciones de la IMU
 * 
 * @param out Apuntador a estructura de datos con las mediciones
 */
void imu_get_data(imu_data_t *out);
/**
 * @brief Cancela la alarma de muestreo sin tocar el chip ni el bias.
 *        Llamar antes de entrar en APP_SLEEPING / APP_ALARM.
 */
void imu_alarm_pause(void);

/**
 * @brief Reanuda la alarma de muestreo con el mismo período.
 *        Llamar al salir de APP_SLEEPING / APP_ALARM.
 */
void imu_alarm_resume(void);
#endif