/**
 * @file Driver_TB6612FNG.h
 * @author Andrés Felipe Agudelo Zapata (andresf.agudeloz@udea.edu.co)
 * @brief Controlador para un driver de motores TB6612FNG
 * @version 0.1
 * @date 2026-06-09
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef DRIVER_TB6612FNG__H
#define DRIVER_TB6612FNG__H
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"
#include <stdint.h>
#include <stdlib.h>


//Motor izquierdo — canal A
#define PIN_AIN1 19  /**< GP20 — dirección izquierdo bit-0 */
#define PIN_AIN2 18  /**< GP21 — dirección izquierdo bit-1 */
#define PIN_PWMA 13  /**< GP12 — PWM slice 6, canal A */

// Motor derecho — canal B 
#define PIN_BIN1 20  /**< GP19 — dirección derecho bit-0 */
#define PIN_BIN2 21  /**< GP18 — dirección derecho bit-1 */
#define PIN_PWMB 12  /**< GP13 — PWM slice 6, canal B */

// CONSTANTES 
#define MAXPWM 70       /**< Los motores solo reciben hasta 6v la alimentacion es de 7.4v con valores de hasta 8.4v usamos un 70% como segurridad */
#define PWM_WRAP 12499  /**< Para una frecuencia de 10 KHz con reloj del micro de 125 MHz */
#define PWMSlice 6      /**< Número de slice PWM utilizado para controlar los GPIO 12 y 13 */

#define MAXPWM 70        /**< % — límite superior absoluto de velocidad */
#define MINPWM_LEFT  38  /**< % — zona muerta motor izquierdo */
#define MINPWM_RIGHT 38  /**< % — zona muerta motor derecho */


/**
 * @brief Inicializa el Driver, configura los pines GPIO, los canales PWM
 * y la interrupción de PWM
 * 
 * 
 */
void motor_driver_init();

/**
 * @brief Actualiza la consigna de velocidad de ambos motores.
 *
 * Aplica una zona muerta independiente para cada motor. Si la
 * magnitud de la velocidad solicitada es inferior al umbral
 * mínimo calibrado, el motor se detiene.
 *
 * Para velocidades válidas, configura la dirección de giro y
 * calcula el duty cycle PWM correspondiente. Los nuevos valores
 * se aplican posteriormente desde la ISR de PWM para mantener
 * la actualización sincronizada con el período PWM.
 *
 * @param left_speed  Consigna del motor izquierdo.
 * @param right_speed Consigna del motor derecho.
 */
void motors_set(int left_speed, int right_speed);

/**
 * @brief Marcha hacia adelante
 * 
 * @param speed Consigna de velocidad normalizada [-100, 100].
 */
static inline void GoForward(int speed) 
{
    motors_set(speed, speed);
}

/**
 * @brief Marcha hacia atrás
 * 
 * @param speed Consigna de velocidad normalizada [-100, 100].
 */
static inline void GoBackwards(int speed) 
{
    motors_set(-speed, -speed);
}

/**
 * @brief Giro a la izquierda
 * 
 * @param speed Consigna de velocidad normalizada [-100, 100].
 */
static inline void SpinLeft(int speed) 
{
    motors_set(-speed, speed);
}

/**
 * @brief Giro a la derecha
 * 
 * @param speed Consigna de velocidad normalizada [-100, 100].
 */
static inline void SpinRight(int speed) 
{
    motors_set(speed, -speed);
}

/**
 * @brief Detiene los motores
 * 
 */
static inline void motors_stop() 
{
    motors_set(0, 0);
}

/**
 * @brief Obtiene el número de períodos PWM transcurridos.
 * Devuelve el contador de eventos de wrap acumulados por la ISR
 * del PWM desde la inicialización del controlador
 *
 * La lectura se realiza de forma atómica deshabilitando
 * temporalmente la interrupción PWM.
 *
 * @return Número de eventos PWM wrap acumulados.
 */
uint32_t motor_get_ticks(void);

#endif