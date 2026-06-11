/**
* @file speedControl.h
* @author Robinson Correa Morales (robinson.corream@udea.edu.co)
* @brief Controlador diferencial de velocidad
* 
* Objetivo:
* 
*  Garantizar que ambas orugas giren a la misma velocidad real 
*  garantizando movimiento en linea recta, compensando las diferencias físicas 
*   entre motores con un control proporcional (P)
*  de lazo cerrado sobre ticks de encoder.
*
*  Relación con los módulos de bajo nivel:
*
*  Lee : encoder_get_left(), encoder_get_right()  [encoder.h]
*  Escribe : motors_set() [Driver_TB6612FNG.h]
*  Llamado por : odometry_update() o el main, a 100 Hz
*
*  Filosofía de diseño:
*
*  La referencia de velocidad se expresa en mm/s y se convierte
*  internamente a ticks/ciclo usando mm_per_tick. Esto desacopla
*  la capa de navegación de los parámetros físicos del encoder.
*
*  El PWM resultante se calcula así en cada ciclo:
*
*    ticks_objetivo = (v_ref_mm_s * ODO_DT) / mm_per_tick
*
*    error_L = ticks_objetivo_L - delta_ticks_L
*    error_R = ticks_objetivo_R - delta_ticks_R
*
*    integral_L += error_L  (si |error_L| > DEADBAND)
*    integral_R += error_R  (si |error_R| > DEADBAND)
*
*    pwm_L = FF_SLOPE_LEFT  * |target_L| + Kp*error_L + Ki*integral_L
*    pwm_R = FF_SLOPE_RIGHT * |target_R| + Kp*error_R + Ki*integral_R
*
*  El feed-forward por canal (FF_SLOPE_LEFT / FF_SLOPE_RIGHT) absorbe
*  la diferencia DC estática entre motores. El PI elimina la varianza
*  corrida a corrida.
*
* @version 0.1
* @date 2026-06-09
* 
* @copyright Copyright (c) 2026
* 
*/

#ifndef SPEED_CONTROL_H
#define SPEED_CONTROL_H

#include "pico/stdlib.h"
#include <stdint.h>
#include <stdbool.h>

#define SC_WINDOW_CYCLES  3   /**< Número de ciclos del controlador que se acumulan */

//----------------- Constantes PI ---------------------//

#define SC_KP 0.4f  /**< Ganancia Proporcional */
#define SC_KI 0.07f /**< Ganancia Integral */

#define SC_FF_SLOPE_LEFT 7.5f  /**<  Pendiente feed-forward por canal [PWM% / (tick/ciclo)] */
#define SC_FF_SLOPE_RIGHT 7.5f /**<  Pendiente feed-forward por canal [PWM% / (tick/ciclo)] */

/**
 *@brief SC_INTEGRAL_DEADBAND [ticks]
 *El integrador solo actúa cuando |error| supera este umbral.
 *Ajustado a 0.5 para corregir pequeños sesgos entre canales.
 */
#define SC_INTEGRAL_DEADBAND 1.0f
#define SC_INTEGRAL_LIMIT 15.0f   /**< Limite del integrador */
#define SC_MAX_CORRECTION 10.0f   /**< Corrección total máxima %PWM */
#define SC_DT 0.01f               /**< Periodo de muestreo Coincidente con IMU_SAMPLE_US */


/**
 *@brief Comando de velocidad para el robot
 *
 *Para movimiento recto: v_left_mm_s == v_right_mm_s == V
 *Para giro en el lugar: v_left_mm_s == -v_right_mm_s
 *Para curva: valores distintos con el mismo signo
 *
 *Velocidades negativas producen movimiento hacia atrás.
 *Cero en ambos genera freno activo.
 */
typedef struct 
{
    float v_left_mm_s;  /**< Velocidad lineal de la oruga izquierda en mm/s */
    float v_right_mm_s; /**< Velocidad lineal de la oruga derecha en mm/s */
} sc_command_t;

/**
 *@brief Estado de diagnóstico del controlador
 */
typedef struct 
{
    float ticks_target_left;    /**< ticks/ciclo objetivo canal izquierdo */
    float ticks_target_right;   /**< ticks/ciclo objetivo canal derecho */
    int32_t ticks_actual_left;  /**< ticks reales del último ciclo */
    int32_t ticks_actual_right; /**< ticks reales del último ciclo */
    float error_left;           /**< Error izquierdo = objetivo - real */
    float error_right;          /**< Error derecho = objetivo - real */
    int pwm_left;               /**< PWM% izquierdo aplicado [-100, 100] */
    int pwm_right;              /**< PWM% izquierdo aplicado [-100, 100] */
} sc_state_t;

//--------------- Funciones Públicas --------------------//

/**
 * @brief Inicializa el controlador con los parámetros físicos del robot.
 * 
 * @param mm_per_tick Distancia en mm por pulso de encoder.
 */
void speed_control_init(float mm_per_tick);

/**
 * @brief Establece el comando de velocidad objetivo.
 *  El controlador usa estos valores en los próximos ciclos.
 * 
 * @param cmd Apuntador a comando de velocidad 
 */
void speed_control_set(const sc_command_t *cmd);

/**
 * @brief Ejecuta un ciclo del controlador P. Llamar a 100 Hz desde el main,
 *  justo después de que imu_data_ready() devuelva true (mismo ciclo
 *  que odometry_update()) para que los valores sean los reales.
 *
 *  Internamente:
 *    1. Lee delta de ticks desde la última llamada (atómico).
 *    2. Calcula ticks objetivo a partir del comando actual.
 *    3. Calcula error por canal.
 *    4. Aplica feed-forward + corrección P.
 *    5. Satura y aplica via motors_set().
 * 
 */
void speed_control_update();

/**
 * @brief Copia el estado diagnóstico del último ciclo en *out*.
 *  Usar solo para tuning y logging.
 * 
 * @param out Apuntador a estado de diagnóstico del controlador 
 */
void speed_control_get_state(sc_state_t *out);

/**
 * @brief Freno inmediato: pone el comando a cero y llama motors_set(0,0)
 *  directamente sin esperar al próximo ciclo.
 * 
 */
void speed_control_stop();

#endif