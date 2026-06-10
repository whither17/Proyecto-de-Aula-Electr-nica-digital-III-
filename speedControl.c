//Módulo 4: Controlador diferencial de velocidad

#include "speedControl.h"
#include "encoder.h"
#include "Driver_TB6612FNG.h"
#include <math.h>
#include <stdlib.h>


static int32_t g_accum_left = 0;    /**< Acumulador de ticks del encoder izquierdo */
static int32_t g_accum_right = 0;   /**< Acumulador de ticks del encoder derecho */
static int g_window_count = 0;      /**< Contador de ciclos transcurridos dentro de la ventana de acumulación [0, SC_WINDOW_CYCLES] */
static float g_mm_per_tick = 1.0f;  /**< Distancia lineal recorrida por tick de encoder [mm/tick] */

static volatile float g_target_left_mm_s = 0.0f;  /**< Velocidad objetivo del motor izquierdo [mm/s] */
static volatile float g_target_right_mm_s = 0.0f; /**< Velocidad objetivo del motor derecho [mm/s] */

// Ticks del ciclo anterior — para calcular delta 
static int32_t g_prev_left = 0;  /**< Lectura de encoder izquierdo del ciclo anterior */
static int32_t g_prev_right = 0; /**< Lectura del encoder derecho del ciclo anterior */

// Integradores por canal
static float g_integral_left = 0.0f;  /**< Acumulador del término integral del canal izquierdo */
static float g_integral_right = 0.0f; /**< Acumulador del término integral del canal derecho */

// Estado diagnóstico del último ciclo
static sc_state_t g_state = {0};      /**< Snapshot diagnóstico del último ciclo de control completo */


//---------------- Funciones Privadas -----------------//

/**
 * @brief Satura un valor flotante al intervalo [lo, hi].
 *
 * @param v Valor a saturar.
 * @param lo Límite inferior.
 * @param hi Límite superior.
 * @return float v saturado al intervalo [@p lo, @p hi].
 */
static inline float clampf(float v, float lo, float hi) 
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/**
 * @brief Convierte una velocidad lineal [mm/s] a ticks por ciclo de control.
 *
 * Utiliza la constante de periodo @c SC_DT y la resolución física del encoder
 * almacenada en @c g_mm_per_tick.
 *
 * @param v_mm_s Velocidad lineal en mm/s.
 * @return float Ticks equivalentes por ciclo de control [ticks/ciclo].
 */
static inline float mm_s_to_ticks_per_cycle(float v_mm_s) 
{
    return (v_mm_s * SC_DT) / g_mm_per_tick;
}

/**
 * @brief Calcula el PWM de un canal mediante control PI con feed-forward.
 *
 * Implementa el siguiente esquema por canal:
 *  - Feed-forward proporcional a |target_per_cycle| para reducir el sesgo DC.
 *  - Término proporcional sobre el error de la ventana acumulada.
 *  - Término integral con deadband (anti-ruido de cuantización) y anti-windup.
 *  - Corrección total saturada a +-SC_MAX_CORRECTION antes de sumarse al FF.
 *  - PWM final saturado a [-100, 100].
 *
 * @param target_ticks Objetivo acumulado en la ventana [ticks/ventana].
 * @param target_per_cycle Objetivo por ciclo individual [ticks/ciclo],
 * usado exclusivamente para el cálculo del FF.
 * @param actual_ticks Ticks reales medidos en la ventana actual.
 * @param ff_slope Pendiente feed-forward calibrada para este canal
 * [%PWM / (ticks/ciclo)].
 * @param integral Puntero al acumulador integral del canal.
 * Se actualiza in-place en cada llamada.
 * @param out_error Puntero donde se escribe el error del ciclo
 * (target_ticks − actual_ticks) para diagnóstico.
 * @return int Duty cycle PWM con signo en el rango [-100, 100].
 */
static int compute_pwm_for_channel(float target_ticks, float   target_per_cycle,
            int32_t actual_ticks, float ff_slope, float *integral, float *out_error) 
{

    float error = target_ticks - (float)actual_ticks;
    *out_error = error;

    if (fabsf(error) > SC_INTEGRAL_DEADBAND * SC_WINDOW_CYCLES) 
    {
        *integral += error;
    }
    *integral = clampf(*integral, -SC_INTEGRAL_LIMIT, SC_INTEGRAL_LIMIT);

    float ff = ff_slope * fabsf(target_per_cycle);
    float correction = clampf(SC_KP * error + SC_KI * (*integral), -SC_MAX_CORRECTION, SC_MAX_CORRECTION);
    float sign = (target_ticks >= 0.0f) ? 1.0f : -1.0f;
    float pwm_f = sign * ff + correction;

    return (int)clampf(pwm_f, -100.0f, 100.0f);
}

//------------------ Funciones Públicas --------------------------//

void speed_control_init(float mm_per_tick) 
{
    g_mm_per_tick = (mm_per_tick > 0.0f) ? mm_per_tick : 1.0f;

    g_prev_left = encoder_get_left();
    g_prev_right = encoder_get_right();

    g_target_left_mm_s = 0.0f;
    g_target_right_mm_s = 0.0f;

    g_integral_left  = 0.0f;
    g_integral_right = 0.0f;
    g_accum_left = 0;
    g_accum_right = 0;
    g_window_count = 0;
}

void speed_control_set(const sc_command_t *cmd) 
{
    g_target_left_mm_s = cmd->v_left_mm_s;
    g_target_right_mm_s = cmd->v_right_mm_s;
}

void speed_control_update() 
{
    // Freno activado: Caso especial
    if (g_target_left_mm_s == 0.0f && g_target_right_mm_s == 0.0f) 
    {
        motors_set(0, 0);

        g_prev_left = encoder_get_left();
        g_prev_right = encoder_get_right();

        //Resetear integradores al frenar para no arrancar el próximo
        //movimiento con una corrección acumulada del anterior
        g_integral_left = 0.0f;
        g_integral_right = 0.0f;

        g_state.ticks_target_left = 0.0f;
        g_state.ticks_target_right = 0.0f;
        g_state.ticks_actual_left = 0;
        g_state.ticks_actual_right = 0;
        g_state.error_left = 0.0f;
        g_state.error_right = 0.0f;
        g_state.pwm_left = 0;
        g_state.pwm_right = 0;
        return;
    }
    /* Pasos */

    // PASO 1: Delta de ticks

    /* DESPUÉS */
    int32_t cur_left = encoder_get_left();
    int32_t cur_right = encoder_get_right();

    g_accum_left += cur_left - g_prev_left;
    g_accum_right += cur_right - g_prev_right;

    g_prev_left = cur_left;
    g_prev_right = cur_right;

    g_window_count++;
    if (g_window_count < SC_WINDOW_CYCLES) 
        return;

    int32_t delta_left = g_accum_left;
    int32_t delta_right = g_accum_right;
    g_accum_left = 0;
    g_accum_right = 0;
    g_window_count = 0;

    // PASO 2: Convertir mm/s a ticks/ciclo 
    // PASO 2 — ajustar el target a la ventana
    // En PASO 2, calcular ambos
    float target_per_cycle_L = mm_s_to_ticks_per_cycle(g_target_left_mm_s);
    float target_per_cycle_R = mm_s_to_ticks_per_cycle(g_target_right_mm_s);

    float target_ticks_L = target_per_cycle_L * SC_WINDOW_CYCLES;
    float target_ticks_R = target_per_cycle_R * SC_WINDOW_CYCLES;

    // PASO 3: PI + FF por canal

    float error_L, error_R;

    int pwm_L = compute_pwm_for_channel(
        target_ticks_L, target_per_cycle_L, delta_left,
        SC_FF_SLOPE_LEFT, &g_integral_left, &error_L
    );

    int pwm_R = compute_pwm_for_channel(
        target_ticks_R, target_per_cycle_R, delta_right,
        SC_FF_SLOPE_RIGHT, &g_integral_right, &error_R
    );

    // PASO 4: Aplicar

    motors_set(pwm_L, pwm_R);

    // Diagnóstico

    g_state.ticks_target_left = target_ticks_L;
    g_state.ticks_target_right = target_ticks_R;
    g_state.ticks_actual_left = delta_left;
    g_state.ticks_actual_right = delta_right;
    g_state.error_left = error_L;
    g_state.error_right = error_R;
    g_state.pwm_left = pwm_L;
    g_state.pwm_right = pwm_R;
}

void speed_control_get_state(sc_state_t *out) 
{
    *out = g_state;
}

void speed_control_stop() 
{
    g_target_left_mm_s = 0.0f;
    g_target_right_mm_s = 0.0f;
    motors_set(0, 0);
    g_prev_left = encoder_get_left();
    g_prev_right = encoder_get_right();
    g_integral_left = 0.0f;
    g_integral_right = 0.0f;
    g_accum_left = 0;
    g_accum_right = 0;
    g_window_count = 0;
}