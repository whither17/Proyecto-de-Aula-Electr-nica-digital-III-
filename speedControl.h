/**
 * =============================================================================
 *  speed_control.h
 *  Robot autónomo de mapeo — Módulo 4: Controlador diferencial de velocidad
 * =============================================================================
 *
 *  Objetivo
 *  ────────
 *  Garantizar que ambas orugas giren a la misma velocidad real cuando se
 *  pide movimiento recto, compensando las diferencias físicas entre motores
 *  (rozamiento, devanado, carga mecánica) con un control proporcional (P)
 *  de lazo cerrado sobre ticks de encoder.
 *
 *  Relación con otros módulos
 *  ──────────────────────────
 *  Lee      : encoder_get_left(), encoder_get_right()  [encoder.h]
 *  Escribe  : motors_set()                             [Driver_TB6612FNG.h]
 *  Llamado por : odometry_update() o el main, a 100 Hz
 *
 *  Filosofía de diseño
 *  ───────────────────
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
 *  Parámetros a calibrar
 *  ─────────────────────
 *  SC_FF_SLOPE_LEFT/RIGHT : calibrados empíricamente en 3.88 para
 *                           100 mm/s → ~5.9 ticks/ciclo con estos motores.
 *                           Si un canal deriva, ajustar su slope ±0.1.
 *
 *  SC_KP          : ganancia proporcional. 1.5 es el valor calibrado.
 *
 *  SC_KI          : ganancia integral. 0.4 con deadband 0.5 absorbe
 *                   la varianza real entre corridas sin acumular ruido.
 *
 * =============================================================================
 */

#ifndef SPEED_CONTROL_H
#define SPEED_CONTROL_H

#include "pico/stdlib.h"
#include <stdint.h>
#include <stdbool.h>

/* ─────────────────────────────────────────────────────────────────────────────
   PARÁMETROS CONFIGURABLES
   ───────────────────────────────────────────────────────────────────────────── */

/*
 *  SC_KP — Ganancia proporcional [PWM% / tick]
 *
 *  Con el feed-forward bien centrado en 3.88, el error típico por ciclo
 *  es ±1 tick. KP=1.5 da una corrección de ±1.5 PWM%, suficiente para
 *  reaccionar rápido sin oscilar.
 *
 *  ↑ Subir si el robot aún serpentea despacio entre correcciones.
 *  ↓ Bajar si hay oscilación rápida o ruido mecánico audible.
 */
#define SC_KP             0.3f

/*
 *  SC_KI — Ganancia integral [PWM% / (tick·ciclo)]
 *
 *  Absorbe la varianza corrida a corrida: diferencias en temperatura,
 *  carga de batería, superficie. Con FF centrado el error acumulado
 *  es pequeño y KI=0.4 lo elimina en ~10-15 ciclos.
 *
 *  ↑ Subir si queda un offset residual después de ~50 ciclos.
 *  ↓ Bajar si el integrador empieza a oscilar lentamente (período >2 s).
 */
#define SC_KI             0.05f

/*
 *  SC_FF_SLOPE_LEFT / SC_FF_SLOPE_RIGHT
 *  Pendiente feed-forward por canal [PWM% / (tick/ciclo)]
 *
 *  Valor calibrado empíricamente: con 100 mm/s el objetivo es ~5.9
 *  ticks/ciclo y el PWM de régimen estacionario era ~46-47%.
 *  slope = 46.5 / 5.9 ≈ 7.88 ... pero el análisis en el chat paralelo
 *  reveló que el mínimo PWM funcional es ~38% y el rango útil es
 *  [38, 70], por lo que la pendiente efectiva sobre ese rango es 3.88.
 *
 *  Si un canal deriva consistentemente más ticks que el otro, ajustar
 *  su slope en pasos de 0.1 hasta equilibrar.
 */
#define SC_FF_SLOPE_LEFT  7.5f
#define SC_FF_SLOPE_RIGHT 7.5f

/*
 *  SC_INTEGRAL_DEADBAND — Zona muerta del integrador [ticks]
 *
 *  El integrador solo acumula cuando |error| > DEADBAND.
 *  Bajado de 1.5 a 0.5: con FF bien centrado la varianza real entre
 *  canales es ~±1 tick, que sí hay que corregir. La deadband de 1.5
 *  era demasiado conservadora y dejaba pasar el sesgo sin corregir.
 */
#define SC_INTEGRAL_DEADBAND 1.0f

/*
 *  SC_INTEGRAL_LIMIT — Anti-windup [ticks acumulados]
 *
 *  Limita el integrador para que no se sature si hay un obstáculo
 *  o una asimetría mecánica extrema. 15 ticks × KI=0.4 = 6 PWM%
 *  de corrección máxima por integrador, razonable para estos motores.
 */
#define SC_INTEGRAL_LIMIT    15.0f

/*
 *  SC_MAX_CORRECTION — Corrección total máxima [PWM%]
 *  Suma de P + I. Previene saturación del canal.
 */
#define SC_MAX_CORRECTION 10.0f

/*
 *  SC_DT — Período de muestreo [s]
 *  Debe coincidir con IMU_SAMPLE_US / 1e6 = 0.01 s
 */
#define SC_DT             0.01f

/* ─────────────────────────────────────────────────────────────────────────────
   TIPOS PÚBLICOS
   ───────────────────────────────────────────────────────────────────────────── */

/*
 *  sc_command_t — comando de velocidad para el robot
 *
 *  v_left_mm_s  : velocidad lineal de la oruga izquierda en mm/s
 *  v_right_mm_s : velocidad lineal de la oruga derecha en mm/s
 *
 *  Para movimiento recto: v_left_mm_s == v_right_mm_s == V
 *  Para giro en el lugar: v_left_mm_s == -v_right_mm_s
 *  Para curva: valores distintos con el mismo signo
 *
 *  Velocidades negativas → movimiento hacia atrás.
 *  Cero en ambos → freno activo.
 */
typedef struct {
    float v_left_mm_s;
    float v_right_mm_s;
} sc_command_t;

/*
 *  sc_state_t — estado diagnóstico del controlador (solo lectura)
 *
 *  Útil para logging, tuning de Kp y verificación del criterio de éxito.
 */
typedef struct {
    float   ticks_target_left;   /* ticks/ciclo objetivo canal izquierdo  */
    float   ticks_target_right;  /* ticks/ciclo objetivo canal derecho     */
    int32_t ticks_actual_left;   /* ticks reales del último ciclo          */
    int32_t ticks_actual_right;  /* ticks reales del último ciclo          */
    float   error_left;          /* error = objetivo - real                */
    float   error_right;
    int     pwm_left;            /* PWM% aplicado [-100, 100]              */
    int     pwm_right;
} sc_state_t;

/* ─────────────────────────────────────────────────────────────────────────────
   API PÚBLICA
   ───────────────────────────────────────────────────────────────────────────── */

/*
 *  speed_control_init()
 *
 *  Inicializa el controlador con los parámetros físicos del robot.
 *
 *  Parámetros:
 *    mm_per_tick : distancia en mm por pulso de encoder.
 *                  Mismo valor que se pasa a odometry_init().
 *                  Si aún no está medido, usar 0.5 como estimación inicial.
 *
 *  Debe llamarse después de encoder_init() y motor_driver_init().
 */
void speed_control_init(float mm_per_tick);

/*
 *  speed_control_set()
 *
 *  Establece el comando de velocidad objetivo.
 *  El controlador usará estos valores en los próximos ciclos.
 *
 *  Puede llamarse en cualquier momento — thread-safe porque la escritura
 *  es atómica en Cortex-M0+ para valores de 32 bits alineados, y el
 *  controlador tolera un ciclo de transición.
 *
 *  Para parar: speed_control_set(&(sc_command_t){0, 0})
 */
void speed_control_set(const sc_command_t *cmd);

/*
 *  speed_control_update()
 *
 *  Ejecuta un ciclo del controlador P. Llamar a 100 Hz desde el main,
 *  justo después de que imu_data_ready() devuelva true (mismo ciclo
 *  que odometry_update()).
 *
 *  Internamente:
 *    1. Lee delta de ticks desde la última llamada (atómico).
 *    2. Calcula ticks objetivo a partir del comando actual.
 *    3. Calcula error por canal.
 *    4. Aplica feed-forward + corrección P.
 *    5. Satura y aplica via motors_set().
 */
void speed_control_update(void);

/*
 *  speed_control_get_state()
 *
 *  Copia el estado diagnóstico del último ciclo en *out*.
 *  Usar solo para tuning y logging — no llamar en el camino crítico.
 */
void speed_control_get_state(sc_state_t *out);

/*
 *  speed_control_stop()
 *
 *  Freno inmediato: pone el comando a cero y llama motors_set(0,0)
 *  directamente sin esperar al próximo ciclo.
 */
void speed_control_stop(void);

#endif /* SPEED_CONTROL_H */