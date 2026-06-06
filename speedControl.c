/**
 * =============================================================================
 *  speed_control.c
 *  Robot autónomo de mapeo — Módulo 4: Controlador diferencial de velocidad
 * =============================================================================
 *
 *  Plataforma : Raspberry Pi Pico (RP2040)
 *  Lenguaje   : C puro (sin SO, bare-metal)
 *
 *  ── Estructura del ciclo de control (100 Hz) ─────────────────────────────────
 *
 *    [encoder] ──→ delta_ticks_L, delta_ticks_R
 *                         │
 *                         ▼
 *                   error_L = target_L - delta_L
 *                   error_R = target_R - delta_R
 *                         │
 *                         ▼
 *              si |error| > DEADBAND → integral += error  (anti-ruido)
 *              integral = clamp(integral, ±LIMIT)          (anti-windup)
 *                         │
 *                         ▼
 *              ff_L = FF_SLOPE_LEFT  * |target_L|   (feed-forward por canal)
 *              ff_R = FF_SLOPE_RIGHT * |target_R|
 *              correction = clamp(Kp*error + Ki*integral, ±MAX_CORRECTION)
 *              pwm = sign(target) * ff + correction
 *                         │
 *                         ▼
 *              saturar [-100, 100], aplicar → motors_set(pwm_L, pwm_R)
 *
 *  ── Por qué FF por canal ────────────────────────────────────────────────────
 *
 *  Los dos motores N20 tienen características ligeramente distintas: umbral
 *  de arranque diferente, rozamiento interno diferente, etc. Un FF simétrico
 *  deja un sesgo DC que el integrador tiene que compensar en cada corrida.
 *  Con FF_SLOPE_LEFT / FF_SLOPE_RIGHT calibrados individualmente, el punto
 *  de operación de ambos canales coincide y el PI solo trabaja con varianza
 *  aleatoria corrida a corrida — mucho más pequeña y más fácil de corregir.
 *
 *  ── Por qué deadband en el integrador ───────────────────────────────────────
 *
 *  El objetivo es 5.9 ticks/ciclo (no entero). Los encoders solo pueden dar
 *  enteros, así que siempre hay un error residual de ±0.1 a ±0.9 ticks por
 *  cuantización pura — no representa ninguna diferencia real entre motores.
 *  La deadband de 0.5 ticks filtra ese ruido de cuantización mientras deja
 *  pasar errores reales (≥0.5 tick) para que el integrador los corrija.
 *
 * =============================================================================
 */

#include "speedControl.h"
#include "encoder.h"
#include "Driver_TB6612FNG.h"
#include <math.h>
#include <stdlib.h>

/* ─────────────────────────────────────────────────────────────────────────────
   ESTADO INTERNO
   ───────────────────────────────────────────────────────────────────────────── */

   /* ── Constante de ventana — agregar al estado interno ── */
#define SC_WINDOW_CYCLES  3   /* acumular 5 ciclos antes de corregir */

/* ── Estado interno — agregar estas dos variables ── */
static int32_t g_accum_left   = 0;
static int32_t g_accum_right  = 0;
static int     g_window_count = 0;

/* Parámetro físico recibido en init */
static float g_mm_per_tick = 1.0f;

/* Comando de velocidad actual [mm/s] */
static volatile float g_target_left_mm_s  = 0.0f;
static volatile float g_target_right_mm_s = 0.0f;

/* Ticks del ciclo anterior — para calcular delta */
static int32_t g_prev_left  = 0;
static int32_t g_prev_right = 0;

/* Integradores por canal */
static float g_integral_left  = 0.0f;
static float g_integral_right = 0.0f;

/* Estado diagnóstico del último ciclo */
static sc_state_t g_state = {0};

/* ─────────────────────────────────────────────────────────────────────────────
   HELPERS INTERNOS
   ───────────────────────────────────────────────────────────────────────────── */

static inline float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static inline float mm_s_to_ticks_per_cycle(float v_mm_s) {
    return (v_mm_s * SC_DT) / g_mm_per_tick;
}

/**
 * compute_pwm_for_channel()
 *
 * PI con feed-forward por canal.
 *
 * Parámetros:
 *   target_ticks : objetivo en ticks/ciclo (con signo)
 *   actual_ticks : ticks reales del ciclo actual
 *   ff_slope     : pendiente feed-forward específica de este canal
 *   integral     : puntero al acumulador del integrador (actualizado aquí)
 *   out_error    : puntero donde se escribe el error del ciclo (diagnóstico)
 *
 * Retorna PWM% con signo [-100, 100].
 */
static int compute_pwm_for_channel(float   target_ticks,    /* para error e integral */
                                   float   target_per_cycle, /* solo para el FF */
                                   int32_t actual_ticks,
                                   float   ff_slope,
                                   float  *integral,
                                   float  *out_error) {

    float error = target_ticks - (float)actual_ticks;
    *out_error = error;

    if (fabsf(error) > SC_INTEGRAL_DEADBAND * SC_WINDOW_CYCLES) {
        *integral += error;
    }
    *integral = clampf(*integral, -SC_INTEGRAL_LIMIT, SC_INTEGRAL_LIMIT);

    float ff = ff_slope * fabsf(target_per_cycle);  /* ← ticks/ciclo, no acumulados */

    float correction = clampf(
        SC_KP * error + SC_KI * (*integral),
        -SC_MAX_CORRECTION,
         SC_MAX_CORRECTION
    );

    float sign  = (target_ticks >= 0.0f) ? 1.0f : -1.0f;
    float pwm_f = sign * ff + correction;

    return (int)clampf(pwm_f, -100.0f, 100.0f);
}

/* ─────────────────────────────────────────────────────────────────────────────
   API PÚBLICA
   ───────────────────────────────────────────────────────────────────────────── */

void speed_control_init(float mm_per_tick) {
    g_mm_per_tick = (mm_per_tick > 0.0f) ? mm_per_tick : 1.0f;

    g_prev_left  = encoder_get_left();
    g_prev_right = encoder_get_right();

    g_target_left_mm_s  = 0.0f;
    g_target_right_mm_s = 0.0f;

    g_integral_left  = 0.0f;
    g_integral_right = 0.0f;
    g_accum_left     = 0;
    g_accum_right    = 0;
    g_window_count   = 0;
}

void speed_control_set(const sc_command_t *cmd) {
    g_target_left_mm_s  = cmd->v_left_mm_s;
    g_target_right_mm_s = cmd->v_right_mm_s;
}

void speed_control_update(void) {

    /* ── Caso especial: freno activo ── */
    if (g_target_left_mm_s == 0.0f && g_target_right_mm_s == 0.0f) {
        motors_set(0, 0);

        g_prev_left  = encoder_get_left();
        g_prev_right = encoder_get_right();

        /* Resetear integradores al frenar para no arrancar el próximo
           movimiento con una corrección acumulada del anterior. */
        g_integral_left  = 0.0f;
        g_integral_right = 0.0f;

        g_state.ticks_target_left  = 0.0f;
        g_state.ticks_target_right = 0.0f;
        g_state.ticks_actual_left  = 0;
        g_state.ticks_actual_right = 0;
        g_state.error_left         = 0.0f;
        g_state.error_right        = 0.0f;
        g_state.pwm_left           = 0;
        g_state.pwm_right          = 0;
        return;
    }

    /* ── PASO 1: Delta de ticks ── */

    /* DESPUÉS */
    int32_t cur_left  = encoder_get_left();
    int32_t cur_right = encoder_get_right();

    g_accum_left  += cur_left  - g_prev_left;
    g_accum_right += cur_right - g_prev_right;

    g_prev_left  = cur_left;
    g_prev_right = cur_right;

    g_window_count++;
    if (g_window_count < SC_WINDOW_CYCLES) return;  /* esperar sin actuar */

    int32_t delta_left  = g_accum_left;
    int32_t delta_right = g_accum_right;
    g_accum_left   = 0;
    g_accum_right  = 0;
    g_window_count = 0;

    /* ... continúa al PASO 2 igual que antes */

    /* ── PASO 2: Convertir mm/s a ticks/ciclo ── */

    /* PASO 2 — ajustar el target a la ventana */
/* En PASO 2, calcular ambos */
    float target_per_cycle_L = mm_s_to_ticks_per_cycle(g_target_left_mm_s);
    float target_per_cycle_R = mm_s_to_ticks_per_cycle(g_target_right_mm_s);
    float target_ticks_L = target_per_cycle_L * SC_WINDOW_CYCLES;
    float target_ticks_R = target_per_cycle_R * SC_WINDOW_CYCLES;

    /* ── PASO 3: PI + FF por canal ── */

    float error_L, error_R;

int pwm_L = compute_pwm_for_channel(
    target_ticks_L, target_per_cycle_L, delta_left,
    SC_FF_SLOPE_LEFT, &g_integral_left, &error_L
);

int pwm_R = compute_pwm_for_channel(
    target_ticks_R, target_per_cycle_R, delta_right,
    SC_FF_SLOPE_RIGHT, &g_integral_right, &error_R
);

    /* ── PASO 4: Aplicar ── */

    motors_set(pwm_L, pwm_R);

    /* ── Diagnóstico ── */

    g_state.ticks_target_left  = target_ticks_L;
    g_state.ticks_target_right = target_ticks_R;
    g_state.ticks_actual_left  = delta_left;
    g_state.ticks_actual_right = delta_right;
    g_state.error_left         = error_L;
    g_state.error_right        = error_R;
    g_state.pwm_left           = pwm_L;
    g_state.pwm_right          = pwm_R;
}

void speed_control_get_state(sc_state_t *out) {
    *out = g_state;
}

void speed_control_stop(void) {
    g_target_left_mm_s  = 0.0f;
    g_target_right_mm_s = 0.0f;
    motors_set(0, 0);
    g_prev_left  = encoder_get_left();
    g_prev_right = encoder_get_right();
    g_integral_left  = 0.0f;
    g_integral_right = 0.0f;
    g_accum_left     = 0;
    g_accum_right    = 0;
    g_window_count   = 0;
}