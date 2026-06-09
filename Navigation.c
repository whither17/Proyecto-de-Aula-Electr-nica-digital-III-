/**
 * =============================================================================
 *  navigation.c
 *  Robot autónomo de mapeo — Módulo 5: Navegación punto a punto
 * =============================================================================
 *
 *  Plataforma : Raspberry Pi Pico (RP2040)
 *  Lenguaje   : C puro (sin SO, bare-metal)
 *
 *  ── Máquina de estados ───────────────────────────────────────────────────────
 *
 *    nav_go_to() siempre arranca desde TURNING, incluso si el robot ya
 *    apunta en la dirección correcta — la fase de giro termina en el
 *    primer ciclo si el error angular ya está dentro de la tolerancia.
 *
 *    IDLE
 *      │  nav_go_to(x, y)
 *      ▼
 *    TURNING ── |angle_error| < NAV_ANGLE_TOL ──→ DRIVING
 *                                                     │
 *                                         dist < pos_tol
 *                                                     │
 *                                                     ▼
 *                                                   DONE ──→ (IDLE en el
 *                                                              siguiente ciclo
 *                                                              si no hay nuevo
 *                                                              destino)
 *
 *  ── Control de giro ──────────────────────────────────────────────────────────
 *
 *  Control P puro sobre el error angular:
 *
 *    angle_to_goal = atan2(dy, dx)
 *    angle_error   = normalize(angle_to_goal - pose.theta)
 *    turn_pwm      = clamp(NAV_KP_TURN * angle_error, ±NAV_TURN_SPEED)
 *
 *  Si angle_error > 0 → destino a la izquierda → motors_set(-turn_pwm, +turn_pwm)
 *  Si angle_error < 0 → destino a la derecha  → motors_set(+|turn_pwm|, -|turn_pwm|)
 *
 *  El clamp al rango ±NAV_TURN_SPEED limita la velocidad máxima de giro
 *  para mejorar la precisión de parada.
 *
 *  ── Control de avance ────────────────────────────────────────────────────────
 *
 *  Delega completamente en el Bloque 1 (speed_control):
 *
 *    sc_command_t cmd = { NAV_LINEAR_SPEED_MM_S, NAV_LINEAR_SPEED_MM_S };
 *    speed_control_set(&cmd);
 *
 *  En cada ciclo de DRIVING solo verifica la distancia al destino.
 *  El controlador diferencial mantiene el avance recto internamente.
 *
 *  ── Cálculo de distancia y ángulo ────────────────────────────────────────────
 *
 *  Se recalculan en cada ciclo con la pose fresca de odometry_get_pose().
 *  Esto hace el control robusto a la deriva acumulada de la odometría:
 *  si el robot se desplaza lateralmente, el heading al destino cambia
 *  y la detección de llegada usa la distancia real, no la planificada.
 *
 * =============================================================================
 */

#include "navigation.h"
#include "odometry.h"
#include "speedControl.h"
#include "Driver_TB6612FNG.h"
#include <math.h>
#include <stdio.h>
/* ─────────────────────────────────────────────────────────────────────────────
   ESTADO INTERNO
   ───────────────────────────────────────────────────────────────────────────── */

/* Estado de la FSM */
static nav_state_t g_state = NAV_IDLE;

/* Coordenadas del destino actual [mm] */
static float g_target_x = 0.0f;
static float g_target_y = 0.0f;

/* Tolerancia de posición activa [mm] — modificable en tiempo de ejecución */
static float g_pos_tolerance = NAV_POS_TOL_DEFAULT;

/* Snapshot de diagnóstico del último ciclo */
static nav_status_t g_status = {0};

/*
 *  Contador de ciclos de rampa para la fase DRIVING.
 *  Se resetea a 0 cada vez que se entra en DRIVING.
 *  La velocidad sube linealmente durante NAV_RAMP_CYCLES ciclos
 *  antes de alcanzar NAV_LINEAR_SPEED_MM_S.
 *
 *  Con 100 Hz y NAV_RAMP_CYCLES=50 la rampa dura 500 ms —
 *  igual que en el main_example.c existente (RAMP_CYCLES=50).
 */
static int g_ramp_cycle = 0;

/* ─────────────────────────────────────────────────────────────────────────────
   HELPERS INTERNOS
   ───────────────────────────────────────────────────────────────────────────── */

/**
 * normalize_angle()
 *
 * Lleva un ángulo al rango (-π, π].
 * Necesario para que el error angular sea siempre el camino más corto:
 *   sin normalizar, girar de 170° a -170° daría error = -340° en vez de 20°.
 */
static float normalize_angle(float a) {
    while (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
    while (a <= -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}

/**
 * clampf()
 * Satura un valor float entre [lo, hi].
 */
static inline float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/**
 * compute_nav_geometry()
 *
 * A partir de la pose actual y el destino, calcula:
 *   - distancia euclidiana al destino [mm]
 *   - error angular normalizado [rad]
 *
 * Retorna false si el destino es el mismo punto que la pose actual
 * (distancia < 1 mm), en cuyo caso no tiene sentido calcular el ángulo.
 */
static bool compute_nav_geometry(const pose_t *pose,
                                  float *out_dist,
                                  float *out_angle_error) {
    float dx = g_target_x - pose->x;
    float dy = g_target_y - pose->y;

    *out_dist = sqrtf(dx * dx + dy * dy);

    /* Si ya estamos encima del destino, no calcular ángulo */
    if (*out_dist < 1.0f) {
        *out_angle_error = 0.0f;
        return false;
    }

    float angle_to_goal = atan2f(dy, dx);
    *out_angle_error = normalize_angle(angle_to_goal - pose->theta);
    return true;
}

/* ─────────────────────────────────────────────────────────────────────────────
   FSM — FASE TURNING
   ───────────────────────────────────────────────────────────────────────────── */

/**
 * fsm_turning()
 *
 * Aplica control P sobre el error angular.
 * Transición a DRIVING cuando |error| < NAV_ANGLE_TOL.
 *
 * El PWM de giro se calcula así:
 *   turn_pwm = clamp(KP_TURN * angle_error, ±NAV_TURN_SPEED)
 *
 * Nota sobre el signo en motors_set():
 *   angle_error > 0 → destino a la izquierda del heading actual
 *                   → oruga izquierda atrás, derecha adelante
 *                   → motors_set(-turn_pwm, +turn_pwm)
 *   angle_error < 0 → destino a la derecha
 *                   → motors_set(+|turn_pwm|, -|turn_pwm|)
 *
 * La asimetría de signos en el motors_set hace que un turn_pwm positivo
 * siempre gire hacia donde está el destino, sin condiciones adicionales.
 */
static nav_state_t fsm_turning(void) {

    pose_t pose;
    odometry_get_pose(&pose);

    float dist, angle_error;
    bool valid = compute_nav_geometry(&pose, &dist, &angle_error);

    /* Actualizar diagnóstico */
    g_status.distance_to_goal = dist;
    g_status.angle_error      = angle_error;

    /* Si la distancia es cero (mismo punto), no hay ángulo que calcular */
    if (!valid) {
        motors_set(0, 0);
        return NAV_DONE;
    }

    /* ¿Ya apunta al destino? → pasar a DRIVING */
    if (fabsf(angle_error) < NAV_ANGLE_TOL) {
        motors_set(0, 0);
        g_ramp_cycle = 0;   /* resetear rampa para el arranque limpio */
        return NAV_DRIVING;
    }

    /*
     *  Control P sobre el error angular.
     *
     *  KP_TURN * angle_error da la intensidad proporcional al error.
     *  Se satura a ±NAV_TURN_SPEED para no superar la velocidad máxima de giro.
     *
     *  turn_pwm tiene el signo del error angular:
     *    > 0 → hay que girar izquierda
     *    < 0 → hay que girar derecha
     */
    // navigation.c — en fsm_turning(), reemplazar el bloque del clamp:

float turn_pwm_f = clampf(
    NAV_KP_TURN * angle_error,
    -(float)NAV_TURN_SPEED,
     (float)NAV_TURN_SPEED
);

// ── AGREGAR: PWM mínimo para vencer fricción estática ──────────────

if (turn_pwm_f > 0.0f && turn_pwm_f < NAV_TURN_PWM_MIN)
    turn_pwm_f = NAV_TURN_PWM_MIN;
else if (turn_pwm_f < 0.0f && turn_pwm_f > -NAV_TURN_PWM_MIN)
    turn_pwm_f = -NAV_TURN_PWM_MIN;
// ────────────────────────────────────────────────────────────────────

int turn_pwm = (int)turn_pwm_f;
motors_set(turn_pwm, -turn_pwm);

    return NAV_TURNING;
}

/* ─────────────────────────────────────────────────────────────────────────────
   FSM — FASE DRIVING
   ───────────────────────────────────────────────────────────────────────────── */

/**
 * fsm_driving()
 *
 * Avanza en línea recta delegando el control en speed_control_update().
 * Verifica en cada ciclo si la distancia al destino cayó bajo la tolerancia.
 *
 * El Bloque 1 (speed_control) mantiene ambas orugas a la misma velocidad.
 * Este módulo solo necesita verificar cuándo parar.
 */
static nav_state_t fsm_driving(void) {

    pose_t pose;
    odometry_get_pose(&pose);

    float dist, angle_error;
    compute_nav_geometry(&pose, &dist, &angle_error);

    /* Actualizar diagnóstico */
    g_status.distance_to_goal = dist;
    g_status.angle_error      = angle_error;

    /* ¿Llegamos? */
    if (dist < g_pos_tolerance) {
        speed_control_stop();
        return NAV_DONE;
    }

    /*
     *  Rampa de aceleración lineal durante NAV_RAMP_CYCLES ciclos.
     *
     *  En los primeros NAV_RAMP_CYCLES ciclos la velocidad sube de 0
     *  a NAV_LINEAR_SPEED_MM_S de forma proporcional al ciclo actual.
     *  Pasada la rampa, g_ramp_cycle >= NAV_RAMP_CYCLES y la velocidad
     *  se fija en NAV_LINEAR_SPEED_MM_S para siempre.
     *
     *  Esto evita el golpe de corriente y el patinaje en el arranque,
     *  igual que el RAMP_CYCLES del main_example.c existente.
     */
    float v;
    if (g_ramp_cycle < NAV_RAMP_CYCLES) {
        g_ramp_cycle++;
        v = NAV_LINEAR_SPEED_MM_S * ((float)g_ramp_cycle / (float)NAV_RAMP_CYCLES);
    } else {
        v = NAV_LINEAR_SPEED_MM_S;
    }

    sc_command_t cmd = {
        .v_left_mm_s  = v,
        .v_right_mm_s = v
    };
    speed_control_set(&cmd);

    return NAV_DRIVING;
}

/* ─────────────────────────────────────────────────────────────────────────────
   API PÚBLICA
   ───────────────────────────────────────────────────────────────────────────── */

void nav_init(void) {
    g_state         = NAV_IDLE;
    g_target_x      = 0.0f;
    g_target_y      = 0.0f;
    g_pos_tolerance = NAV_POS_TOL_DEFAULT;

    g_status.state            = NAV_IDLE;
    g_status.target_x         = 0.0f;
    g_status.target_y         = 0.0f;
    g_status.distance_to_goal = 0.0f;
    g_status.angle_error      = 0.0f;
    g_status.pos_tolerance    = NAV_POS_TOL_DEFAULT;
}

void nav_go_to(float x_mm, float y_mm) {

    /* Cancelar movimiento anterior si lo hubiera */
    speed_control_stop();
    motors_set(0, 0);

    /* Guardar destino */
    g_target_x = x_mm;
    g_target_y = y_mm;

    /* Siempre arrancar desde TURNING — si ya apunta bien,
       la primera llamada a fsm_turning() transiciona inmediatamente */
    g_state      = NAV_TURNING;
    g_ramp_cycle = 0;

    /* Actualizar diagnóstico — incluyendo state para que nav_get_status()
       refleje TURNING inmediatamente sin necesitar un nav_update() previo */
    g_status.state         = NAV_TURNING;
    g_status.target_x      = x_mm;
    g_status.target_y      = y_mm;
    g_status.pos_tolerance = g_pos_tolerance;
}

nav_state_t nav_update(void) {

    switch (g_state) {

        case NAV_TURNING:
            g_state = fsm_turning();
            break;

        case NAV_DRIVING:
            g_state = fsm_driving();
            break;

        case NAV_DONE:
            /*
             *  Permanecer en DONE UN ciclo para que el main pueda leerlo.
             *  Solo transicionar a IDLE si nav_go_to() NO fue llamado
             *  después de que se puso DONE — en ese caso g_state ya sería
             *  TURNING y este case nunca se ejecutaría.
             */
            g_state = NAV_IDLE;
            break;

        case NAV_IDLE:
        default:
            break;
    }

    g_status.state = g_state;
    return g_state;
}

bool nav_is_done(void) {
    return (g_state == NAV_DONE || g_state == NAV_IDLE);
}

void nav_stop(void) {
    speed_control_stop();
    motors_set(0, 0);
    g_state        = NAV_IDLE;
    g_status.state = NAV_IDLE;
}

void nav_set_pos_tolerance(float tol_mm) {
    if (tol_mm > 0.0f) {
        g_pos_tolerance        = tol_mm;
        g_status.pos_tolerance = tol_mm;
    }
}

void nav_get_status(nav_status_t *out) {
    *out = g_status;
}