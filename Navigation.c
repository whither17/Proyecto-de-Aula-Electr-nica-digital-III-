/**
 * @file navigation.c
 * @author Robinson Correa Morales - Andres Felipe Agudelo Zapata
 * @brief Implementación del módulo de navegación punto a punto.
 *
 * Robot autónomo de mapeo — Módulo 5.
 *
 * ── Máquina de estados ───────────────────────────────────────────────────────
 *
 * nav_go_to() siempre arranca desde TURNING, incluso si el robot ya apunta
 * en la dirección correcta — la fase de giro termina en el primer ciclo si
 * el error angular ya está dentro de la tolerancia.
 *
 * @code
 *   IDLE
 *     │  nav_go_to(x, y)
 *     ▼
 *   TURNING ── |angle_error| < NAV_ANGLE_TOL ──→ DRIVING
 *                                                    │
 *                                        dist < pos_tol
 *                                                    │
 *                                                    ▼
 *                                                  DONE ──→ IDLE (siguiente ciclo
 *                                                            si no hay nuevo destino)
 * @endcode
 *
 * Control de giro:
 *
 * Control P puro sobre el error angular con PWM mínimo para vencer fricción:
 * @code
 *   angle_to_goal = atan2(dy, dx)
 *   angle_error   = normalize(angle_to_goal - pose.theta)
 *   turn_pwm      = clamp(NAV_KP_TURN * angle_error, ±NAV_TURN_SPEED)
 *   if |turn_pwm| < NAV_TURN_PWM_MIN → turn_pwm = ±NAV_TURN_PWM_MIN
 * @endcode
 *
 * Control de avance:
 *
 * Delega completamente en speed_control (Bloque 1):
 * @code
 *   sc_command_t cmd = { v, v };   // v sube linealmente durante NAV_RAMP_CYCLES
 *   speed_control_set(&cmd);
 * @endcode
 *
 * Cálculo de distancia y ángulo:
 *
 * Se recalculan en cada ciclo con la pose fresca de odometry_get_pose().
 */

#include "navigation.h"
#include "odometry.h"
#include "speedControl.h"
#include "Driver_TB6612FNG.h"
#include <math.h>
#include <stdio.h>


//------------ Estados Internos ----------------//

static nav_state_t g_state = NAV_IDLE; /**< Estado actual de la FSM de navegación. */

static float g_target_x = 0.0f;        /**< Coordenada X del destino activo [mm]. */
static float g_target_y = 0.0f;        /**< Coordenada Y del destino activo [mm]. */

static float g_pos_tolerance = NAV_POS_TOL_DEFAULT; /**< Radio de llegada activo [mm].
                                                       *   Modificable en tiempo de ejecución
                                                       *   con nav_set_pos_tolerance(). */

static nav_status_t g_status = {0};    /**< Snapshot de diagnóstico del último ciclo.
                                         *   Accesible externamente con nav_get_status(). */

static int g_ramp_cycle = 0;           /**< Contador de ciclos transcurridos en la rampa
                                         *   de aceleración de la fase DRIVING.
                                         *   Rango [0, NAV_RAMP_CYCLES]. Se resetea a 0
                                         *   cada vez que se entra en DRIVING.
                                         *   Con 100 Hz y NAV_RAMP_CYCLES = 70, la rampa
                                         *   dura 700 ms. */

//--------------- Funciones Privadas ------------------//

/**
 * @brief Normaliza un ángulo al intervalo (-π, π].
 *
 * Necesario para que el error angular represente siempre el camino de giro
 * más corto. Sin normalizar, girar de 170° a -170° daría un error de -340°
 * en lugar del correcto de 20°.
 *
 * @param a  Ángulo a normalizar [rad].
 * @return   Ángulo equivalente en (-π, π] [rad].
 */
static float normalize_angle(float a)
{
    while (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
    while (a <= -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}

/**
 * @brief Satura un valor flotante al intervalo [lo, hi].
 *
 * @param v   Valor a saturar.
 * @param lo  Límite inferior.
 * @param hi  Límite superior.
 * @return    @p v saturado al intervalo [@p lo, @p hi].
 */
static inline float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/**
 * @brief Calcula la distancia al destino y el error angular desde la pose actual.
 *
 * Usa g_target_x / g_target_y como destino y la pose provista como origen.
 * El error angular se normaliza al rango (-π, π] para garantizar el giro
 * por el camino más corto.
 *
 * @param[in]  pose            Pose actual del robot (x, y, theta).
 * @param[out] out_dist        Distancia euclidiana al destino [mm].
 * @param[out] out_angle_error Error angular normalizado [rad].
 * @return     @c true  si la geometría es válida (distancia ≥ 1 mm). @n
 *             @c false si el destino coincide con la pose actual (dist < 1 mm),
 *                      en cuyo caso no tiene sentido calcular el ángulo y
 *                      se escribe 0.0 en @p out_angle_error.
 */
static bool compute_nav_geometry(const pose_t *pose,
                                  float       *out_dist,
                                  float       *out_angle_error)
{
    float dx = g_target_x - pose->x;
    float dy = g_target_y - pose->y;

    *out_dist = sqrtf(dx * dx + dy * dy);

    if (*out_dist < 1.0f)
    {
        *out_angle_error = 0.0f;
        return false;
    }

    float angle_to_goal  = atan2f(dy, dx);
    *out_angle_error     = normalize_angle(angle_to_goal - pose->theta);
    return true;
}

/* ─────────────────────────────────────────────────────────────────────────────
   FSM — FASE TURNING
   ───────────────────────────────────────────────────────────────────────────── */
/**
 * @brief Ejecuta un ciclo de la fase TURNING.
 *
 * Aplica control proporcional sobre el error angular con saturación y PWM
 * mínimo para vencer la fricción estática de los motores.
 *
 * Lógica de signos en motors_set():
 *  - angle_error > 0 → destino a la izquierda del heading actual →
 *    motors_set(+turn_pwm, -turn_pwm)
 *  - angle_error < 0 → destino a la derecha del heading actual →
 *    motors_set(+turn_pwm, -turn_pwm)  (turn_pwm ya tiene signo negativo)
 *
 * La asimetría se resuelve naturalmente por el signo de turn_pwm: un valor
 * positivo gira a la izquierda, negativo a la derecha, sin condiciones extra.
 *
 * @return NAV_DONE    si la distancia al destino es < 1 mm (ya llegó).
 * @return NAV_DRIVING si |angle_error| < #NAV_ANGLE_TOL (alineado).
 * @return NAV_TURNING en cualquier otro caso (continúa girando).
 */
static nav_state_t fsm_turning(void)
{
    pose_t pose;
    odometry_get_pose(&pose);

    float dist, angle_error;
    bool valid = compute_nav_geometry(&pose, &dist, &angle_error);

    g_status.distance_to_goal = dist;
    g_status.angle_error      = angle_error;

    /* Destino coincide con posición actual — no hay ángulo que corregir */
    if (!valid)
    {
        motors_set(0, 0);
        return NAV_DONE;
    }

    /* Robot ya apunta al destino — arrancar avance con rampa limpia */
    if (fabsf(angle_error) < NAV_ANGLE_TOL)
    {
        motors_set(0, 0);
        g_ramp_cycle = 0;
        return NAV_DRIVING;
    }

    /*
     * Control P sobre el error angular.
     * La saturación a ±NAV_TURN_SPEED limita la velocidad máxima de giro
     * para mejorar la precisión de parada.
     */
    float turn_pwm_f = clampf(
        NAV_KP_TURN * angle_error,
        -(float)NAV_TURN_SPEED,
         (float)NAV_TURN_SPEED
    );

    /*
     * PWM mínimo para vencer fricción estática.
     * Si el control P produce un valor inferior al umbral de arranque
     * de los motores, se fuerza al mínimo garantizado de movimiento.
     */
    if      (turn_pwm_f > 0.0f && turn_pwm_f <  NAV_TURN_PWM_MIN) turn_pwm_f =  NAV_TURN_PWM_MIN;
    else if (turn_pwm_f < 0.0f && turn_pwm_f > -NAV_TURN_PWM_MIN) turn_pwm_f = -NAV_TURN_PWM_MIN;

    int turn_pwm = (int)turn_pwm_f;
    motors_set(turn_pwm, -turn_pwm);

    return NAV_TURNING;
}

/* ─────────────────────────────────────────────────────────────────────────────
   FSM — FASE DRIVING
   ───────────────────────────────────────────────────────────────────────────── */

/**
 * @brief Ejecuta un ciclo de la fase DRIVING.
 *
 * Gestiona la rampa de aceleración y verifica la distancia al destino.
 * El control diferencial de velocidad se delega completamente al Bloque 1
 * (speed_control): este módulo solo decide cuándo parar.
 *
 * Rampa lineal durante #NAV_RAMP_CYCLES ciclos:
 * @code
 *   v = NAV_LINEAR_SPEED_MM_S * (g_ramp_cycle / NAV_RAMP_CYCLES)
 * @endcode
 * Pasada la rampa, la velocidad se fija en #NAV_LINEAR_SPEED_MM_S.
 *
 * @return NAV_DONE    si dist < g_pos_tolerance (destino alcanzado).
 * @return NAV_DRIVING en cualquier otro caso (continúa avanzando).
 */
static nav_state_t fsm_driving(void)
{
    pose_t pose;
    odometry_get_pose(&pose);

    float dist, angle_error;
    compute_nav_geometry(&pose, &dist, &angle_error);

    g_status.distance_to_goal = dist;
    g_status.angle_error      = angle_error;

    if (dist < g_pos_tolerance)
    {
        speed_control_stop();
        return NAV_DONE;
    }
    static uint32_t dbg = 0;
    if (++dbg % 20 == 0)
    {
        printf(
            "dist=%.1f  err=%.2fdeg  x=%.1f y=%.1f\n",
            dist,
            angle_error * 180.0f / M_PI,
            pose.x,
            pose.y
        );
    }

    /*
     * Rampa de aceleración lineal.
     * Evita el golpe de corriente y el patinaje en el arranque,
     * consistente con el RAMP_CYCLES del main_example.c existente.
     */
    float v;
    if (g_ramp_cycle < NAV_RAMP_CYCLES)
    {
        g_ramp_cycle++;
        v = NAV_LINEAR_SPEED_MM_S * ((float)g_ramp_cycle / (float)NAV_RAMP_CYCLES);
    }
    else
    {
        v = NAV_LINEAR_SPEED_MM_S;
    }

    float correction = NAV_KP_HEADING * (-angle_error);
    if (correction >  NAV_HEADING_CORR_MAX) 
        correction =  NAV_HEADING_CORR_MAX;
    else if (correction < -NAV_HEADING_CORR_MAX) 
        correction = -NAV_HEADING_CORR_MAX;

    sc_command_t cmd = {
        .v_left_mm_s  = v + (correction < 0.0f ? -correction : 0.0f),
        .v_right_mm_s = v + (correction > 0.0f ?  correction : 0.0f)
    };
    speed_control_set(&cmd);

    return NAV_DRIVING;
}

//---------------- Funciones Públicas -------------------//


void nav_init(void)
{
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


void nav_go_to(float x_mm, float y_mm)
{
    speed_control_stop();
    motors_set(0, 0);

    g_target_x = x_mm;
    g_target_y = y_mm;

    g_state      = NAV_TURNING;
    g_ramp_cycle = 0;

    /* Actualizar diagnóstico para que nav_get_status() refleje TURNING
     * inmediatamente, sin necesitar un nav_update() previo. */
    g_status.state         = NAV_TURNING;
    g_status.target_x      = x_mm;
    g_status.target_y      = y_mm;
    g_status.pos_tolerance = g_pos_tolerance;
}

nav_state_t nav_update(void)
{
    switch (g_state)
    {
        case NAV_TURNING:
            g_state = fsm_turning();
            break;

        case NAV_DRIVING:
            g_state = fsm_driving();
            break;

        case NAV_DONE:
            /*
             * Permanecer en DONE UN ciclo para que el main pueda detectarlo.
             * Si nav_go_to() fue llamado mientras estaba en DONE, g_state ya
             * sería NAV_TURNING y este case no se ejecutaría.
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

bool nav_is_done(void)
{
    return (g_state == NAV_DONE || g_state == NAV_IDLE);
}

void nav_stop(void)
{
    speed_control_stop();
    motors_set(0, 0);
    g_state        = NAV_IDLE;
    g_status.state = NAV_IDLE;
}

void nav_set_pos_tolerance(float tol_mm)
{
    if (tol_mm > 0.0f)
    {
        g_pos_tolerance        = tol_mm;
        g_status.pos_tolerance = tol_mm;
    }
}

void nav_get_status(nav_status_t *out)
{
    *out = g_status;
}