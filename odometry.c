/**
 * =============================================================================
 *  odometry.c
 *  Robot autónomo de mapeo — Módulo 3: Odometría con fusión de sensores
 * =============================================================================
 *
 *  Plataforma : Raspberry Pi Pico (RP2040)
 *  Lenguaje   : C puro (sin SO, bare-metal)
 *
 *  ── Qué hace este módulo ─────────────────────────────────────────────────────
 *
 *  Mantiene una estimación continua de la pose del robot (x, y, θ) fusionando
 *  dos fuentes de información:
 *
 *    Encoders  → distancia recorrida por cada oruga → posición y ángulo grueso
 *    IMU gyro  → velocidad angular real del chasis   → ángulo preciso en giros
 *
 *  La fusión se hace con un filtro complementario de primer orden sobre Δθ:
 *
 *    Δθ_fusionado = α * Δθ_imu + (1-α) * Δθ_odo
 *
 *  ── Flujo de un ciclo ────────────────────────────────────────────────────────
 *
 *    1. Leer ticks_L, ticks_R  (encoders, atómico)
 *    2. Calcular Δticks por encoder desde el ciclo anterior
 *    3. Convertir a distancias: d_L, d_R  [mm]
 *    4. Cinemática diferencial: d_centro, Δθ_odo
 *    5. Leer gyro_z de la IMU, restar bias calibrado
 *    6. Calcular Δθ_imu = gyro_z_corregido * Δt
 *    7. Filtro complementario → Δθ_fusionado
 *    8. Integrar: x, y con ángulo promedio; θ acumula Δθ_fusionado
 *    9. Normalizar θ ∈ (-π, π]
 *
 *  ── Calibración del bias ─────────────────────────────────────────────────────
 *
 *  Durante odometry_init(), con el robot quieto, se acumulan
 *  ODO_CALIB_SAMPLES lecturas de gyro_z y se promedian. Ese promedio
 *  es el offset estático que se resta en cada ciclo posterior.
 *
 * =============================================================================
 */

#include "odometry.h"
#include "encoder.h"
#include "imu.h"
#include <math.h>

/* ─────────────────────────────────────────────────────────────────────────────
   ESTADO INTERNO
   ───────────────────────────────────────────────────────────────────────────── */

/* Pose actual del robot — modificada en odometry_update(), leída en get_pose() */
static volatile pose_t g_pose = {0.0f, 0.0f, 0.0f};

/* Parámetros físicos guardados en init */
static float g_mm_per_tick = 0.0f;
static float g_baseline    = 0.0f;

/* Ticks del ciclo anterior — para calcular el delta en cada actualización */
static int32_t g_prev_ticks_left  = 0;
static int32_t g_prev_ticks_right = 0;

/* Bias del giroscopio estimado durante calibración [rad/s] */
static float g_gyro_bias_z = 0.0f;

/* ─────────────────────────────────────────────────────────────────────────────
   HELPERS INTERNOS
   ───────────────────────────────────────────────────────────────────────────── */

/**
 * normalize_angle()
 *
 * Lleva cualquier ángulo en radianes al rango (-π, π].
 * Necesario para evitar que θ crezca sin límite al acumular deltas.
 *
 * Funciona correctamente para saltos de hasta ±2π por llamada,
 * que es suficiente dado el período de muestreo de 10 ms.
 */
static float normalize_angle(float angle) {
    while (angle >  (float)M_PI) angle -= 2.0f * (float)M_PI;
    while (angle <= -(float)M_PI) angle += 2.0f * (float)M_PI;
    return angle;
}

/* ─────────────────────────────────────────────────────────────────────────────
   INICIALIZACIÓN Y CALIBRACIÓN
   ───────────────────────────────────────────────────────────────────────────── */

bool odometry_init(const odo_params_t *params, const pose_t *init_pose) {

    /* ── Guardar parámetros físicos ── */
    g_mm_per_tick = params->mm_per_tick;
    g_baseline    = params->baseline;

    /* ── Pose inicial ── */
    g_pose.x     = init_pose->x;
    g_pose.y     = init_pose->y;
    g_pose.theta = init_pose->theta;

    /* ── Leer ticks actuales como punto de partida del delta ── */
    g_prev_ticks_left  = encoder_get_left();
    g_prev_ticks_right = encoder_get_right();

    /* ── Calibración del bias del giroscopio ─────────────────────────────────
     *
     *  El robot debe estar completamente quieto durante esta sección.
     *  Se acumulan ODO_CALIB_SAMPLES lecturas de gyro_z esperando
     *  a que la IMU señale datos nuevos en cada muestra.
     *
     *  Tiempo total ≈ ODO_CALIB_SAMPLES * ODO_DT = 2.0 segundos.
     *
     *  Si en algún momento imu_read() falla (error I2C), se abandona
     *  la calibración y se devuelve false para que el main lo sepa.
     * ─────────────────────────────────────────────────────────────────────── */

    float accumulator = 0.0f;

    for (int i = 0; i < ODO_CALIB_SAMPLES; i++) {

        /* Esperar al siguiente disparo de la alarma de la IMU */
        while (!imu_data_ready()) {
            tight_loop_contents(); /* yield en bare-metal: no hace nada,
                                      pero indica al compilador que el
                                      bucle es intencional */
        }

        /* Leer datos del chip por I2C */
        if (!imu_read()) {
            return false; /* Error de comunicación — abortar calibración */
        }

        imu_data_t d;
        imu_get_data(&d);

        accumulator += d.gyro_z;
    }

    /* El bias es el promedio de todas las muestras con el robot quieto */
    g_gyro_bias_z = accumulator / (float)ODO_CALIB_SAMPLES;

    return true;
}

/* ─────────────────────────────────────────────────────────────────────────────
   CICLO DE ACTUALIZACIÓN
   ───────────────────────────────────────────────────────────────────────────── */

void odometry_update(void) {

    /* ── PASO 1-2: Leer encoders y calcular delta de ticks ── */

    int32_t ticks_left  = encoder_get_left();
    int32_t ticks_right = encoder_get_right();

    int32_t delta_left  = ticks_left  - g_prev_ticks_left;
    int32_t delta_right = ticks_right - g_prev_ticks_right;

    g_prev_ticks_left  = ticks_left;
    g_prev_ticks_right = ticks_right;

    /* ── PASO 3: Convertir ticks a distancia en mm ── */

    float d_L = (float)delta_left  * g_mm_per_tick;
    float d_R = (float)delta_right * g_mm_per_tick;

    /* ── PASO 4: Cinemática diferencial ── */

    /*
     *  d_centro: distancia lineal recorrida por el centro del robot.
     *  Si d_L == d_R el robot fue recto; si difieren, giró.
     */
    float d_centro = (d_R + d_L) * 0.5f;

    /*
     *  Δθ_odo: cambio de ángulo según odometría.
     *  Derivado de la diferencia de arcos dividida por la separación.
     *  Positivo = giro antihorario (izquierda); negativo = horario (derecha).
     */
    float dtheta_odo = (d_L - d_R) / g_baseline;

    /* ── PASO 5: Leer IMU y restar bias ── */

    imu_data_t imu;
    imu_get_data(&imu);

    /*
     *  Restar el bias estimado durante calibración.
     *  Sin esta corrección, gyro_z en reposo no es cero y el ángulo
     *  derivaría aunque el robot esté quieto.
     */
    float gyro_z_corr = -(imu.gyro_z - g_gyro_bias_z);

    /* ── PASO 6: Integrar giroscopio para obtener Δθ_imu ── */

    /*
     *  Δθ_imu = ω_z * Δt
     *  gyro_z está en rad/s, ODO_DT en segundos → resultado en radianes.
     */
    float dtheta_imu = gyro_z_corr * ODO_DT;

    /* ── PASO 7: Filtro complementario ── */

    /*
     *  Fusiona las dos estimaciones de cambio de ángulo.
     *
     *  ODO_ALPHA pondera el giroscopio (alta frecuencia, preciso en giros).
     *  (1 - ODO_ALPHA) pondera los encoders (baja frecuencia, sin deriva).
     *
     *  La suma complementaria garantiza que los pesos sumen 1.0,
     *  lo que conserva las unidades y evita ganancia o atenuación.
     */
    float dtheta = ODO_ALPHA * dtheta_imu + (1.0f - ODO_ALPHA) * dtheta_odo;

    /* ── PASO 8: Integrar posición ── */

    /*
     *  Se usa el ángulo promedio entre el inicio y el final del paso
     *  para proyectar el desplazamiento lineal. Esto da una aproximación
     *  de segundo orden que es significativamente más precisa que usar
     *  solo el ángulo anterior cuando el robot curva.
     *
     *  θ_medio = θ_anterior + Δθ/2
     *  x += d_centro * cos(θ_medio)
     *  y += d_centro * sin(θ_medio)
     *  θ += Δθ
     */
    float theta_mid = g_pose.theta + dtheta * 0.5f;

    /*
     *  Actualizar pose — escritura atómica con interrupciones deshabilitadas.
     *
     *  En Cortex-M0+ las escrituras float de 32 bits no son atómicas
     *  si una IRQ puede interrumpirlas. Deshabilitamos brevemente para
     *  que odometry_get_pose() nunca lea un estado parcialmente escrito.
     */
    uint32_t saved = save_and_disable_interrupts();

    g_pose.x     += d_centro * cosf(theta_mid);
    g_pose.y     += d_centro * sinf(theta_mid);
    g_pose.theta  = normalize_angle(g_pose.theta + dtheta);

    restore_interrupts(saved);
}

/* ─────────────────────────────────────────────────────────────────────────────
   API PÚBLICA — LECTURA Y ESCRITURA DE POSE
   ───────────────────────────────────────────────────────────────────────────── */

void odometry_get_pose(pose_t *out) {
    /*
     *  Lectura atómica de los tres campos de la pose.
     *  Se deshabilitan interrupciones para garantizar que x, y y theta
     *  corresponden al mismo instante y no a dos ciclos distintos.
     */
    uint32_t saved = save_and_disable_interrupts();
    *out = g_pose;
    restore_interrupts(saved);
}

void odometry_set_pose(const pose_t *new_pose) {
    /*
     *  Sobreescribe la pose — usado por el módulo de corrección
     *  por landmarks cuando detecta una referencia conocida.
     *
     *  Normalizar theta por si el módulo externo pasa un valor
     *  fuera del rango (-π, π].
     */
    uint32_t saved = save_and_disable_interrupts();
    g_pose.x     = new_pose->x;
    g_pose.y     = new_pose->y;
    g_pose.theta = normalize_angle(new_pose->theta);
    restore_interrupts(saved);
}

float odometry_get_bias(void) {
    return g_gyro_bias_z;
}