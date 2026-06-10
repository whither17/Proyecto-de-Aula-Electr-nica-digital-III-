/**
 *  odometry.c
 *  Robot autónomo de mapeo — Módulo 3: Odometría con fusión de sensores
 *  Mantiene una estimación continua de la pose del robot (x, y, θ) fusionando encoders e IMU.
 */

#include "odometry.h"
#include "encoder.h"
#include "imu.h"
#include <math.h>


static volatile pose_t g_pose = {0.0f, 0.0f, 0.0f}; /**< Pose actual del robot */

static float g_mm_per_tick = 0.0f; /**< Distancia en mm que recorre la oruga por cada pulso del encoder */
static float g_baseline = 0.0f;    /**< Distancia en mm entre los puntos de contacto centrales de la oruga izquierda y la derecha */

static int32_t g_prev_ticks_left = 0;  /**< Ticks del ciclo anterior encoder izquierdo */
static int32_t g_prev_ticks_right = 0; /**< Ticks del ciclo anterior encoder derecho */

static float g_gyro_bias_z = 0.0f; /**< Bias del giroscopio estimado durante calibración [rad/s] */



//---------------- Funciones Privadas -------------------//

/**
 * @brief Lleva cualquier ángulo en radianes al rango (-π, π].
 * Necesario para evitar que θ crezca sin límite al acumular deltas.

 * @param angle Ángulo en radianes de la orientación del robot
 * @return float Ángulo en el rango (-π, π]
 */
static float normalize_angle(float angle) 
{
    while (angle > (float)M_PI) 
        angle -= 2.0f * (float)M_PI;
    while (angle <= -(float)M_PI) 
        angle += 2.0f * (float)M_PI;
    return angle;
}

//Calibración

bool odometry_init(const odo_params_t *params, const pose_t *init_pose) 
{
    // Guardar parámetros físicos
    g_mm_per_tick = params->mm_per_tick;
    g_baseline = params->baseline;

    // Pose inicial
    g_pose.x = init_pose->x;
    g_pose.y = init_pose->y;
    g_pose.theta = init_pose->theta;

    // Leer ticks actuales como punto de partida del delta
    g_prev_ticks_left = encoder_get_left();
    g_prev_ticks_right = encoder_get_right();

    /* ── Calibración del bias del giroscopio ──
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

    for (int i = 0; i < ODO_CALIB_SAMPLES; i++) 
    {
        // Esperar al siguiente disparo de la alarma de la IMU
        while (!imu_data_ready()) 
        {
            tight_loop_contents(); 
        }

        // Leer datos del chip por I2C
        if (!imu_read()) 
        {
            return false; // Error de comunicación — abortar calibración
        }

        imu_data_t d;
        imu_get_data(&d);

        accumulator += d.gyro_z;
    }

    // El bias es el promedio de todas las muestras con el robot quieto
    g_gyro_bias_z = accumulator / (float)ODO_CALIB_SAMPLES;

    return true;
}

//Ciclo de actualización

void odometry_update() 
{

    // Leer encoders y calcular delta de ticks

    int32_t ticks_left = encoder_get_left();
    int32_t ticks_right = encoder_get_right();

    int32_t delta_left = ticks_left - g_prev_ticks_left;
    int32_t delta_right = ticks_right - g_prev_ticks_right;

    g_prev_ticks_left = ticks_left;
    g_prev_ticks_right = ticks_right;

    // Convertir ticks a distancia en mm

    float d_L = (float)delta_left * g_mm_per_tick;
    float d_R = (float)delta_right * g_mm_per_tick;

    // Cinemática diferencial

    // Detección de giro en el robot
    float d_centro = (d_R + d_L) * 0.5f;

    // cambio de ángulo según odometría.
    // Derivado de la diferencia de arcos dividida por la separación.
    // Positivo = giro antihorario (izquierda); negativo = horario (derecha).
    float dtheta_odo = (d_L - d_R) / g_baseline;

    // Leer IMU y restar bias

    imu_data_t imu;
    imu_get_data(&imu);

    float gyro_z_corr = -(imu.gyro_z - g_gyro_bias_z);

    // Integrar giroscopio para obtener Δθ_imu 

    
    //  Δθ_imu = ω_z * Δt
    //  gyro_z está en rad/s, ODO_DT en segundos → resultado en radianes.
    float dtheta_imu = gyro_z_corr * ODO_DT;

    // Filtro complementario 
    // Fusiona las dos estimaciones de cambio de ángulo.
    // ODO_ALPHA pondera el giroscopio (alta frecuencia, preciso en giros).
    // (1 - ODO_ALPHA) pondera los encoders (baja frecuencia, sin deriva).
    // La suma complementaria garantiza que los pesos sumen 1.0,
    // lo que conserva las unidades y evita ganancia o atenuación.
    float dtheta = ODO_ALPHA * dtheta_imu + (1.0f - ODO_ALPHA) * dtheta_odo;

    // Integrar posición
    float theta_mid = g_pose.theta + dtheta * 0.5f;
    uint32_t saved = save_and_disable_interrupts();

    g_pose.x += d_centro * cosf(theta_mid);
    g_pose.y += d_centro * sinf(theta_mid);
    g_pose.theta = normalize_angle(g_pose.theta + dtheta);

    restore_interrupts(saved);
}

// Lectura de pose

void odometry_get_pose(pose_t *out) 
{
    uint32_t saved = save_and_disable_interrupts();
    *out = g_pose;
    restore_interrupts(saved);
}

void odometry_set_pose(const pose_t *new_pose) 
{
    uint32_t saved = save_and_disable_interrupts();
    g_pose.x = new_pose->x;
    g_pose.y = new_pose->y;
    g_pose.theta = normalize_angle(new_pose->theta);
    restore_interrupts(saved);
}

float odometry_get_bias() 
{
    return g_gyro_bias_z;
}