/**
 * @file odometry.h
 * @author Andrés Felipe Agudelo Zapata (andresf.agudeloz@udea.edu.co)
 * @brief Módulo de Odometría
 * 
 *  Estima la pose del robot (x, y, θ) combinando encoders de cuadratura
 *  con el giroscopio del MPU6050 mediante un filtro complementario.
 *
 *  Depende de:
 *    encoder.h  — encoder_get_left(), encoder_get_right()
 *    imu.h      — imu_data_ready(), imu_read(), imu_get_data()
 *
 *  Unidades del sistema:
 *    Posición  → milímetros (mm)
 *    Ángulo    → radianes, rango normalizado (-π, π]
 *    Tiempo    → segundos (internamente)
 * 
 * @version 0.1
 * @date 2026-06-09
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "hardware/sync.h"
#include "pico/stdlib.h"
#include <stdint.h>
#include <stdbool.h>


/**
 * @brief Número de muestras para estimar el bias del giroscopio durante
 *  la calibración. A 100 Hz, 200 muestras = 2 segundos quieto.
 *  El robot debe estar completamente inmóvil durante este tiempo.
 * 
 */
#define ODO_CALIB_SAMPLES 200

#define ODO_ALPHA 0.5f /**< Peso del giroscopio en el filtro complementario (α). α → 1.0  : confía casi totalmente en el giroscopio, α → 0.0  : confía casi totalmente en los encoders */
#define ODO_DT 0.01f   /**< Período de muestreo en segundos — debe coincidir con IMU_SAMPLE_US. */


/**
 * @brief Estado del robot en el plano
 * 
 */
typedef struct {
    float x;      /**< Posición en mm desde el origen x */
    float y;      /**< Posición en mm desde el origen y */
    float theta;  /**< Orientación en radianes, /**
 * theta : , rango (-π, π]
 *  θ = 0      → apunta en dirección +X (derecha)
 *  θ = π/2    → apunta en dirección +Y (arriba)
 *  θ = π/-π   → apunta en dirección -X (izquierda)
 *  θ = -π/2   → apunta en dirección -Y (abajo) 
 */ 
} pose_t;
 
/**
 * @brief Parámetros físicos del robot
 */
typedef struct 
{

    float mm_per_tick; /**< Distancia en mm que recorre la oruga por cada pulso del encoder. */
    float baseline;    /**< Distancia en mm entre los puntos de contacto centrales de la oruga izquierda y la derecha. */

} odo_params_t;

//---------------------- Funciones Públicas ----------------------//

/**
 * @brief Inicializa el módulo con los parámetros físicos del robot y realiza
 *  la calibración del bias del giroscopio.
 *
 *  IMPORTANTE: el robot debe estar quieto y nivelado durante toda la
 *  calibración (ODO_CALIB_SAMPLES * ODO_DT segundos ≈ 2 segundos).
 * 
 * @param params Parámetros físicos del robot odo_params_t
 * @param init_pose Pose inicial del robot
 * @return true Inicialización y calibración exitosas
 * @return false La IMU no respondió o no se pudo completar la calibración
 */
bool odometry_init(const odo_params_t *params, const pose_t *init_pose);

/**
 * @brief Ejecuta un ciclo completo de estimación de pose. Debe llamarse
 *  cada vez que la IMU tiene datos nuevos, es decir cuando
 *  imu_data_ready() devuelve true.
 */
void odometry_update();

/**
 * @brief Copia la pose estimada actual en la estructura apuntada por *out.
 *  Lectura atómica — deshabilita interrupciones brevemente para
 *  garantizar consistencia de los tres campos.
 * 
 * @param out Pose de salida (por referencia)
 */
void odometry_get_pose(pose_t *out);

/**
 * @brief Sobreescribe la pose estimada con un valor conocido.
 * 
 * @param new_pose Nueva pose
 */
void odometry_set_pose(const pose_t *new_pose);

/**
 * @brief Devuelve el bias del giroscopio estimado durante la calibración,
 *  en rad/s. Útil para diagnóstico o logging.
 * 
 * @return float Bias estimado
 */
float odometry_get_bias();

#endif