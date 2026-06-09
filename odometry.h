/**
 * =============================================================================
 *  odometry.h
 *  Robot autónomo de mapeo — Módulo 3: Odometría con fusión de sensores
 * =============================================================================
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
 * =============================================================================
 */

#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "hardware/sync.h"
#include "pico/stdlib.h"
#include <stdint.h>
#include <stdbool.h>

/* ─────────────────────────────────────────────────────────────────────────────
   CONSTANTES CONFIGURABLES
   ───────────────────────────────────────────────────────────────────────────── */

/*
 *  Número de muestras para estimar el bias del giroscopio durante
 *  la calibración. A 100 Hz, 200 muestras = 2 segundos quieto.
 *  El robot debe estar completamente inmóvil durante este tiempo.
 */
#define ODO_CALIB_SAMPLES   200

/*
 *  Peso del giroscopio en el filtro complementario (α).
 *  Rango: [0.0, 1.0]
 *    α → 1.0  : confía casi totalmente en el giroscopio
 *    α → 0.0  : confía casi totalmente en los encoders
 *  Valor inicial recomendado: 0.98
 *  Ajustar empíricamente según cuánto deslizamiento tengan las orugas.
 */
#define ODO_ALPHA           0.5f

/*
 *  Período de muestreo en segundos — debe coincidir con IMU_SAMPLE_US.
 *  10 ms = 0.01 s = 100 Hz
 */
#define ODO_DT              0.01f

/* ─────────────────────────────────────────────────────────────────────────────
   TIPOS PÚBLICOS
   ───────────────────────────────────────────────────────────────────────────── */

/*
 *  pose_t — estado completo del robot en el plano
 *
 *  x, y  : posición en mm desde el origen (esquina inferior izquierda
 *           de la región de trabajo)
 *  theta : orientación en radianes, rango (-π, π]
 *           θ = 0      → apunta en dirección +X (derecha)
 *           θ = π/2    → apunta en dirección +Y (arriba)
 *           θ = π/-π   → apunta en dirección -X (izquierda)
 *           θ = -π/2   → apunta en dirección -Y (abajo)
 */
typedef struct {
    float x;
    float y;
    float theta;
} pose_t;

/*
 *  odo_params_t — parámetros físicos del robot
 *
 *  Estos valores dependen del hardware y deben medirse/calibrarse
 *  antes de usar el módulo. Ver comentarios de cada campo.
 */
typedef struct {

    /*
     *  mm_per_tick: distancia en mm que recorre la oruga por cada pulso
     *  del encoder.
     *
     *  Cálculo geométrico (punto de partida):
     *    mm_per_tick = (π * D_efectivo) / N
     *    D_efectivo  = diámetro de la rueda motriz con oruga puesta (mm)
     *    N           = pulsos por vuelta del encoder
     *
     *  Recomendación: refinar este valor con calibración empírica
     *  (marcar 500 mm en el suelo, medir distancia real recorrida,
     *  despejar mm_per_tick = distancia_real / ticks_contados).
     */
    float mm_per_tick;

    /*
     *  baseline: distancia en mm entre los puntos de contacto centrales
     *  de la oruga izquierda y la derecha.
     *
     *  Usado en: Δθ = (d_R - d_L) / baseline
     *
     *  También se puede refinar empíricamente: mandar al robot a girar
     *  360° en el sitio, medir el ángulo real girado y ajustar baseline
     *  hasta que Δθ acumulado = 2π.
     */
    float baseline;

} odo_params_t;

/* ─────────────────────────────────────────────────────────────────────────────
   API PÚBLICA
   ───────────────────────────────────────────────────────────────────────────── */

/*
 *  odometry_init()
 *
 *  Inicializa el módulo con los parámetros físicos del robot y realiza
 *  la calibración del bias del giroscopio.
 *
 *  IMPORTANTE: el robot debe estar quieto y nivelado durante toda la
 *  calibración (ODO_CALIB_SAMPLES * ODO_DT segundos ≈ 2 segundos).
 *
 *  Parámetros:
 *    params      → parámetros físicos del robot (ver odo_params_t)
 *    init_pose   → pose inicial del robot; puede ser {0,0,0} si el robot
 *                  arranca en el origen, o cualquier pose conocida.
 *
 *  Retorna:
 *    true  → inicialización y calibración exitosas
 *    false → la IMU no respondió o no se pudo completar la calibración
 */
bool odometry_init(const odo_params_t *params, const pose_t *init_pose);

/*
 *  odometry_update()
 *
 *  Ejecuta un ciclo completo de estimación de pose. Debe llamarse
 *  cada vez que la IMU tiene datos nuevos, es decir cuando
 *  imu_data_ready() devuelve true.
 *
 *  Internamente:
 *    1. Lee ticks de ambos encoders (delta desde la última llamada)
 *    2. Lee gyro_z de la IMU y resta el bias calibrado
 *    3. Calcula d_L, d_R → d_centro, Δθ_odo
 *    4. Calcula Δθ_imu = gyro_z_corregido * ODO_DT
 *    5. Fusiona: Δθ = α * Δθ_imu + (1-α) * Δθ_odo
 *    6. Integra posición usando ángulo promedio
 *    7. Normaliza θ al rango (-π, π]
 *
 *  Llamar esta función sin que haya datos nuevos en la IMU no produce
 *  error, pero sí introduce un ciclo de doble lectura — idealmente
 *  llamar solo cuando imu_data_ready() == true.
 */
void odometry_update(void);

/*
 *  odometry_get_pose()
 *
 *  Copia la pose estimada actual en la estructura apuntada por *out*.
 *  Lectura atómica — deshabilita interrupciones brevemente para
 *  garantizar consistencia de los tres campos.
 *
 *  Uso típico:
 *    pose_t p;
 *    odometry_get_pose(&p);
 *    // usar p.x, p.y, p.theta
 */
void odometry_get_pose(pose_t *out);

/*
 *  odometry_set_pose()
 *
 *  Sobreescribe la pose estimada con un valor conocido.
 *  Útil cuando el módulo de corrección por landmarks detecta una
 *  referencia y necesita corregir una coordenada específica.
 *
 *  Ejemplo de uso desde el módulo de landmarks:
 *    pose_t p;
 *    odometry_get_pose(&p);
 *    p.x = 0.0f;          // el robot está en el borde izquierdo
 *    odometry_set_pose(&p);
 */
void odometry_set_pose(const pose_t *new_pose);

/*
 *  odometry_get_bias()
 *
 *  Devuelve el bias del giroscopio estimado durante la calibración,
 *  en rad/s. Útil para diagnóstico o logging.
 */
float odometry_get_bias(void);

#endif /* ODOMETRY_H */