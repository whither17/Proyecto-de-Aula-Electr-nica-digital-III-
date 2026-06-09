/**
 * =============================================================================
 *  navigation.h
 *  Robot autónomo de mapeo — Módulo 5: Navegación punto a punto
 * =============================================================================
 *
 *  Lleva el robot desde su pose actual hasta una coordenada destino (x, y)
 *  usando la pose estimada por odometry_get_pose() como retroalimentación.
 *
 *  ── Máquina de estados ───────────────────────────────────────────────────────
 *
 *    IDLE ──nav_go_to()──→ TURNING ──ángulo OK──→ DRIVING ──distancia OK──→ IDLE
 *                              ↑                       │
 *                              └──────── nunca ────────┘   (fases estrictas)
 *
 *    TURNING : gira en el sitio con control P sobre error angular.
 *              Termina cuando |error_angular| < NAV_ANGLE_TOL.
 *
 *    DRIVING : avanza en línea recta con speed_control_set().
 *              Termina cuando distancia al destino < NAV_POS_TOL.
 *              El Bloque 1 (speed_control) mantiene ambas orugas iguales.
 *
 *  ── Tolerancias ──────────────────────────────────────────────────────────────
 *
 *    NAV_POS_TOL   : radio de llegada en mm. El robot declara "llegué" cuando
 *                    la distancia euclidiana al destino es menor que este valor.
 *                    Valor por defecto: 40 mm (4 cm), ajustable con
 *                    nav_set_pos_tolerance().
 *
 *    NAV_ANGLE_TOL : tolerancia angular antes de arrancar el avance, en rad.
 *                    Valor por defecto: 0.05 rad (~3°).
 *
 *  ── Dependencias ─────────────────────────────────────────────────────────────
 *
 *    odometry.h       → odometry_get_pose()
 *    speedControl.h   → speed_control_set(), speed_control_update()
 *    Driver_TB6612FNG.h → motors_set() para el giro (diferencial puro)
 *
 *  ── Unidades ─────────────────────────────────────────────────────────────────
 *
 *    Posición → mm
 *    Ángulo   → radianes, rango (-π, π]
 *    Velocidad → mm/s
 *
 * =============================================================================
 */

#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "pico/stdlib.h"
#include "odometry.h"
#include <stdint.h>
#include <stdbool.h>

/* ─────────────────────────────────────────────────────────────────────────────
   PARÁMETROS CONFIGURABLES
   ───────────────────────────────────────────────────────────────────────────── */

/*
 *  NAV_RAMP_CYCLES — ciclos de aceleración al entrar en DRIVING
 *
 *  La velocidad sube linealmente de 0 a NAV_LINEAR_SPEED_MM_S
 *  durante este número de ciclos (a 100 Hz → 50 ciclos = 500 ms).
 *  Mismo valor que RAMP_CYCLES en main_example.c para consistencia.
 *
 *  ↑ Subir si los motores patinán al arrancar.
 *  ↓ Bajar si el robot tarda demasiado en alcanzar velocidad de crucero.
 */
#define NAV_RAMP_CYCLES         70
#define DETECTION_CONFIRM_COUNT  3
/*
 *  NAV_LINEAR_SPEED_MM_S — velocidad de avance en línea recta [mm/s]
 *
 *  Debe ser un valor que el Bloque 1 ya haya demostrado que controla bien.
 *  130 mm/s es el valor validado con el controlador diferencial actual.
 */
#define NAV_LINEAR_SPEED_MM_S   95.0f

/*
 *  NAV_TURN_SPEED — velocidad angular de giro en el sitio [PWM%]
 *
 *  Se aplica directamente a motors_set() como diferencial puro:
 *    motors_set(+NAV_TURN_SPEED, -NAV_TURN_SPEED) → giro izquierda
 *    motors_set(-NAV_TURN_SPEED, +NAV_TURN_SPEED) → giro derecha
 *
 *  Valor bajo para precisión de parada. Si el robot se pasa del ángulo
 *  objetivo consistentemente, bajar este valor.
 *
 *  ↑ Subir si el giro es demasiado lento para el criterio de éxito.
 *  ↓ Bajar si el robot sobrepasa el ángulo y oscila antes de estabilizar.
 */
#define NAV_TURN_SPEED          60

/*
 *  NAV_ANGLE_TOL — tolerancia angular para salir de TURNING [rad]
 *
 *  0.05 rad ≈ 2.9°. Con la baseline del robot y velocidad de giro baja,
 *  el robot debería detenerse dentro de esta ventana sin overshoot.
 *
 *  ↑ Subir (más tolerante) si el robot oscila mucho en la fase de giro.
 *  ↓ Bajar (más estricto) si el error de llegada viene del heading inicial.
 */
#define NAV_ANGLE_TOL           0.05f

/*
 *  NAV_POS_TOL_DEFAULT — radio de llegada por defecto [mm]
 *
 *  El robot declara "llegué" cuando la distancia euclidiana al destino
 *  es menor que este valor. 40 mm = 4 cm, dentro del criterio de 5 cm.
 *
 *  Se puede cambiar en tiempo de ejecución con nav_set_pos_tolerance().
 */
#define NAV_POS_TOL_DEFAULT     20.0f

/*
 *  NAV_KP_TURN — ganancia proporcional del controlador de giro
 *
 *  Escala el error angular [rad] a PWM% de corrección adicional.
 *  Con error de π rad (~180°) y KP=20, la corrección sería 62 PWM%,
 *  que sumada al NAV_TURN_SPEED base da la velocidad máxima.
 *
 *  ↑ Subir si el robot tarda demasiado en centrarse en el ángulo.
 *  ↓ Bajar si oscila alrededor del ángulo objetivo durante el giro.
 */
#define NAV_KP_TURN             45.0f
#define NAV_TURN_PWM_MIN  47    // justo encima de tu umbral de arranque
/* ─────────────────────────────────────────────────────────────────────────────
   TIPOS PÚBLICOS
   ───────────────────────────────────────────────────────────────────────────── */

/*
 *  nav_state_t — estado de la máquina de navegación
 *
 *  NAV_IDLE    : sin objetivo activo, motores parados
 *  NAV_TURNING : girando en el sitio hacia el heading del destino
 *  NAV_DRIVING : avanzando en línea recta hacia el destino
 *  NAV_DONE    : llegó al destino (transición automática a IDLE en el
 *                próximo ciclo si no se da un nuevo destino)
 */
typedef enum {
    NAV_IDLE    = 0,
    NAV_TURNING = 1,
    NAV_DRIVING = 2,
    NAV_DONE    = 3
} nav_state_t;

/*
 *  nav_status_t — snapshot del estado de navegación para diagnóstico
 */
typedef struct {
    nav_state_t state;          /* estado actual de la FSM              */
    float       target_x;       /* coordenada destino X [mm]            */
    float       target_y;       /* coordenada destino Y [mm]            */
    float       distance_to_goal; /* distancia euclidiana restante [mm] */
    float       angle_error;    /* error angular actual [rad]           */
    float       pos_tolerance;  /* radio de llegada activo [mm]         */
} nav_status_t;

/* ─────────────────────────────────────────────────────────────────────────────
   API PÚBLICA
   ───────────────────────────────────────────────────────────────────────────── */

/*
 *  nav_init()
 *
 *  Inicializa el módulo de navegación. Debe llamarse después de
 *  speed_control_init() y odometry_init().
 *
 *  Deja la FSM en NAV_IDLE y carga la tolerancia de posición por defecto.
 */
void nav_init(void);

/*
 *  nav_go_to()
 *
 *  Establece un nuevo destino y arranca la FSM desde TURNING.
 *  Si ya hay un movimiento en curso, lo cancela y empieza el nuevo.
 *
 *  Parámetros:
 *    x_mm, y_mm : coordenadas del destino en mm desde el origen
 *
 *  La pose actual se lee de odometry_get_pose() en el momento de la llamada
 *  para calcular el heading inicial.
 */
void nav_go_to(float x_mm, float y_mm);

/*
 *  nav_update()
 *
 *  Ejecuta un ciclo de la FSM. Debe llamarse a 100 Hz, en el mismo
 *  ciclo que odometry_update() y speed_control_update().
 *
 *  Internamente:
 *    - En TURNING : calcula error angular, aplica P, espera convergencia
 *    - En DRIVING : llama speed_control_update(), verifica distancia
 *    - En DONE/IDLE: no hace nada
 *
 *  Retorna el estado actual de la FSM — útil para que el main sepa
 *  cuándo el robot llegó sin tener que llamar nav_get_status().
 */
nav_state_t nav_update(void);

/*
 *  nav_is_done()
 *
 *  Retorna true si la FSM está en NAV_DONE o NAV_IDLE.
 *  Forma más cómoda de preguntar "¿llegó?" desde el main.
 */
bool nav_is_done(void);

/*
 *  nav_stop()
 *
 *  Cancela el movimiento actual y pone la FSM en NAV_IDLE.
 *  Llama speed_control_stop() internamente.
 */
void nav_stop(void);

/*
 *  nav_set_pos_tolerance()
 *
 *  Cambia el radio de llegada en mm.
 *  Útil para ajustar la tolerancia por segmento:
 *    - Esquinas del área de trabajo: 40 mm (por defecto)
 *    - Aproximación a una caja (Bloque 7): 20 mm
 *
 *  Toma efecto en el próximo ciclo de nav_update().
 */
void nav_set_pos_tolerance(float tol_mm);

/*
 *  nav_get_status()
 *
 *  Copia el estado de diagnóstico actual en *out*.
 *  Solo para logging y tuning — no llamar en el camino crítico.
 */
void nav_get_status(nav_status_t *out);

#endif /* NAVIGATION_H */