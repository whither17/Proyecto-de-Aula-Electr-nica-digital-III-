/**
 * @file navigation.h
 * @author Robinson Correa Morales - Andres Felipe Agudelo Zapata
 * @brief Módulo de navegación punto a punto para robot diferencial autónomo.
 *
 * Implementa una FSM de tres fases (TURNING → DRIVING → DONE) que lleva
 * al robot desde su posición actual hasta un destino arbitrario en el plano XY.
 *
 * La navegación se divide en dos etapas secuenciales:
 *  -# **TURNING** : giro en el sitio mediante control P sobre el error angular.
 *  -# **DRIVING** : avance recto con rampa de aceleración, delegando el control
 *                   de velocidad diferencial al módulo speed_control (Bloque 1).
 *
 * La pose del robot se obtiene en cada ciclo de odometry_get_pose(), lo que
 * hace el módulo robusto a la deriva acumulada: si el robot se desvía durante
 * el avance, la detección de llegada usa la distancia real al destino.
 *
 * @note Debe llamarse nav_init() después de speed_control_init() y
 *       odometry_init(). nav_update() debe invocarse a 100 Hz en el mismo
 *       ciclo que odometry_update() y speed_control_update().
 *
 * @version 0.1
 * @date 2026-06-09
 * @copyright Copyright (c) 2026
 */
 
#ifndef NAVIGATION_H
#define NAVIGATION_H
 
#include "pico/stdlib.h"
#include "odometry.h"
#include <stdint.h>
#include <stdbool.h>
 
//--------------- Parámetros Configurables ---------------------------//
 
#define NAV_RAMP_CYCLES          70     /**< Ciclos de aceleración al entrar en DRIVING.
                                          *   La velocidad sube linealmente de 0 a
                                          *   #NAV_LINEAR_SPEED_MM_S durante este número
                                          *   de ciclos (a 100 Hz → 70 ciclos = 700 ms).*/
 
#define NAV_LINEAR_SPEED_MM_S    99.0f  /**< Velocidad de avance en línea recta [mm/s].
                                          *   Valor validado con el controlador diferencial
                                          *   actual. Debe ser un valor que el Bloque 1
                                          *   ya haya demostrado controlar correctamente. */
 
#define NAV_TURN_SPEED           55     /**< Velocidad angular máxima de giro en el sitio [%PWM].
                                          *   Se aplica como diferencial puro a motors_set():
                                          *   - Giro izquierda: motors_set(+NAV_TURN_SPEED, -NAV_TURN_SPEED)
                                          *   - Giro derecha:   motors_set(-NAV_TURN_SPEED, +NAV_TURN_SPEED). */
 
#define NAV_ANGLE_TOL            0.03f  /**< Tolerancia angular para salir de TURNING [rad].
                                          *   0.05 rad ≈ 2.9°. Con velocidad de giro baja,
                                          *   el robot debería detenerse dentro de esta ventana
                                          *   sin overshoot apreciable. */
 
#define NAV_POS_TOL_DEFAULT      20.0f  /**< Radio de llegada por defecto [mm].
                                          *   El robot declara "llegué" cuando la distancia
                                          *   euclidiana al destino es menor que este valor.
                                          *   Modificable en tiempo de ejecución con
                                          *   nav_set_pos_tolerance(). */
 
#define NAV_KP_TURN              47.0f  /**< Ganancia proporcional del controlador de giro [-].
                                          *   Escala el error angular [rad] a corrección en %PWM.
                                          *   Con error de π rad y KP=45, la corrección sería
                                          *   141 %PWM, saturada al límite #NAV_TURN_SPEED. */
 
#define NAV_TURN_PWM_MIN         47     /**< PWM mínimo aplicado durante el giro [%PWM].
                                          *   Umbral justo por encima del punto de arranque
                                          *   de los motores para vencer la fricción estática.
                                          *   Si el control P devuelve un valor entre 0 y este
                                          *   umbral, se reemplaza por NAV_TURN_PWM_MIN para
                                          *   garantizar que el robot siempre se mueva. */
 
#define NAV_KP_HEADING          25.0f   /**< Ganancia proporcional de corrección de heading en DRIVING
                                          *  Escala el error angular [rad] a mm/s de corrección que se suma
                                          *  ÚNICAMENTE a la rueda rezagada — la adelantada se queda en v base.
                                          *  Esto evita bajar por debajo de la zona muerta del motor.
                                          */
#define NAV_HEADING_CORR_MAX    10.0f   /**< Saturación de la corrección de heading. Limita cuánto puede acelerar 
                                          *  la rueda rezagada durante DRIVING.
                                          *  Sin este límite, errores grandes (por deriva acumulada) disparan
                                          *  la corrección y el robot termina girando en círculo.
                                          */
/**
 * @brief Estados de la máquina de navegación punto a punto.
 *
 * Diagrama de transiciones:
 * @code
 *   IDLE ──nav_go_to()──→ TURNING ──|error|<TOL──→ DRIVING ──dist<tol──→ DONE
 *    ▲                                                                      │
 *    └──────────────────────────── (siguiente ciclo) ──────────────────────┘
 * @endcode
 */
typedef enum {
    NAV_IDLE    = 0, /**< Sin objetivo activo. Motores detenidos.
                       *   Estado inicial y de reposo tras completar
                       *   una navegación o llamar nav_stop(). */
    NAV_TURNING = 1, /**< Girando en el sitio hacia el heading del destino.
                       *   Control P sobre el error angular. Transiciona a
                       *   NAV_DRIVING cuando |angle_error| < #NAV_ANGLE_TOL. */
    NAV_DRIVING = 2, /**< Avanzando en línea recta hacia el destino.
                       *   Rampa de aceleración activa durante #NAV_RAMP_CYCLES
                       *   ciclos. Delega el control diferencial a speed_control.
                       *   Transiciona a NAV_DONE cuando dist < pos_tolerance. */
    NAV_DONE    = 3  /**< Destino alcanzado. Motores detenidos.
                       *   Transiciona automáticamente a NAV_IDLE en el siguiente
                       *   ciclo de nav_update() si no se llama nav_go_to(). */
} nav_state_t;
 
/**
 * @brief Snapshot del estado de navegación para diagnóstico y logging.
 *
 * Se actualiza en cada ciclo de nav_update(). Accesible externamente
 * mediante nav_get_status(). No debe usarse en el camino crítico de control.
 */
typedef struct {
    nav_state_t state;              /**< Estado actual de la FSM. */
    float       target_x;           /**< Coordenada X del destino activo [mm]. */
    float       target_y;           /**< Coordenada Y del destino activo [mm]. */
    float       distance_to_goal;   /**< Distancia euclidiana al destino en el
                                      *   ciclo actual [mm]. */
    float       angle_error;        /**< Error angular normalizado al destino
                                      *   en el ciclo actual [rad]. Rango (-π, π]. */
    float       pos_tolerance;      /**< Radio de llegada activo en el ciclo
                                      *   actual [mm]. */
} nav_status_t;
 
//--------------- Funciones Públicas --------------------//

/**
 * @brief Inicializa el módulo de navegación.
 *
 * Pone la FSM en NAV_IDLE, carga la tolerancia de posición por defecto
 * (#NAV_POS_TOL_DEFAULT) y pone a cero el snapshot de diagnóstico.
 */
void nav_init();
 
/**
 * @brief Establece un nuevo destino y arranca la navegación.
 *
 * Si hay un movimiento en curso lo cancela inmediatamente (llama
 * speed_control_stop() y motors_set(0,0)) antes de iniciar el nuevo.
 *
 * La FSM siempre arranca desde NAV_TURNING: si el robot ya apunta al
 * destino, la primera llamada a nav_update() transicionará a DRIVING
 * en el mismo ciclo.
 *
 * @param x_mm  Coordenada X del destino [mm], referida al origen de odometría.
 * @param y_mm  Coordenada Y del destino [mm], referida al origen de odometría.
 */
void nav_go_to(float x_mm, float y_mm);
 
/**
 * @brief Ejecuta un ciclo de la FSM de navegación.
 *
 * Debe invocarse a 100 Hz, en el mismo ciclo que odometry_update() y
 * speed_control_update(). Cada llamada:
 *  - En **TURNING** : calcula error angular, aplica control P con PWM mínimo,
 *                     verifica convergencia.
 *  - En **DRIVING** : gestiona rampa de aceleración, verifica distancia al destino.
 *  - En **DONE**    : transiciona a NAV_IDLE para liberar la FSM.
 *  - En **IDLE**    : no hace nada.
 *
 * @return Estado de la FSM al finalizar el ciclo.
 *         Útil para que el main detecte NAV_DONE sin llamar nav_get_status().
 */
nav_state_t nav_update();
 
/**
 * @brief Consulta si la navegación ha terminado.
 *
 * @return  true si la FSM está en NAV_DONE o NAV_IDLE.
 *          false si está en NAV_TURNING o NAV_DRIVING.
 */
bool nav_is_done();
 
/**
 * @brief Cancela el movimiento actual y detiene los motores.
 *
 * Llama speed_control_stop() y motors_set(0,0) internamente.
 * Deja la FSM en NAV_IDLE.
 */
void nav_stop();
 
/**
 * @brief Cambia el radio de llegada en tiempo de ejecución.
 *
 * Toma efecto en el próximo ciclo de nav_update(). Permite ajustar
 * la tolerancia por segmento de trayectoria según la precisión requerida:
 *  - Waypoints intermedios: valor más alto (ej. 40 mm).
 *  - Aproximación final a un objeto: valor más bajo (ej. 20 mm).
 *
 * @param tol_mm  Nuevo radio de llegada [mm]. Debe ser > 0; valores
 *                menores o iguales a cero se ignoran.
 */
void nav_set_pos_tolerance(float tol_mm);
 
/**
 * @brief Copia el snapshot de diagnóstico del ciclo actual.
 *
 * @param out  Puntero al buffer donde se escribe el estado. No debe ser NULL.
 * @note Solo para logging y tuning. No llamar en el camino crítico de control.
 */
void nav_get_status(nav_status_t *out);
 
#endif