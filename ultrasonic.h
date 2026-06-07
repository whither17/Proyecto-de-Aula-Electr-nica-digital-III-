/**
 * @file ultrasonic.h
 * @author Robinson Correa Morales (robinson.corream@udea.edu.co)
 * @brief Diver de sensor ultrasónico HCSR-04. 
 * Flujo de uso:
 *  ultrasonic_init(&front, 17, 16);
 *  *
 *  *
 *  *
 *  ultrasonic_start(&front);
 *  ultrasonic_process(&front);
 *    ...
 *  if(ultrasonic_ready(&front))
        {
            uint32_t d = ultrasonic_get_distance(&front);

            printf("Distancia: %lu cm\n", d);
            ...
            ultrasonic_start(&front);
            printf("READY\n");
        }


 * @version 0.1
 * @date 2026-06-06
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef ULTRASONCI_H
#define ULTRASONIC_H

#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "hardware/timer.h"
#include "hardware/irq.h"
#include <stdbool.h>
#include <stdint.h>

#define NO_CAPTURE_READY 0xffffffff /**<No se obtuvo captura válida.*/
#define CONSTANTE_PRESICION 58      /**<Contante para calcular distancia (d = vel_sonido * T /2).*/

/**
 * @brief Estados de la máquina de estados que controla el funcionamiento del sensor.
 * 
 */
typedef enum 
{
    IDLE,      /**<Iniciar.*/
    WAIT_RISE, /**<Esperar flanco de subida del ECHO.*/
    WAIT_FALL, /**<Esperar flanco de bajada del ECHO.*/
    DONE,      /**<Realizar medición de distancia.*/
} states;

/**
 * @brief Estructura de datos del sensor. Describe el sensor HCSR-04.
 * 
 */
typedef struct
{
    uint trig_pin;               /**<Pin GPIO trigger.*/
    uint echo_pin;               /**<Pin GPIO echo.*/
    uint32_t timer_period_us;    /**<Periodo de duración del TRIGGER.*/
    uint32_t distance;           /**<Distancia calculada.*/
    volatile states estado;      /**<Estado de la FSM del sensor.*/
    volatile bool data_ready;    /**<Bandera que indica si hay medición.*/
    bool flag_rise;              /**<Bandera de flanco de subida del ECHO.*/
    bool flag_fall;              /**<Bandera de flanco de bajada del ECHO.*/
    volatile uint32_t time_rise; /**<Tiempo al momento del flanco de dubida del ECHO (us).*/
    volatile uint32_t time_fall; /**<Tiempo al momento del flanco de bajada del ECHO (us).*/
    uint32_t measured_period;    /**<Periodo medido del ECHO (us).*/
    uint32_t timeout;            /**<Tiempo de espera entre mediciones.*/
} hcsr04_t;

/**
 * @brief Función que inicia si hay medición.
 * 
 * @param dev Apuntador a estructura
 * @return true Hay medición
 * @return false No hay medición.
 */
bool ultrasonic_ready(hcsr04_t *dev);

/**
 * @brief Retorna la distancia calculada
 * 
 * @param dev Apuntador a estructura
 * @return uint32_t Distancia en centímetros.
 */
uint32_t ultrasonic_get_distance(hcsr04_t *dev);

/**
 * @brief Inicializa el sensor
 * 
 * @param dev Apuntador a estructura
 * @param trig_pin GPIO trigger
 * @param echo_pin GPIO ECHO
 */
void ultrasonic_init(hcsr04_t *dev, uint trig_pin, uint echo_pin);

/**
 * @brief Inicializa una medición.
 * 
 * @param dev Apuntador a estructura
 */
void ultrasonic_start(hcsr04_t *dev);

/**
 * @brief Procesa el tiempo de espera y calcula la medición.
 * 
 * @param dev Apuntador a estructura
 */
void ultrasonic_process(hcsr04_t *dev);

#endif
