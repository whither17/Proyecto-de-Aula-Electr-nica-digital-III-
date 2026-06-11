/**
 * @file encoder.h
 * @author Andrés Felipe Agudelo Zapata (andresf.agudeloz@udea.edu.co)
 * @brief  Módulo de encoders Con Cuadratura, para encoders magnéticos de efecto Hall
 * Con canales A y B por rueda.
 * 
 * Rueda izquierda
 *    Canal A → GP0   (dispara la IRQ en flanco de subida y bajada)
 *    Canal B → GP1   (se lee dentro del handler para detectar dirección)
 *
 * Rueda derecha
 *    Canal A → GP17  (dispara la IRQ en flanco de subida y bajada)
 *    Canal B → GP16  (se lee dentro del handler para detectar dirección)
 * 
 * En un encoder de cuadratura los canales A y B están desfasados 90°.
 *  Cuando el motor gira hacia adelante, A sube ANTES que B:
 *
 *    Adelante:   A sube → B está en ALTO   → tick positivo
 *    Atrás:      A sube → B está en BAJO   → tick negativo
 *
 *  Disparando en ambos flancos (subida Y bajada) de A se duplica la
 *  resolución del encoder sin necesidad de leer el canal B dos veces.
 *
 *    Flanco de SUBIDA  de A + B en ALTO  → avanza  → ticks++
 *    Flanco de SUBIDA  de A + B en BAJO  → retrocede → ticks--
 *    Flanco de BAJADA  de A + B en BAJO  → avanza  → ticks++
 *    Flanco de BAJADA  de A + B en ALTO  → retrocede → ticks--
 * 
 * Advertencia: 
 * Este módulo solo cuenta ticks. La conversión a distancia y ángulo
 * se hace en la capa superior de odometría con
 *
 *    distancia = ticks * (PI * diametro_rueda / pulsos_por_vuelta)

 * @version 0.1
 * @date 2026-06-09
 * 
 * @copyright Copyright (c) 2026
 * 
 */
#ifndef ENCODER_H
#define ENCODER_H

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include <stdint.h>
#include <stdlib.h>

// Definición de pines

#define ENCODER_LEFT_A 0     /**< GP0 canal A rueda izquierda (dispara IRQ) */
#define ENCODER_LEFT_B 1     /**< GP1 canal B rueda izquierda (lee dirección) */
#define ENCODER_RIGHT_A 17   /**< GP17 canal A rueda derecha (dispara IRQ) */
#define ENCODER_RIGHT_B 16   /**< GP16 canal B rueda derecha (lee dirección) */

/**
 * @brief Inicializa los pines GPIO y habilita la interrupción
 * más no la configura
 * 
 */
void encoder_init();

/**
 * @brief Manejador de interrupción GPIO para los encoders
 * Expuesta a otros módulos.
 * 
 * Esta función decide si el evento corresponde a
 * un encoder, en qué dirección giró y actualiza el contador.
 *
 * Lógica de cuadratura:
 *   Flanco RISE en A + B en ALTO  → adelante  → ++
 *   Flanco RISE en A + B en BAJO  → atrás     → --
 *   Flanco FALL en A + B en BAJO  → adelante  → ++
 *   Flanco FALL en A + B en ALTO  → atrás     → --
 *
 * En resumen: tick positivo cuando A y B están en el mismo estado,
 * tick negativo cuando están en estados opuestos.
 * 
 * @param gpio PIN GPIO que generó la interrupción
 * @param events Máscara de eventos GPIO_IRQ_* asociados al disparo.
 */
void encoder_handle_irq(uint gpio, uint32_t events);

/* Lectura atómica de ticks — para uso de la capa de odometría */

/**
 * @brief Obtiene los ticks de la izquierda
 * 
 * @return int32_t Número de Ticks
 */
int32_t encoder_get_left();

/**
 * @brief Obtiene los ticks de la derecha
 * 
 * @return int32_t Número de Ticks
 */
int32_t encoder_get_right();

/**
 * @brief Reestablece ambos contadores a cero
 * 
 */
void encoder_reset();

#endif