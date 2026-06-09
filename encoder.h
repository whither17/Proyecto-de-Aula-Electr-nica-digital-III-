#ifndef ENCODER_H
#define ENCODER_H

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include <stdint.h>
#include <stdlib.h>

/* DEFINICIÓN DE PINES */

#define ENCODER_LEFT_A  0    /* GP0  — canal A rueda izquierda (dispara IRQ) */
#define ENCODER_LEFT_B  1    /* GP1  — canal B rueda izquierda (lee dirección) */
#define ENCODER_RIGHT_A 17   /* GP17 — canal A rueda derecha   (dispara IRQ) */
#define ENCODER_RIGHT_B 16   /* GP16 — canal B rueda derecha   (lee dirección) */

void encoder_init(void);

/* Llamar desde el gpio_irq_handler del main cuando el pin sea de un encoder */
void encoder_handle_irq(uint gpio, uint32_t events);

/* Lectura atómica de ticks — para uso de la capa de odometría */
int32_t encoder_get_left(void);
int32_t encoder_get_right(void);

/* Resetea ambos contadores a cero — útil al iniciar una nueva medición */
void encoder_reset(void);

#endif