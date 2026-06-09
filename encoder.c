/**
 * =============================================================================
 *  encoder.c
 *  Robot autónomo de mapeo — Módulo 2: Encoders de cuadratura
 * =============================================================================
 *
 *  Plataforma : Raspberry Pi Pico (RP2040)
 *  Lenguaje   : C puro (sin SO, bare-metal)
 *  Encoders   : Magnéticos de cuadratura, 2 canales (A y B) por rueda
 *
 *  ── Pinout ───────────────────────────────────────────────────────────────────
 *
 *  Rueda izquierda
 *    Canal A → GP0   (dispara la IRQ en flanco de subida y bajada)
 *    Canal B → GP1   (se lee dentro del handler para detectar dirección)
 *
 *  Rueda derecha
 *    Canal A → GP17  (dispara la IRQ en flanco de subida y bajada)
 *    Canal B → GP16  (se lee dentro del handler para detectar dirección)
 *
 *  ── Cómo funciona la detección de dirección ──────────────────────────────────
 *
 *  En un encoder de cuadratura los canales A y B están desfasados 90°.
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
 *  ── Arquitectura de interrupciones ──────────────────────────────────────────
 *
 *  El RP2040 tiene un único callback GPIO (IO_IRQ_BANK0).
 *  Este driver NO registra el callback — eso lo hace el main.
 *  Este driver solo:
 *    1. Configura los pines como entradas con pull-up.
 *    2. Habilita la IRQ en los pines A de ambos encoders.
 *    3. Expone encoder_handle_irq() para que el main la llame
 *       desde su gpio_irq_handler cuando corresponda.
 *
 *  ── Base para odometría ──────────────────────────────────────────────────────
 *
 *  Este módulo solo cuenta ticks. La conversión a distancia y ángulo
 *  se hace en la capa superior de odometría con:
 *
 *    distancia = ticks * (PI * diametro_rueda / pulsos_por_vuelta)
 *
 *  Los parámetros físicos (diámetro, pulsos por vuelta) no viven aquí
 *  sino en el módulo de odometría, ya que son propiedades del robot,
 *  no del sensor.
 *
 * =============================================================================
 */

#include "encoder.h"

/* ─────────────────────────────────────────────────────────────────────────────
   ESTADO INTERNO
   ───────────────────────────────────────────────────────────────────────────── */

/* Contadores de pulsos — volatile porque se modifican en la ISR */
static volatile int32_t ticks_left  = 0;
static volatile int32_t ticks_right = 0;

/* ─────────────────────────────────────────────────────────────────────────────
   INICIALIZACIÓN
   ───────────────────────────────────────────────────────────────────────────── */

void encoder_init(void) {

    /* Configurar los cuatro pines como entradas con pull-up interno.
       Pull-up porque los encoders magnéticos suelen tener salida de
       colector abierto — sin pull-up el pin flota y genera IRQ falsas. */
    uint8_t pins[] = {
        ENCODER_LEFT_A, ENCODER_LEFT_B,
        ENCODER_RIGHT_A, ENCODER_RIGHT_B
    };
    for (int i = 0; i < 4; i++) {
        gpio_init(pins[i]);
        gpio_set_dir(pins[i], GPIO_IN);
        gpio_pull_up(pins[i]);
    }

    /*
     *  Habilitar IRQ en los canales A — flancos de SUBIDA y BAJADA.
     *  Disparar en ambos flancos duplica la resolución:
     *  por cada ciclo completo del encoder se obtienen 4 eventos
     *  en vez de 2, mejorando la precisión de la odometría.
     *
     *  IMPORTANTE: NO se registra el callback aquí.
     *  El main llama gpio_set_irq_enabled_with_callback() una sola vez
     *  con su gpio_irq_handler general. Este driver solo habilita
     *  los pines para que generen la interrupción.
     */
    gpio_set_irq_enabled(ENCODER_LEFT_A,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

    gpio_set_irq_enabled(ENCODER_RIGHT_A,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
}

/* ─────────────────────────────────────────────────────────────────────────────
   HANDLER — llamado desde el gpio_irq_handler del main
   ───────────────────────────────────────────────────────────────────────────── */

/**
 * encoder_handle_irq()
 *
 * El main la llama desde su gpio_irq_handler pasando el pin que disparó
 * y el tipo de evento. Esta función decide si el evento corresponde a
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
 */
void encoder_handle_irq(uint gpio, uint32_t events) {

    if (gpio == ENCODER_LEFT_A) {
        bool a_rise = (events & GPIO_IRQ_EDGE_RISE) != 0;
        bool b_high = gpio_get(ENCODER_LEFT_B);

        /*
         *  Ambos canales contaban negativo al avanzar → lógica invertida.
         *  Corregido: estado opuesto → adelante, mismo estado → atrás.
         */
        if (a_rise != b_high) {
            ticks_left++;
        } else {
            ticks_left--;
        }
    }
    else if (gpio == ENCODER_RIGHT_A) {
        bool a_rise = (events & GPIO_IRQ_EDGE_RISE) != 0;
        bool b_high = gpio_get(ENCODER_RIGHT_B);

        /*
         *  Motor derecho montado en espejo + mismo problema de signo.
         *  Corregido: cambiar == por != respecto al canal izquierdo.
         */
        if (a_rise == b_high) {
            ticks_right++;
        } else {
            ticks_right--;
        }
    }
}

/* ─────────────────────────────────────────────────────────────────────────────
   LECTURA ATÓMICA — para la capa de odometría
   ───────────────────────────────────────────────────────────────────────────── */

/**
 * encoder_get_left()
 * encoder_get_right()
 *
 * Deshabilitan brevemente la IRQ de GPIO para garantizar que el valor
 * de 32 bits se lee completo sin que una interrupción lo modifique
 * a la mitad. En Cortex-M0+ las lecturas de 32 bits no son atómicas
 * si pueden ser interrumpidas entre los dos accesos de 16 bits internos.
 */
int32_t encoder_get_left(void) {
    irq_set_enabled(IO_IRQ_BANK0, false);
    int32_t t = ticks_left;
    irq_set_enabled(IO_IRQ_BANK0, true);
    return t;
}

int32_t encoder_get_right(void) {
    irq_set_enabled(IO_IRQ_BANK0, false);
    int32_t t = ticks_right;
    irq_set_enabled(IO_IRQ_BANK0, true);
    return t;
}

/**
 * encoder_reset()
 * Pone ambos contadores a cero de forma atómica.
 * Útil al inicio de cada segmento de movimiento para la odometría.
 */
void encoder_reset(void) {
    irq_set_enabled(IO_IRQ_BANK0, false);
    ticks_left  = 0;
    ticks_right = 0;
    irq_set_enabled(IO_IRQ_BANK0, true);
}