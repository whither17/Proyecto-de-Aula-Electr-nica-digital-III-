#include "encoder.h"


//volatile porque se modifican en la ISR
static volatile int32_t ticks_left  = 0; /**< Contador de ticks del encoder izquierdo */
static volatile int32_t ticks_right = 0; /**< Contador de ticks del encoder derecho */


//------------- Funciones Públicas ---------------------//

void encoder_init() 
{

    /* Configurar los cuatro pines como entradas con pull-up interno.
       Pull-up porque los encoders magnéticos suelen tener salida de
       colector abierto — sin pull-up el pin flota y genera IRQ falsas. 
    */
    uint8_t pins[] = {
        ENCODER_LEFT_A, ENCODER_LEFT_B,
        ENCODER_RIGHT_A, ENCODER_RIGHT_B
    };
    for (int i = 0; i < 4; i++) 
    {
        gpio_init(pins[i]);
        gpio_set_dir(pins[i], GPIO_IN);
        gpio_pull_up(pins[i]);
    }

    /*
     *  Habilitar IRQ en los canales A — flancos de SUBIDA y BAJADA.
     *  Disparar en ambos flancos duplica la resolución:
     *  por cada ciclo completo del encoder se obtienen 4 eventos
     *  en vez de 2, mejorando la precisión de la odometría.
     */
    gpio_set_irq_enabled(ENCODER_LEFT_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(ENCODER_RIGHT_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
}

// Handler
void encoder_handle_irq(uint gpio, uint32_t events) 
{
    if (gpio == ENCODER_LEFT_A) 
    {
        bool a_rise = (events & GPIO_IRQ_EDGE_RISE) != 0;
        bool b_high = gpio_get(ENCODER_LEFT_B);

        if (a_rise != b_high) 
        {
            ticks_left++;
        } 
        else 
        {
            ticks_left--;
        }
    }
    else if (gpio == ENCODER_RIGHT_A) 
    {
        bool a_rise = (events & GPIO_IRQ_EDGE_RISE) != 0;
        bool b_high = gpio_get(ENCODER_RIGHT_B);

        if (a_rise == b_high) 
        {
            ticks_right++;
        } 
        else 
        {
            ticks_right--;
        }
    }
}



/**
 * encoder_get_left()
 * encoder_get_right()
 *
 * Deshabilitan brevemente la IRQ de GPIO para garantizar que el valor
 * de 32 bits se lee completo sin que una interrupción lo modifique
 * a la mitad. Advertencia: En Cortex-M0+ las lecturas de 32 bits no son atómicas
 * si pueden ser interrumpidas entre los dos accesos de 16 bits internos.
 */
int32_t encoder_get_left() 
{
    irq_set_enabled(IO_IRQ_BANK0, false);
    int32_t t = ticks_left;
    irq_set_enabled(IO_IRQ_BANK0, true);
    return t;
}

int32_t encoder_get_right() 
{
    irq_set_enabled(IO_IRQ_BANK0, false);
    int32_t t = ticks_right;
    irq_set_enabled(IO_IRQ_BANK0, true);
    return t;
}

void encoder_reset() 
{
    irq_set_enabled(IO_IRQ_BANK0, false);
    ticks_left  = 0;
    ticks_right = 0;
    irq_set_enabled(IO_IRQ_BANK0, true);
}