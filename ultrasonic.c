
#include "ultrasonic.h"

static hcsr04_t *instances[30] = {0};   /*<Arreglo para múltiples sensores.*/
static bool callback_installed = false; /*<Bandera que indica que la callback ya fue asignada.*/


/**
 * @brief Callback del pulso TRIGGER
 * 
 * @param id alarma
 * @param user_data parámetros del callback (hcsr04_t * en este caso)
 * @return int64_t Retorna siempre cero.
 */
int64_t pulse_callback(alarm_id_t id, void *user_data)
{
    hcsr04_t *dev = (hcsr04_t *)user_data;
    gpio_put(dev->trig_pin, 0);
    return 0;
}

/**
 * @brief Obtiene el periodo del ECHO.
 * 
 * @param dev Apuntador a estructura
 * @return uint32_t Periodo en microsegundos.
 */
static uint32_t ultrasonic_get_period(hcsr04_t *dev)
{
    if(dev->flag_rise && dev->flag_fall)
    {
        dev->flag_rise = false;
        dev->flag_fall = false;
        return dev->time_fall - dev->time_rise;
    } 
    else 
    {
        return NO_CAPTURE_READY;
    }
}

/**
 * @brief Calcula la distancia en cm utilizando el periodo.
 * 
 * @param time Periodo medido de ECHO.
 * @return uint32_t distancia en cm.
 */
static uint32_t ultrasonic_time2cm(uint32_t time)
{
    if (time == NO_CAPTURE_READY)
    {
        return NO_CAPTURE_READY;
    } 
    else 
    {
        uint32_t distance = (time / CONSTANTE_PRESICION);
        return distance;
    }
}

/**
 * @brief Calcula la distancia en cm.
 * 
 * @param dev Apuntador a estructura
 */
static void ultrasonic_calculate_distance(hcsr04_t *dev)
{
    dev->measured_period = ultrasonic_get_period(dev);
    if (dev->measured_period != NO_CAPTURE_READY)
    {
        dev->distance = ultrasonic_time2cm(dev->measured_period);
        return;
    } 
    else 
    {
        dev->distance = NO_CAPTURE_READY;
        return;
    }
    
}

/**
 * @brief Configura la alarma del TRIGGER.
 * 
 * @param dev Apuntador a estructura 
 */
static void ultrasonic_trigger(hcsr04_t *dev)
{
    gpio_put(dev->trig_pin, 1);
    add_alarm_in_us(dev->timer_period_us, pulse_callback, dev, true);
}

/**
 * @brief Callback del ECHO, registra el tiempo en flanco de subida y en flanco de bajada.
 * 
 * @param gpio GPIO del ECHO.
 * @param events Máscara de eventos de GPIO
 */
void ultrasonic_eco_callback(uint gpio, uint32_t events)
{
    hcsr04_t *dev = instances[gpio];
    
    if(dev == NULL)
        return;

    if ((events & GPIO_IRQ_EDGE_RISE) && dev->estado == WAIT_RISE)
    {
        dev->time_rise = time_us_32();
        dev->flag_rise = true;
        dev->estado = WAIT_FALL;
    }

    else if ((events & GPIO_IRQ_EDGE_FALL) && dev->estado == WAIT_FALL)
    {
        dev->time_fall = time_us_32();
        dev->flag_fall = true;
        dev->estado = DONE;
    }
}

bool ultrasonic_ready(hcsr04_t *dev)
{
    return dev->data_ready;
}

uint32_t ultrasonic_get_distance(hcsr04_t *dev)
{
    dev->data_ready = false;
    return dev->distance;
}

void ultrasonic_init(hcsr04_t *dev, uint trig_pin, uint echo_pin)
{
    dev->trig_pin = trig_pin;
    dev->echo_pin = echo_pin;
    dev->timer_period_us = 10;

    dev->estado = IDLE;
    dev->flag_rise = false;
    dev->flag_fall = false;
    dev->time_rise = 0;
    dev->time_fall = 0;

    gpio_init(trig_pin);
    gpio_init(echo_pin);
    gpio_set_dir(trig_pin, GPIO_OUT);
    gpio_set_dir(echo_pin, GPIO_IN);
    gpio_put(trig_pin, 0);

    instances[echo_pin] = dev;
    gpio_set_irq_enabled(echo_pin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
}

void ultrasonic_start(hcsr04_t *dev)
{
    if (dev->estado != IDLE)
        return;

    dev->flag_rise = false;
    dev->flag_fall = false;

    dev->time_rise = 0;
    dev->time_fall = 0;

    dev->distance = NO_CAPTURE_READY;
    dev->data_ready = false;

    dev->timeout = time_us_32();

    ultrasonic_trigger(dev);

    dev->estado = WAIT_RISE;
}

void ultrasonic_process(hcsr04_t *dev)
{
    switch(dev->estado)
    {
        case IDLE:
            break;

        case WAIT_RISE:

            if(time_us_32() - dev->timeout > 30000)
                {
                    dev->data_ready = true;
                    dev->distance = NO_CAPTURE_READY;
                    dev->estado = IDLE;
                }
            break;

        case WAIT_FALL:

            if(time_us_32() - dev->timeout > 30000)
            {
                dev->data_ready = true;
                dev->distance = NO_CAPTURE_READY;
                dev->estado = IDLE;
            }
            break;

        case DONE:

            ultrasonic_calculate_distance(dev);
            dev->data_ready = true;
            dev->estado = IDLE;
            break;

        default:
            dev->estado = IDLE;
            break;
    }
}