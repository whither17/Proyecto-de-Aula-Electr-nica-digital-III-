#include "Driver_TB6612FNG.h"


//Ciclos de dureza a aplicar en los motores en la proxima IRQ
static volatile uint16_t g_duty_left  = 0; /**< Ciclo de dureza del motor izquierdo */
static volatile uint16_t g_duty_right = 0; /**< Ciclo de dureza del motor derecho */
 
static volatile bool g_cmd_pending = false; /**< Banderea de cambio de estado */
static volatile uint32_t g_pwm_ticks = 0;   /**< Base de tiempo por medio del uso de las interrupciones */


/**
 * @brief Rutina de servicio de interrupción (ISR) del PWM.
 *
 * Se ejecuta al producirse un evento de wrap del slice PWM.
 * La ISR contabiliza los períodos transcurridos y aplica de
 * forma sincronizada los cambios de duty cycle pendientes.
 *
 * La actualización del duty se realiza en el instante de wrap
 * para evitar cambios a mitad de período y mantener ambas
 * salidas PWM sincronizadas.
 */
static void pwm_wrap_isr() 
{
    //Limpiamos la Flag
    pwm_clear_irq(PWMSlice);
    //Umentamos Ticks
    g_pwm_ticks++;

    // Aplicamos nuevo duty cycle si hay un comando pendiente
    if (g_cmd_pending) 
    {
        uint16_t limit = ((uint16_t)((MAXPWM * (int)PWM_WRAP) / 100));
        if (g_duty_left > limit)
        {
            pwm_set_chan_level(PWMSlice,  PWM_CHAN_A, limit);
        }
        else
        {
            pwm_set_chan_level(PWMSlice,  PWM_CHAN_A, g_duty_left);
        }
        if (g_duty_right > limit)
        {
            pwm_set_chan_level(PWMSlice,  PWM_CHAN_B, limit);
        }
        else
        {
            pwm_set_chan_level(PWMSlice,  PWM_CHAN_B, g_duty_right);
        }
        g_cmd_pending = false;
    }
}

/**
 * 
 * @brief Configura AIN1/AIN2 (motor izquierdo) según el signo de speed.
 * speed > 0  →  AIN1=1, AIN2=0  (adelante)
 * speed < 0  →  AIN1=0, AIN2=1  (atrás)
 * speed = 0  →  AIN1=0, AIN2=0  (freno)
 * 
 * @param speed Valor de velocidad
 */
static inline void set_motor_direction_left(int speed) 
{
    if (speed > 0) 
    {
        gpio_put(PIN_AIN1, 1);
        gpio_put(PIN_AIN2, 0);
    } 
    else if (speed < 0) 
    {
        gpio_put(PIN_AIN1, 0);
        gpio_put(PIN_AIN2, 1);
    } 
    else 
    {
        gpio_put(PIN_AIN1, 0);
        gpio_put(PIN_AIN2, 0);
    }
}

/**
 * @brief Configura BIN1/BIN2 (motor derecho) según el signo de speed.
 *   speed > 0  →  BIN1=1, BIN2=0  (adelante)
 *   speed < 0  →  BIN1=0, BIN2=1  (atrás)
 *   speed = 0  →  BIN1=0, BIN2=0  (freno)
 * 
 * @param Speed valor de velocidad
 */
static inline void set_motor_direction_right(int speed) 
{
    if (speed > 0) 
    {
        gpio_put(PIN_BIN1, 1);
        gpio_put(PIN_BIN2, 0);
    } 
    else if (speed < 0) 
    {
        gpio_put(PIN_BIN1, 0);
        gpio_put(PIN_BIN2, 1);
    } 
    else 
    {
        gpio_put(PIN_BIN1, 0);
        gpio_put(PIN_BIN2, 0);
    }
}

/**
 * @brief Convierte una consigna de velocidad en un duty cycle PWM.
 *
 * La entrada se interpreta como una velocidad normalizada en el
 * rango [-100, 100]. La magnitud se transforma linealmente en un
 * valor de comparación PWM comprendido entre 0 y PWM_WRAP.
 * Se aplica una zona muerta definida por minpwm para compensar el
 * umbral mínimo necesario para vencer la fricción y arrancar el
 * motor. Valores por debajo de este umbral producen duty cero.
 *
 * @param speed Consigna de velocidad normalizada [-100, 100].
 * @param minpwm Umbral mínimo efectivo del motor [%].
 *
 * @return Duty cycle PWM escalado al rango [0, PWM_WRAP].
 */
static inline uint16_t speed_to_duty(int speed, int minpwm) 
{
    //Saturar rango
    if (speed > 100) 
        speed = 100;

    if (speed < -100) 
        speed = -100;
 
    int mag = abs(speed);
 
    // Zona muerta
    if (mag < minpwm) 
        return 0;
 
    //Mapeo lineal [minpwm, MAXPWM] → [0, PWM_WRAP]
    int range  = MAXPWM - minpwm;
    int scaled = ((mag - minpwm) * (int)PWM_WRAP) / range;
 
    // Saturar al wrap máximo de ser necesario
    if (scaled > (int)PWM_WRAP) 
        scaled = (int)PWM_WRAP;
 
    return (uint16_t)scaled;
}

void motors_set(int left_speed, int right_speed) 
{
    //Zona muerta
    if (abs(left_speed) < MINPWM_LEFT) 
        left_speed  = 0;

    if (abs(right_speed) < MINPWM_RIGHT) 
        right_speed = 0;
 
    // Configurar dirección en los pines IN
    set_motor_direction_left(left_speed);
    set_motor_direction_right(right_speed);
 
    // Calcular duty con el MINPWM calibrado de cada canal
    g_duty_left = speed_to_duty(left_speed, MINPWM_LEFT);
    g_duty_right = speed_to_duty(right_speed, MINPWM_RIGHT);
 
    // Señalar a la ISR que hay un nuevo comando
    g_cmd_pending = true;
}

void motor_driver_init() 
{
    //Inicializar y configurar los pines
    const uint dir_pins[] = {PIN_AIN1, PIN_AIN2, PIN_BIN1, PIN_BIN2};
    for (int i = 0; i < 4; i++) 
    {
        gpio_init(dir_pins[i]);
        gpio_set_dir(dir_pins[i], GPIO_OUT);
        gpio_put(dir_pins[i], 0);
    }

    gpio_set_function(PIN_PWMA, GPIO_FUNC_PWM);   // GP12 → slice 6  ch-A 
    gpio_set_function(PIN_PWMB, GPIO_FUNC_PWM); 

    //Configuración por defecto
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv_int(&cfg, 1);
    pwm_config_set_wrap(&cfg, PWM_WRAP);

    //Motor Izquierdo 
    pwm_init(PWMSlice,  &cfg, false);
    pwm_set_chan_level(PWMSlice, PWM_CHAN_A, 0);
    //Motor derecho
    pwm_set_chan_level(PWMSlice, PWM_CHAN_B, 0);

    //Habilitar la interrupcion PWM
    
    pwm_clear_irq(PWMSlice);
    pwm_set_irq_enabled(PWMSlice,  true);

    // Registrar el handler y configurar prioridad
    irq_set_exclusive_handler(PWM_IRQ_WRAP, pwm_wrap_isr); //Registrar la Isr en el NVIC
    irq_set_priority(PWM_IRQ_WRAP, 0u);   //Prioridad máxima
    irq_set_enabled(PWM_IRQ_WRAP, true);  //Habilitar ña interrupción

    //Arranque

    //Habilitar el Slice para ambos canales
    pwm_set_enabled(PWMSlice,1);
}

uint32_t motor_get_ticks() 
{
    irq_set_enabled(PWM_IRQ_WRAP, false);
    uint32_t t = g_pwm_ticks;
    irq_set_enabled(PWM_IRQ_WRAP, true);
    return t;
}