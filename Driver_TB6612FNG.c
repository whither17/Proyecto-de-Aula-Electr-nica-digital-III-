#include "Driver_TB6612FNG.h"

/* ─────────────────────────────────────────────────────────────────────────────
   DEFINICIÓN DE PINES Y CONSTANTES
   ───────────────────────────────────────────────────────────────────────────── */

/* Motor izquierdo — canal A */
#define PIN_AIN1   19  /* GP20 — dirección izquierdo bit-0 */
#define PIN_AIN2   18  /* GP21 — dirección izquierdo bit-1 */
#define PIN_PWMA   13  /* GP12 — PWM slice 6, canal A      */

/* Motor derecho — canal B */
#define PIN_BIN1   20  /* GP19 — dirección derecho bit-0   */
#define PIN_BIN2   21  /* GP18 — dirección derecho bit-1   */
#define PIN_PWMB   12  /* GP13 — PWM slice 6, canal B     */

/* CONSTANTES */
#define MAXPWM 70 /* Los motores solo reciben hasta 6v la alimentacion es de 7.4v con valores de hasta 8.4v usamos un 70% como segurridad*/
#define PWM_WRAP 12499 /* Para una frecuencia de 10 KHz con reloj del micro de 125 MHz*/
#define PWMSlice 6 /* El canal de PWM a usar es el 6 ya que este es el del GPIO 12(A) y del GPIO 13(B) */

/* ─────────────────────────────────────────────────────────────────────────────
   Variables Internas
   ───────────────────────────────────────────────────────────────────────────── */

/* Ciclos de dureza a aplicar en los motores en la proxima IRQ */
static volatile uint16_t g_duty_left  = 0;
static volatile uint16_t g_duty_right = 0;

/* Banderea de cambio de estado */
static volatile bool g_cmd_pending = false;

/* Base de tiempo por medio del uso de las interrupciones */
static volatile uint32_t g_pwm_ticks = 0;


/* Handler de la interrupcion*/
static void pwm_wrap_isr(void) {
    /* Limpiamos la Flag */
    pwm_clear_irq(PWMSlice);
    g_pwm_ticks++;

    /* Aplicar nuevo duty cycle si hay un comando pendiente */
    if (g_cmd_pending) {
        uint16_t limit =((uint16_t)((MAXPWM * (int)PWM_WRAP) / 100));
        if (g_duty_left > limit)
        {
            pwm_set_chan_level(PWMSlice,  PWM_CHAN_A, limit);
        }else{
            pwm_set_chan_level(PWMSlice,  PWM_CHAN_A, g_duty_left);
        }
        if (g_duty_right > limit)
        {
            pwm_set_chan_level(PWMSlice,  PWM_CHAN_B, limit);
        }else{
            pwm_set_chan_level(PWMSlice,  PWM_CHAN_B, g_duty_right);
        }
        g_cmd_pending = false;
    }
}

/**
 * set_motor_direction_left()
 * Configura AIN1/AIN2 según el signo de speed.
 *
 *   speed > 0  →  AIN1=1, AIN2=0  (adelante)
 *   speed < 0  →  AIN1=0, AIN2=1  (atrás)
 *   speed = 0  →  AIN1=0, AIN2=0  (freno)
 */
static inline void set_motor_direction_left(int speed) {
    if (speed > 0) {
        gpio_put(PIN_AIN1, 1);
        gpio_put(PIN_AIN2, 0);
    } else if (speed < 0) {
        gpio_put(PIN_AIN1, 0);
        gpio_put(PIN_AIN2, 1);
    } else {
        gpio_put(PIN_AIN1, 0);
        gpio_put(PIN_AIN2, 0);
    }
}

/**
 * set_motor_direction_right()
  * Configura BIN1/BIN2 según el signo de speed.
 *
 *   speed > 0  →  BIN1=1, BIN2=0  (adelante)
 *   speed < 0  →  BIN1=0, BIN2=1  (atrás)
 *   speed = 0  →  BIN1=0, BIN2=0  (freno)
 */
static inline void set_motor_direction_right(int speed) {
    if (speed > 0) {
        gpio_put(PIN_BIN1, 1);
        gpio_put(PIN_BIN2, 0);
    } else if (speed < 0) {
        gpio_put(PIN_BIN1, 0);
        gpio_put(PIN_BIN2, 1);
    } else {
        gpio_put(PIN_BIN1, 0);
        gpio_put(PIN_BIN2, 0);
    }
}

/**
 * speed_to_duty()
 * Convierte porcentaje [-100, 100] a valor de wrap [0, PWM_WRAP].
 * La dirección va por los pines IN, no por el duty — siempre positivo.
 */
static inline uint16_t speed_to_duty(int speed, int minpwm) {
 
    /* Saturar rango */
    if (speed >  100) speed =  100;
    if (speed < -100) speed = -100;
 
    int mag = abs(speed);
 
    /* Zona muerta → freno limpio, nunca stall */
    if (mag < minpwm) return 0;
 
    /* Mapeo lineal [minpwm, MAXPWM] → [0, PWM_WRAP] */
    int range  = MAXPWM - minpwm;
    int scaled = ((mag - minpwm) * (int)PWM_WRAP) / range;
 
    /* Saturar al wrap máximo por si acaso */
    if (scaled > (int)PWM_WRAP) scaled = (int)PWM_WRAP;
 
    return (uint16_t)scaled;
}

/**
 * motors_set()
 * pasamos de un valor de velocidad cruda a la organizacion de los diferentes pines
 * y el ciclo de dureza de los canales PWM
 */
void motors_set(int left_speed, int right_speed) {
 
    /*
     *  Zona muerta por canal: si la magnitud no alcanza el umbral
     *  del motor, forzar a 0 para que set_motor_direction ponga
     *  AINx=0 → freno activo, no stall.
     */
    if (abs(left_speed)  < MINPWM_LEFT)  left_speed  = 0;
    if (abs(right_speed) < MINPWM_RIGHT) right_speed = 0;
 
    /* Configurar dirección en los pines IN */
    set_motor_direction_left(left_speed);
    set_motor_direction_right(right_speed);
 
    /* Calcular duty con el MINPWM calibrado de cada canal */
    g_duty_left  = speed_to_duty(left_speed,  MINPWM_LEFT);
    g_duty_right = speed_to_duty(right_speed, MINPWM_RIGHT);
 
    /* Señalar a la ISR que hay un nuevo comando */
    g_cmd_pending = true;
}

void motor_driver_init(void) {

    /* ── 7.1  GPIO de dirección y STBY ── */
    const uint dir_pins[] = {PIN_AIN1, PIN_AIN2, PIN_BIN1, PIN_BIN2};
    for (int i = 0; i < 4; i++) {
        gpio_init(dir_pins[i]);
        gpio_set_dir(dir_pins[i], GPIO_OUT);
        gpio_put(dir_pins[i], 0);
    }

    /* ── 7.2  Función PWM en los pines de potencia ── */
    gpio_set_function(PIN_PWMA, GPIO_FUNC_PWM);   /* GP12 → slice 6  ch-A */
    gpio_set_function(PIN_PWMB, GPIO_FUNC_PWM);   /* GP21 → slice 10 ch-B */

    /* ── 7.3  Configuración de los dos slices con parámetros idénticos ── */
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv_int(&cfg, 1);
    pwm_config_set_wrap(&cfg, PWM_WRAP);

    /* Slice 6  — motor izquierdo (GP12, canal A) */
    pwm_init(PWMSlice,  &cfg, false);
    pwm_set_chan_level(PWMSlice,  PWM_CHAN_A, 0);
    pwm_set_chan_level(PWMSlice, PWM_CHAN_B, 0);

    /* ── 7.4  Interrupción PWM ── */

    /*
     *  Habilitar la IRQ de wrap en AMBOS slices.
     *  El RP2040 tiene una única línea PWM_IRQ_WRAP — la misma ISR
     *  recibe la interrupción de cualquiera de los dos slices.
     *  Por eso la ISR limpia los flags de los dos.
     */
    pwm_clear_irq(PWMSlice);
    pwm_set_irq_enabled(PWMSlice,  true);

    /* Registrar el handler y configurar prioridad */
    irq_set_exclusive_handler(PWM_IRQ_WRAP, pwm_wrap_isr);
    irq_set_priority(PWM_IRQ_WRAP, 0u);   /* 0 = máxima en Cortex-M0+ */
    irq_set_enabled(PWM_IRQ_WRAP, true);

    /* ── 7.5  Arranque ── */

    /* Activar ambos slices simultáneamente para minimizar desfase inicial */
    pwm_set_enabled(PWMSlice,1);
}
uint32_t motor_get_ticks(void) {
    irq_set_enabled(PWM_IRQ_WRAP, false);
    uint32_t t = g_pwm_ticks;
    irq_set_enabled(PWM_IRQ_WRAP, true);
    return t;
}