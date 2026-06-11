/**
 * @file    ProyectoFinal_test.c
 * @author  Andres
 * @date    2025
 *
 * @brief   Robot autónomo — barrido rotacional + detección ultrasónica
 *          + clasificación IR. Versión de TESTEO (con printf).
 *
 * @details
 * ## Arquitectura de control
 *
 * ### Polling + IRQ con WFI
 *
 * El loop principal duerme con `__wfi()` y despierta con cualquier IRQ
 * activa. Los handlers SOLO seteen banderas (acknowledgment mínimo);
 * todo el trabajo se hace en el loop dentro de bloques `if (flag_*)`.
 *
 *   - `flag_imu`   — seteada por `imu_alarm_callback` (~100 Hz).
 *                    Gobierna odometría, speed control, sonar y FSM.
 *   - `flag_pause` — seteada por `pause_alarm_callback` al expirar
 *                    la pausa de APP_ALARM o APP_SLEEPING.
 *                    Reactiva la alarma de IMU y reinicia el barrido.
 *
 * Durante APP_ALARM y APP_SLEEPING la alarma de IMU se cancela
 * (`imu_alarm_pause`) para que el robot no procese nada. El WFI
 * solo despertará cuando expire la alarma de pausa.
 *
 * ### FSM por tabla de punteros a funciones
 *
 * Cada estado tiene su propia función `state_*`. Un puntero
 * `state_fn` apunta al estado activo. Transicionar = reasignar el
 * puntero. No hay `switch` central.
 *
 * ```
 * SCANNING → BRAKING → WAITING → APPROACHING → READING_IR
 *                                                   ├─(blanco)→ TURNING_BACK → RETURNING → ALARM → SCANNING
 *                                                   └─(negro) → PUSHING     → TURNING_BACK → RETURNING → ALARM → SCANNING
 *
 * SCANNING →(timeout)→ SLEEPING → SCANNING
 * ```
 *
 * ### Nota sobre IRQ compartida
 *
 * Una única función `gpio_irq_handler` despacha a
 * `encoder_handle_irq()` y a `ultrasonic_eco_callback()`.
 * Ambos handlers filtran internamente por pin/estado — no se
 * necesitan guardas adicionales en el dispatcher.
 */

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"   /* __wfi()                    */
#include "Driver_TB6612FNG.h"
#include "encoder.h"
#include "imu.h"
#include "odometry.h"
#include "speedControl.h"
#include "ultrasonic.h"
#include "navigation.h"
#include <math.h>
#include <stdio.h>

/* =========================================================================
   PARÁMETROS CONFIGURABLES
   ========================================================================= */

#define DETECTION_DISTANCE_CM       35u
#define DETECTION_CONFIRM_COUNT     8u
#define APPROACH_DISTANCE_CM        5u
#define WAIT_AFTER_DETECT_CYCLES    200u
#define SCAN_TURN_PWM               49
#define BRAKE_PULSE_PWM             43
#define BRAKE_PULSE_MS              25
#define SCAN_DIRECTION              1
#define PUSH_EXIT_THRESHOLD_MM      400.0f
#define ALARM_DURATION_MS           2500u
#define SLEEP_DURATION_MS           5000u
#define SCAN_TIMEOUT_CYCLES         500u

/* =========================================================================
   PINES
   ========================================================================= */

#define ULTRASONIC_TRIG_PIN         4
#define ULTRASONIC_ECHO_PIN         5
#define IR_SENSOR_PIN               10
#define IR_WHITE                    0

/* =========================================================================
   PARÁMETROS FÍSICOS
   ========================================================================= */

#define ROBOT_MM_PER_TICK           0.1682f
#define ROBOT_BASELINE_MM           105.38f

/* =========================================================================
   CONSTANTES INTERNAS
   ========================================================================= */

#define PRINT_INTERVAL_CYCLES       10
#define IR_SETTLE_CYCLES            5

/* =========================================================================
   BANDERAS VOLÁTILES  (seteadas en IRQ, leídas en loop)
   ========================================================================= */

/** Seteada por imu_alarm_callback — indica sample listo. */
volatile bool flag_imu   = false;

/** Seteada por pause_alarm_callback — indica fin de pausa SLEEP/ALARM. */
volatile bool flag_pause = false;

/* =========================================================================
   ALARMA DE PAUSA  (APP_ALARM / APP_SLEEPING)
   ========================================================================= */

/**
 * @brief Callback de la alarma de pausa.
 *
 * Solo setea la bandera. El trabajo (reanudar IMU, reiniciar barrido)
 * se hace en el loop dentro de `if (flag_pause)`.
 * Retorna 0 → one-shot, no se rearma solo.
 */
static int64_t pause_alarm_callback(alarm_id_t id, void *user_data)
{
    (void)id;
    (void)user_data;
    flag_pause = true;
    return 0;
}

/* =========================================================================
   ESTADO COMPARTIDO DEL FSM
   ========================================================================= */

/** Estructura con todo el contexto mutable del FSM. */
typedef struct {
    uint32_t  cycle;
    uint32_t  last_distance;
    uint32_t  detect_count;
    uint32_t  wait_cycles;
    uint32_t  ir_read_cycles;
    uint32_t  scan_no_detect_cycles;
    float     return_dist_mm;
    /* APP_TURNING_BACK */
    float     theta_back;
    bool      theta_back_set;
} fsm_ctx_t;

static fsm_ctx_t ctx;

/* =========================================================================
   SONAR (global para acceso desde estados)
   ========================================================================= */

static hcsr04_t sonar;

/* =========================================================================
   FORWARD DECLARATIONS DE ESTADOS
   ========================================================================= */

static void state_scanning(void);
static void state_braking(void);
static void state_waiting(void);
static void state_approaching(void);
static void state_reading_ir(void);
static void state_pushing(void);
static void state_turning_back(void);
static void state_returning(void);
static void state_alarm(void);
static void state_sleeping(void);

/* =========================================================================
   PUNTERO AL ESTADO ACTIVO
   ========================================================================= */

typedef void (*state_fn_t)(void);
static state_fn_t state_fn = state_scanning;

/* =========================================================================
   HELPERS
   ========================================================================= */

static inline float rad_to_deg(float r)
{
    return r * (180.0f / (float)M_PI);
}

static void apply_scan_turn(void)
{
#if SCAN_DIRECTION >= 0
    motors_set(-SCAN_TURN_PWM,  SCAN_TURN_PWM);
#else
    motors_set( SCAN_TURN_PWM, -SCAN_TURN_PWM);
#endif
}

static void apply_brake_pulse(void)
{
#if SCAN_DIRECTION >= 0
    motors_set( BRAKE_PULSE_PWM, -BRAKE_PULSE_PWM);
#else
    motors_set(-BRAKE_PULSE_PWM,  BRAKE_PULSE_PWM);
#endif
    sleep_ms(BRAKE_PULSE_MS);   /* Único sleep bloqueante permitido:
                                   25 ms de pulso mecánico puro. */
    motors_set(0, 0);
}

/** Nombre del estado activo para telemetría. */
static const char *state_name(void)
{
    if (state_fn == state_scanning)    return "SCAN ";
    if (state_fn == state_braking)     return "BRAKE";
    if (state_fn == state_waiting)     return "WAIT ";
    if (state_fn == state_approaching) return "APROX";
    if (state_fn == state_reading_ir)  return "IR   ";
    if (state_fn == state_pushing)     return "PUSH ";
    if (state_fn == state_turning_back)return "TURN ";
    if (state_fn == state_returning)   return "RETRN";
    if (state_fn == state_alarm)       return "ALARM";
    if (state_fn == state_sleeping)    return "SLEEP";
    return "?????";
}

/* =========================================================================
   GPIO IRQ — callback unificada
   ========================================================================= */

/**
 * @brief Handler unificado de interrupciones GPIO.
 *
 * Acknowledgment mínimo: despacha a los sub-handlers que filtran
 * internamente. No setea banderas adicionales — encoder y sonar
 * escriben en sus propias estructuras/contadores.
 */
static void gpio_irq_handler(uint gpio, uint32_t events)
{
    encoder_handle_irq(gpio, events);
    ultrasonic_eco_callback(gpio, events);
}

/* =========================================================================
   IMPLEMENTACIÓN DE ESTADOS
   ========================================================================= */

/* ── APP_SCANNING ──────────────────────────────────────────────────────── */
static void state_scanning(void)
{
    apply_scan_turn();

    if (ctx.last_distance != NO_CAPTURE_READY &&
        ctx.last_distance  < (uint32_t)DETECTION_DISTANCE_CM)
    {
        ctx.detect_count++;
        ctx.scan_no_detect_cycles = 0;

        if (ctx.detect_count >= DETECTION_CONFIRM_COUNT)
        {
            motors_set(0, 0);
            speed_control_stop();
            ctx.detect_count          = 0;
            ctx.scan_no_detect_cycles = 0;

            printf("\n[DETECCION] Obstaculo confirmado a %lu cm"
                   " (%u lecturas) — frenando...\n",
                   (unsigned long)ctx.last_distance,
                   DETECTION_CONFIRM_COUNT);
            fflush(stdout);

            state_fn = state_braking;
        }
    }
    else
    {
        ctx.detect_count = 0;
        ctx.scan_no_detect_cycles++;

        if (ctx.scan_no_detect_cycles >= SCAN_TIMEOUT_CYCLES)
        {
            motors_set(0, 0);
            ctx.scan_no_detect_cycles = 0;

            printf("[SLEEP] %u ciclos sin deteccion — durmiendo %u ms...\n",
                   SCAN_TIMEOUT_CYCLES, SLEEP_DURATION_MS);
            fflush(stdout);

            state_fn = state_sleeping;
        }
    }
}

/* ── APP_BRAKING ───────────────────────────────────────────────────────── */
static void state_braking(void)
{
    apply_brake_pulse();
    ctx.wait_cycles = 0;

    printf("[ESPERA] Quieto 2 s antes de aproximarse...\n");
    fflush(stdout);

    state_fn = state_waiting;
}

/* ── APP_WAITING ───────────────────────────────────────────────────────── */
static void state_waiting(void)
{
    ctx.wait_cycles++;

    if (ctx.wait_cycles >= WAIT_AFTER_DETECT_CYCLES)
    {
        /* Lectura fresca del sonar (timeout 35 ms bloqueante mínimo
         * para tener distancia válida antes de arrancar). */
        ultrasonic_start(&sonar);
        uint32_t fresh_dist = NO_CAPTURE_READY;
        absolute_time_t t0  = get_absolute_time();

        while (absolute_time_diff_us(t0, get_absolute_time()) < 35000)
        {
            ultrasonic_process(&sonar);
            if (ultrasonic_ready(&sonar))
            {
                fresh_dist = ultrasonic_get_distance(&sonar);
                break;
            }
        }
        if (fresh_dist != NO_CAPTURE_READY)
            ctx.last_distance = fresh_dist;

        ultrasonic_start(&sonar);

        printf("[APROXIMACION] Dist fresca: %lu cm — avanzando hasta %d cm...\n",
               (unsigned long)ctx.last_distance, APPROACH_DISTANCE_CM);
        fflush(stdout);

        state_fn = state_approaching;
    }
}

/* ── APP_APPROACHING ───────────────────────────────────────────────────── */
static void state_approaching(void)
{
    if (ctx.last_distance != NO_CAPTURE_READY &&
        ctx.last_distance <= (uint32_t)APPROACH_DISTANCE_CM)
    {
        speed_control_stop();
        ctx.ir_read_cycles = 0;

        printf("[IR] Sonar confirma %lu cm — leyendo color...\n",
               (unsigned long)ctx.last_distance);
        fflush(stdout);

        state_fn = state_reading_ir;
    }
    else
    {
        sc_command_t cmd = { .v_left_mm_s = 98.0f, .v_right_mm_s = 98.0f };
        speed_control_set(&cmd);
    }
}

/* ── APP_READING_IR ────────────────────────────────────────────────────── */
static void state_reading_ir(void)
{
    ctx.ir_read_cycles++;

    if (ctx.ir_read_cycles < IR_SETTLE_CYCLES)
        return;

    bool obstacle_white = (gpio_get(IR_SENSOR_PIN) == (uint)IR_WHITE);
    pose_t pose;
    odometry_get_pose(&pose);

    if (obstacle_white)
    {
        ctx.return_dist_mm = sqrtf(pose.x * pose.x + pose.y * pose.y);

        printf("[IR] BLANCO — pose: (%.1f, %.1f) mm  theta: %.3f rad (%.1f deg)\n",
               (double)pose.x, (double)pose.y,
               (double)pose.theta, (double)rad_to_deg(pose.theta));
        printf("[IR] Distancia al origen: %.1f mm\n",
               (double)ctx.return_dist_mm);

        pose_t reset = { .x = 0.0f, .y = 0.0f, .theta = 0.0f };
        odometry_set_pose(&reset);
        nav_init();

        printf("[RESET] Pose reseteada a (0,0,0).  Bias: %+.5f rad/s\n",
               (double)odometry_get_bias());
        printf("[GIRO] Iniciando giro 180° — retorno en %.1f mm...\n",
               (double)ctx.return_dist_mm);
        fflush(stdout);

        ctx.theta_back_set = false;
        state_fn = state_turning_back;
    }
    else
    {
        printf("[IR] NEGRO — pose: (%.1f, %.1f) mm  theta: %.3f rad (%.1f deg)\n",
               (double)pose.x, (double)pose.y,
               (double)pose.theta, (double)rad_to_deg(pose.theta));
        printf("[IR] Iniciando empuje — salir de |x|>%.0f mm o |y|>%.0f mm...\n",
               (double)PUSH_EXIT_THRESHOLD_MM,
               (double)PUSH_EXIT_THRESHOLD_MM);
        fflush(stdout);

        sc_command_t cmd = { .v_left_mm_s = 98.0f, .v_right_mm_s = 98.0f };
        speed_control_set(&cmd);
        state_fn = state_pushing;
    }
}

/* ── APP_PUSHING ───────────────────────────────────────────────────────── */
static void state_pushing(void)
{
    pose_t pose;
    odometry_get_pose(&pose);

    if (fabsf(pose.x) > PUSH_EXIT_THRESHOLD_MM ||
        fabsf(pose.y) > PUSH_EXIT_THRESHOLD_MM)
    {
        speed_control_stop();
        motors_set(0, 0);

        ctx.return_dist_mm = sqrtf(pose.x * pose.x + pose.y * pose.y);

        printf("[PUSH] Caja fuera de región — pose: (%.1f, %.1f) mm\n",
               (double)pose.x, (double)pose.y);
        printf("[PUSH] Distancia al origen: %.1f mm\n",
               (double)ctx.return_dist_mm);

        pose_t reset = { .x = 0.0f, .y = 0.0f, .theta = 0.0f };
        odometry_set_pose(&reset);
        nav_init();

        printf("[RESET] Pose reseteada a (0,0,0).  Bias: %+.5f rad/s\n",
               (double)odometry_get_bias());
        printf("[GIRO] Iniciando giro 180° — retorno en %.1f mm...\n",
               (double)ctx.return_dist_mm);
        fflush(stdout);

        ctx.theta_back_set = false;
        state_fn = state_turning_back;
    }
    else
    {
        sc_command_t cmd = { .v_left_mm_s = 98.0f, .v_right_mm_s = 98.0f };
        speed_control_set(&cmd);
    }
}

/* ── APP_TURNING_BACK ──────────────────────────────────────────────────── */
static void state_turning_back(void)
{
    if (!ctx.theta_back_set)
    {
        pose_t p0;
        odometry_get_pose(&p0);
        ctx.theta_back = p0.theta + (float)M_PI;
        if (ctx.theta_back >  (float)M_PI) ctx.theta_back -= 2.0f * (float)M_PI;
        if (ctx.theta_back < -(float)M_PI) ctx.theta_back += 2.0f * (float)M_PI;
        ctx.theta_back_set = true;

        printf("[GIRO] theta_actual: %.3f rad — objetivo: %.3f rad (%.1f deg)\n",
               (double)p0.theta,
               (double)ctx.theta_back,
               (double)rad_to_deg(ctx.theta_back));
        fflush(stdout);
    }

    pose_t p;
    odometry_get_pose(&p);

    float err = ctx.theta_back - p.theta;
    if (err >  (float)M_PI) err -= 2.0f * (float)M_PI;
    if (err < -(float)M_PI) err += 2.0f * (float)M_PI;

    if (fabsf(err) < 0.03f)
    {
        motors_set(0, 0);
        ctx.theta_back_set = false;
        sleep_ms(80);   /* Estabilización de pose: bloqueante corto. */

        /* Actualizar pose tras estabilización. */
        while (!imu_data_ready()) tight_loop_contents();
        imu_read();
        odometry_update();
        odometry_get_pose(&p);

        /* Limpiar flag_imu que quedó pendiente durante sleep_ms(80). */
        flag_imu = false;

        float goal_x = ctx.return_dist_mm * cosf(p.theta);
        float goal_y = ctx.return_dist_mm * sinf(p.theta);

        nav_set_pos_tolerance(50.0f);
        nav_go_to(goal_x, goal_y);

        printf("[RETORNO] Giro OK — theta real: %.3f rad (%.1f deg)\n",
               (double)p.theta, (double)rad_to_deg(p.theta));
        printf("          Destino: (%.1f, %.1f) mm  dist: %.1f mm\n",
               (double)goal_x, (double)goal_y,
               (double)ctx.return_dist_mm);
        fflush(stdout);

        state_fn = state_returning;
    }
    else
    {
#if SCAN_DIRECTION >= 0
        motors_set( SCAN_TURN_PWM, -SCAN_TURN_PWM);
#else
        motors_set(-SCAN_TURN_PWM,  SCAN_TURN_PWM);
#endif
    }
}

/* ── APP_RETURNING ─────────────────────────────────────────────────────── */
static void state_returning(void)
{
    nav_update();

    if (nav_is_done())
    {
        nav_stop();

        pose_t pf;
        odometry_get_pose(&pf);
        float dist_final  = sqrtf(pf.x * pf.x + pf.y * pf.y);
        float err_retorno = dist_final - ctx.return_dist_mm;

        printf("[LLEGADA] Robot en posicion inicial.\n");
        printf("          Pose local: (%.1f, %.1f) mm  theta: %+.3f rad\n",
               (double)pf.x, (double)pf.y, (double)pf.theta);
        printf("          Error de retorno: %.1f mm\n", (double)err_retorno);
        printf("[ALARMA] Sonando %u ms...\n", ALARM_DURATION_MS);
        fflush(stdout);

        state_fn = state_alarm;
    }
}

/* ── APP_ALARM ─────────────────────────────────────────────────────────── */
/**
 * @brief Estado de alarma — se ejecuta UNA vez al entrar.
 *
 * Pausa la alarma de IMU, arma la alarma de pausa y devuelve el
 * control. El loop duerme con WFI hasta que `flag_pause` se setee.
 * El trabajo de salida se hace en el bloque `if (flag_pause)` del loop.
 */
static void state_alarm(void)
{
    /* Pausar IMU para que el WFI solo despierte con flag_pause. */
    imu_alarm_pause();

    /* Armar alarma de pausa one-shot. */
    add_alarm_in_ms(ALARM_DURATION_MS, pause_alarm_callback, NULL, true);

    /* Transicionar a un estado "nulo" — el loop no llamará state_fn
     * mientras flag_imu esté inactiva. La salida real ocurre en
     * el bloque if (flag_pause) del main loop. */
    state_fn = NULL;    /* Centinela: "esperando alarma de pausa". */
}

/* ── APP_SLEEPING ──────────────────────────────────────────────────────── */
/**
 * @brief Estado sleep — análogo a state_alarm.
 *
 * Resetea pose, pausa IMU, arma alarma de pausa.
 */
static void state_sleeping(void)
{
    pose_t reset = { .x = 0.0f, .y = 0.0f, .theta = 0.0f };
    odometry_set_pose(&reset);
    nav_init();
    ctx.return_dist_mm = 0.0f;

    printf("[SLEEP] Durmiendo %u ms...\n", SLEEP_DURATION_MS);
    fflush(stdout);

    imu_alarm_pause();
    add_alarm_in_ms(SLEEP_DURATION_MS, pause_alarm_callback, NULL, true);

    state_fn = NULL;    /* Centinela: esperando alarma de pausa. */
}

/* =========================================================================
   TELEMETRÍA
   ========================================================================= */

static void print_telemetry(void)
{
    if (ctx.cycle % PRINT_INTERVAL_CYCLES != 0)
        return;

    pose_t pose;
    odometry_get_pose(&pose);
    int32_t tL = encoder_get_left();
    int32_t tR = encoder_get_right();

    if (ctx.last_distance == NO_CAPTURE_READY)
    {
        printf("%-6s  th:%+7.2fdeg  x:%7.1f  y:%7.1f"
               "  tL:%6ld  tR:%6ld  dist:---\n",
               state_name(),
               (double)rad_to_deg(pose.theta),
               (double)pose.x, (double)pose.y,
               (long)tL, (long)tR);
    }
    else
    {
        printf("%-6s  th:%+7.2fdeg  x:%7.1f  y:%7.1f"
               "  tL:%6ld  tR:%6ld  dist:%3lu cm\n",
               state_name(),
               (double)rad_to_deg(pose.theta),
               (double)pose.x, (double)pose.y,
               (long)tL, (long)tR,
               (unsigned long)ctx.last_distance);
    }
    fflush(stdout);
}

/* =========================================================================
   MAIN
   ========================================================================= */

int main(void)
{
    stdio_init_all();

    uint32_t usb_wait = 0;
    while (!stdio_usb_connected() && usb_wait < 50)
    {
        sleep_ms(100);
        usb_wait++;
    }

    printf("\n╔══════════════════════════════════════════╗\n");
    printf("║  BARRIDO + DETECCIÓN HC-SR04 + SENSOR IR ║\n");
    printf("╠══════════════════════════════════════════╣\n");
    printf("║  Detección       : %2u cm (%u lecturas)   ║\n",
           DETECTION_DISTANCE_CM, DETECTION_CONFIRM_COUNT);
    printf("║  Aproximación    : %2u cm                 ║\n", APPROACH_DISTANCE_CM);
    printf("║  PWM barrido     : %2d %%                  ║\n", SCAN_TURN_PWM);
    printf("║  PWM freno       : %2d %%                  ║\n", BRAKE_PULSE_PWM);
    printf("║  Duración freno  : %2d ms                 ║\n", BRAKE_PULSE_MS);
#if SCAN_DIRECTION >= 0
    printf("║  Dirección       : IZQUIERDA              ║\n");
#else
    printf("║  Dirección       : DERECHA                ║\n");
#endif
    printf("║  IR sensor GPIO  : %2d                    ║\n", IR_SENSOR_PIN);
    printf("║  Retorno         : LOCAL (reset pose)     ║\n");
    printf("╚══════════════════════════════════════════╝\n\n");
    fflush(stdout);

    /* ── Encoders ── */
    printf("[INIT] Encoders... ");
    fflush(stdout);
    encoder_init();
    gpio_set_irq_enabled_with_callback(ENCODER_LEFT_A,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, gpio_irq_handler);
    gpio_set_irq_enabled(ENCODER_RIGHT_A,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    printf("OK\n");

    /* ── Motores ── */
    printf("[INIT] Motores... ");
    fflush(stdout);
    motor_driver_init();
    printf("OK\n");

    /* ── IMU ── */
    printf("[INIT] IMU... ");
    fflush(stdout);
    if (!imu_init())
    {
        printf("FALLO — detenido\n");
        while (true) tight_loop_contents();
    }
    printf("OK\n");

    /* ── Odometría ── */
    printf("[INIT] Odometria — robot QUIETO 2 s...\n");
    fflush(stdout);
    odo_params_t odo_params = {
        .mm_per_tick = ROBOT_MM_PER_TICK,
        .baseline    = ROBOT_BASELINE_MM
    };
    pose_t init_pose = { .x = 0.0f, .y = 0.0f, .theta = 0.0f };
    if (!odometry_init(&odo_params, &init_pose))
    {
        printf("FALLO IMU — detenido\n");
        while (true) tight_loop_contents();
    }
    printf("[INIT] Odometria OK  (bias gyro: %+.5f rad/s)\n",
           (double)odometry_get_bias());

    /* ── Speed control ── */
    printf("[INIT] Speed control... ");
    fflush(stdout);
    speed_control_init(ROBOT_MM_PER_TICK);
    printf("OK\n");

    /* ── Navegación ── */
    printf("[INIT] Navegacion... ");
    fflush(stdout);
    nav_init();
    printf("OK\n");

    /* ── Sonar ── */
    printf("[INIT] Ultrasonido (TRIG=%d, ECHO=%d)... ",
           ULTRASONIC_TRIG_PIN, ULTRASONIC_ECHO_PIN);
    fflush(stdout);
    ultrasonic_init(&sonar, ULTRASONIC_TRIG_PIN, ULTRASONIC_ECHO_PIN);
    printf("OK\n");

    /* ── Sensor IR ── */
    printf("[INIT] Sensor IR (GPIO %d)... ", IR_SENSOR_PIN);
    fflush(stdout);
    gpio_init(IR_SENSOR_PIN);
    gpio_set_dir(IR_SENSOR_PIN, GPIO_IN);
    gpio_pull_up(IR_SENSOR_PIN);
    printf("OK\n\n");

    printf("[DIAG] ticks pre-loop: L=%ld  R=%ld\n",
           (long)encoder_get_left(), (long)encoder_get_right());
    fflush(stdout);

    for (int i = 3; i >= 1; i--)
    {
        printf("  %d...\n", i); fflush(stdout); sleep_ms(1000);
    }
    printf("  INICIANDO BARRIDO\n\n");
    printf("%-6s  %-12s  %-8s  %-8s  %-6s  %-6s  %-10s\n",
           "ESTADO", "THETA(deg)", "X(mm)", "Y(mm)", "tL", "tR", "DIST(cm)");
    printf("──────  ────────────  ────────  ────────  ──────  ──────  ──────────\n");
    fflush(stdout);

    /* ── Contexto inicial ── */
    ctx = (fsm_ctx_t){ .last_distance = NO_CAPTURE_READY };

    ultrasonic_start(&sonar);
    apply_scan_turn();
    state_fn = state_scanning;
    imu_alarm_resume();
    /* ══════════════════════════════════════════════════════════════════
       BUCLE PRINCIPAL
       ──────────────────────────────────────────────────────────────────
       WFI como estado base: el core duerme hasta que cualquier IRQ
       lo despierta. El trabajo se hace SIEMPRE dentro de bloques
       if (flag_*), nunca en el handler.
       ══════════════════════════════════════════════════════════════════ */
    while (true)
    {
        __wfi();    /* Dormir hasta la próxima IRQ. */

        /* ── Bloque IMU: todo el procesamiento normal ── */
        if (flag_imu)
        {
            flag_imu = false;

            imu_read();
            odometry_update();

            /* Speed control solo en estados que lo requieren. */
            if (state_fn == state_approaching ||
                state_fn == state_pushing      ||
                state_fn == state_returning)
            {
                speed_control_update();
            }

            /* Sonar: bombear máquina de estados y recolectar medida. */
            ultrasonic_process(&sonar);
            if (ultrasonic_ready(&sonar))
            {
                ctx.last_distance = ultrasonic_get_distance(&sonar);
                ultrasonic_start(&sonar);
            }

            /* Ejecutar estado activo (NULL = esperando alarma de pausa). */
            if (state_fn != NULL)
                state_fn();

            /* Telemetría periódica. */
            print_telemetry();
            ctx.cycle++;
        }

        /* ── Bloque PAUSA: fin de ALARM o SLEEPING ── */
        if (flag_pause)
        {
            flag_pause = false;

            /* Reiniciar contexto para nuevo ciclo de barrido. */
            ctx.detect_count          = 0;
            ctx.scan_no_detect_cycles = 0;
            ctx.return_dist_mm        = 0.0f;
            ctx.theta_back_set        = false;

            /* Alarma solo necesita reset si venimos de APP_ALARM
             * (en APP_SLEEPING la pose ya se reseteó al entrar). */
            if (state_fn == NULL)
            {
                pose_t reset = { .x = 0.0f, .y = 0.0f, .theta = 0.0f };
                odometry_set_pose(&reset);
                nav_init();
            }

            printf("[WAKE] Reanudando barrido.\n\n");
            fflush(stdout);

            /* Reanudar alarma de IMU y arrancar barrido. */
            imu_alarm_resume();
            apply_scan_turn();
            state_fn = state_scanning;
        }

    } /* while (true) */

    return 0;
}