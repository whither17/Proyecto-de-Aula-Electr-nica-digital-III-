#include "imu.h"

/* ─────────────────────────────────────────────────────────────────────────────
   REGISTROS
   ───────────────────────────────────────────────────────────────────────────── */
#define REG_SMPLRT_DIV   0x19
#define REG_CONFIG       0x1A
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_INT_ENABLE   0x38
#define REG_ACCEL_XOUT_H 0x3B
#define REG_PWR_MGMT_1   0x6B
#define REG_WHO_AM_I     0x75

/* ─────────────────────────────────────────────────────────────────────────────
   CONVERSIÓN
   ───────────────────────────────────────────────────────────────────────────── */
#define GYRO_SCALE   (1.0f / 131.0f)
#define DEG_TO_RAD   (3.14159265f / 180.0f)
#define ACCEL_SCALE  (9.80665f / 16384.0f)

/* ─────────────────────────────────────────────────────────────────────────────
   ESTADO INTERNO
   ───────────────────────────────────────────────────────────────────────────── */
static volatile bool g_data_ready = false;
static imu_data_t    g_last_data  = {0};

/* ID de la alarma — necesario para reprogramarla dentro del callback */
static alarm_id_t g_alarm_id = -1;

/* ─────────────────────────────────────────────────────────────────────────────
   ALARMA DE HARDWARE — callback a 100 Hz
   ───────────────────────────────────────────────────────────────────────────── */

/**
 * imu_alarm_callback()
 *
 * El SDK llama a esta función desde la ISR del timer de hardware.
 * Retornar un valor negativo hace que el SDK reprograme la alarma
 * automáticamente restando ese valor en µs al tiempo ideal —
 * esto compensa el jitter de la ISR y mantiene la cadencia exacta.
 *
 * Retornar IMU_SAMPLE_US negativo = reprogramar 10 ms después del
 * disparo ideal (no del momento real), lo que evita deriva acumulada.
 */
static int64_t imu_alarm_callback(alarm_id_t id, void *user_data) {
    (void)id;
    (void)user_data;

    g_data_ready = true;

    /* Retorno negativo → el SDK reprograma la alarma automáticamente
       con período exacto de IMU_SAMPLE_US desde el disparo anterior */
    return -(int64_t)IMU_SAMPLE_US;
}

/* ─────────────────────────────────────────────────────────────────────────────
   HELPERS I2C
   ───────────────────────────────────────────────────────────────────────────── */

static bool imu_write_reg(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    return i2c_write_blocking(IMU_I2C_PORT, MPU6050_ADDR, buf, 2, false) == 2;
}

static bool imu_read_regs(uint8_t reg, uint8_t *dst, size_t len) {
    if (i2c_write_blocking(IMU_I2C_PORT, MPU6050_ADDR, &reg, 1, true) != 1)
        return false;
    return i2c_read_blocking(IMU_I2C_PORT, MPU6050_ADDR, dst, len, false) == (int)len;
}

/* ─────────────────────────────────────────────────────────────────────────────
   INICIALIZACIÓN
   ───────────────────────────────────────────────────────────────────────────── */

bool imu_init(void) {

    /* ── I2C ── */
    i2c_init(IMU_I2C_PORT, IMU_I2C_FREQ);
    gpio_set_function(IMU_PIN_SDA, GPIO_FUNC_I2C);
    gpio_set_function(IMU_PIN_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(IMU_PIN_SDA);
    gpio_pull_up(IMU_PIN_SCL);
    sleep_ms(100);  /* esperar power-on reset del MPU6050 */

    /* ── Verificar presencia ── */
    uint8_t who = 0;
    if (!imu_read_regs(REG_WHO_AM_I, &who, 1)) return false;
    if (who != 0x68) return false;

    /* ── Despertar chip, reloj desde PLL giroscopio X ── */
    if (!imu_write_reg(REG_PWR_MGMT_1, 0x01)) return false;
    sleep_ms(10);

    /* ── DLPF modo 3 — 42 Hz gyro, Fs interno = 1 kHz ── */
    if (!imu_write_reg(REG_CONFIG, 0x03)) return false;

    /* ── Sample Rate = 1000 / (1 + 9) = 100 Hz ── */
    if (!imu_write_reg(REG_SMPLRT_DIV, 9)) return false;

    /* ── Giroscopio ±250 °/s ── */
    if (!imu_write_reg(REG_GYRO_CONFIG, 0x00)) return false;

    /* ── Acelerómetro ±2 g ── */
    if (!imu_write_reg(REG_ACCEL_CONFIG, 0x00)) return false;

    /* ── INT del chip deshabilitado — no hay pin conectado ── */
    if (!imu_write_reg(REG_INT_ENABLE, 0x00)) return false;

    /* ── Arrancar la alarma de hardware ──
       add_alarm_in_us() dispara el callback una vez pasados IMU_SAMPLE_US.
       El callback retorna negativo para que el SDK lo repita indefinidamente. */
    g_alarm_id = add_alarm_in_us(IMU_SAMPLE_US, imu_alarm_callback, NULL, true);
    if (g_alarm_id < 0) return false;  /* no había slot de alarma libre */

    return true;
}

/* ─────────────────────────────────────────────────────────────────────────────
   API PÚBLICA
   ───────────────────────────────────────────────────────────────────────────── */

bool imu_data_ready(void) {
    return g_data_ready;
}

bool imu_read(void) {

    /* Bajar bandera antes de leer para no perder el siguiente disparo */
    g_data_ready = false;

    /* 14 bytes: AX AY AZ TEMP GX GY GZ (cada uno 2 bytes big-endian) */
    uint8_t raw[14];
    if (!imu_read_regs(REG_ACCEL_XOUT_H, raw, 14)) return false;

    int16_t ax  = (int16_t)((raw[0]  << 8) | raw[1]);
    int16_t ay  = (int16_t)((raw[2]  << 8) | raw[3]);
    int16_t az  = (int16_t)((raw[4]  << 8) | raw[5]);
    int16_t gx  = (int16_t)((raw[8]  << 8) | raw[9]);
    int16_t gy  = (int16_t)((raw[10] << 8) | raw[11]);
    int16_t gz  = (int16_t)((raw[12] << 8) | raw[13]);
    int16_t tmp = (int16_t)((raw[6]  << 8) | raw[7]);

    g_last_data.accel_x = ax * ACCEL_SCALE;
    g_last_data.accel_y = ay * ACCEL_SCALE;
    g_last_data.accel_z = az * ACCEL_SCALE;

    g_last_data.gyro_x  = gx * GYRO_SCALE * DEG_TO_RAD;
    g_last_data.gyro_y  = gy * GYRO_SCALE * DEG_TO_RAD;
    g_last_data.gyro_z  = gz * GYRO_SCALE * DEG_TO_RAD;

    g_last_data.temp    = (tmp / 340.0f) + 36.53f;

    return true;
}

void imu_get_data(imu_data_t *out) {
    *out = g_last_data;
}