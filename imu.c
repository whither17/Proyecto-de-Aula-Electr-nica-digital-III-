#include "imu.h"

/* ─────────────────────────────────────────────────────────────────────────────
   ESTADO INTERNO
   ───────────────────────────────────────────────────────────────────────────── */
static volatile bool g_data_ready = false; /**< Bandera que indica si la IMU tiene datos nuevos */
static imu_data_t    g_last_data  = {0};   /**< Estructura de datos de la IMU */
static alarm_id_t g_alarm_id = -1;         /**< ID de la alarma — necesario para reprogramarla dentro del callback */


/**
 * @brief Callback periódico de muestreo de la IMU.
 *
 * Esta función es ejecutada por la ISR del temporizador hardware.
 * Su única tarea es señalar que existe una nueva muestra pendiente.
 *
 * Retornar un valor negativo hace que el SDK reprograme la alarma
 * automáticamente usando como referencia el instante programado
 * anterior y no el instante real de ejecución. Esto evita la deriva
 * temporal acumulada y mantiene una frecuencia de muestreo estable.
 *
 * @param id Identificador de la alarma.
 * @param user_data Puntero de usuario (no utilizado).
 *
 * @return -IMU_SAMPLE_US para repetir la alarma periódicamente cada
 *         IMU_SAMPLE_US microsegundos.
 */
static int64_t imu_alarm_callback(alarm_id_t id, void *user_data)
{
   (void)id;
   (void)user_data;

   g_data_ready = true;

   return -(int64_t)IMU_SAMPLE_US;
}


//------------ Funciones Auxiliares ----------------//


/**
 * @brief Escribe un valor en un registro de la IMU
 * 
 * @param reg Dirección del registro
 * @param val Valor 
 * @return true Escritura exitosa
 * @return false Fallo en la escritura
 */
static bool imu_write_reg(uint8_t reg, uint8_t val) 
{
   uint8_t buf[2] = {reg, val};
   return i2c_write_blocking(IMU_I2C_PORT, MPU6050_ADDR, buf, 2, false) == 2;
}

/**
 * @brief Lee un registro o varios rigistros consecutivos y envía su resultado por referencia
 * 
 * @param reg Registro inicial a leer
 * @param dst Buffer de destino
 * @param len Longitud (número de bytes a leer)
 * @return true Lectura exitosa
 * @return false Error en la lectura
 */
static bool imu_read_regs(uint8_t reg, uint8_t *dst, size_t len) 
{
   //Verificar si la IMU está disponible
   if (i2c_write_blocking(IMU_I2C_PORT, MPU6050_ADDR, &reg, 1, true) != 1)
      return false;
   return i2c_read_blocking(IMU_I2C_PORT, MPU6050_ADDR, dst, len, false) == (int)len;
}

//---------------- Funciones Públicas ---------------//

bool imu_init() 
{

   //I2C
   i2c_init(IMU_I2C_PORT, IMU_I2C_FREQ);
   gpio_set_function(IMU_PIN_SDA, GPIO_FUNC_I2C);
   gpio_set_function(IMU_PIN_SCL, GPIO_FUNC_I2C);
   gpio_pull_up(IMU_PIN_SDA);
   gpio_pull_up(IMU_PIN_SCL);
   sleep_ms(100);

   //Verificar presencia del Chip
   uint8_t who = 0;
   //Identificar dispositivo
   if (!imu_read_regs(REG_WHO_AM_I, &who, 1)) 
      return false;
   if (who != 0x68) 
      return false;

   // Despertar chip, reloj desde PLL giroscopio X
   if (!imu_write_reg(REG_PWR_MGMT_1, 0x01)) 
      return false;
   sleep_ms(10);

   // DLPF modo 3 — 42 Hz gyro, Fs interno = 1 kHz
   if (!imu_write_reg(REG_CONFIG, 0x03)) 
      return false;

    // Sample Rate = 1000 / (1 + 9) = 100 Hz
   if (!imu_write_reg(REG_SMPLRT_DIV, 9)) 
      return false;

    // Giroscopio ±250 °/s
   if (!imu_write_reg(REG_GYRO_CONFIG, 0x00)) 
      return false;

    // Acelerómetro ±2 g
   if (!imu_write_reg(REG_ACCEL_CONFIG, 0x00)) 
      return false;

    // INT del chip deshabilitado — no hay pin conectado
   if (!imu_write_reg(REG_INT_ENABLE, 0x00)) 
      return false;

   // Arrancar la alarma de hardware, se dispara el callback una vez pasados IMU_SAMPLE_US.
   // El callback retorna negativo para que el SDK lo repita indefinidamente.

   g_alarm_id = add_alarm_in_us(IMU_SAMPLE_US, imu_alarm_callback, NULL, true);

   //Verificar alarma libre
   if (g_alarm_id < 0)
      return false;

    return true;
}

bool imu_data_ready(void) 
{
    return g_data_ready;
}

bool imu_read() 
{

   // Bajar bandera antes de leer para no perder el siguiente disparo
   g_data_ready = false;

   // Leer 14 bytes: AX AY AZ TEMP GX GY GZ (cada uno 2 bytes big-endian)
   uint8_t raw[14];
   if (!imu_read_regs(REG_ACCEL_XOUT_H, raw, 14)) 
      return false;

   int16_t ax = (int16_t)((raw[0] << 8) | raw[1]);
   int16_t ay = (int16_t)((raw[2] << 8) | raw[3]);
   int16_t az = (int16_t)((raw[4] << 8) | raw[5]);
   int16_t gx = (int16_t)((raw[8] << 8) | raw[9]);
   int16_t gy = (int16_t)((raw[10] << 8) | raw[11]);
   int16_t gz = (int16_t)((raw[12] << 8) | raw[13]);
   int16_t tmp = (int16_t)((raw[6] << 8) | raw[7]);

// Transformaciones lineales a unidades físicas
   g_last_data.accel_x = ax * ACCEL_SCALE;
   g_last_data.accel_y = ay * ACCEL_SCALE;
   g_last_data.accel_z = az * ACCEL_SCALE;

   g_last_data.gyro_x  = gx * GYRO_SCALE * DEG_TO_RAD;
   g_last_data.gyro_y  = gy * GYRO_SCALE * DEG_TO_RAD;
   g_last_data.gyro_z  = gz * GYRO_SCALE * DEG_TO_RAD;

   g_last_data.temp    = (tmp / 340.0f) + 36.53f;

   return true;
}

void imu_get_data(imu_data_t *out) 
{
    *out = g_last_data;
}