#include "imu.h"
#include "main.h"

#define MPU6500_ADDR        0x68 << 1
#define WHO_AM_I_REG        0x75
#define PWR_MGMT_1_REG      0x6B
#define ACCEL_OUT_REG       0x3B   // starting register for accel + gyro (14 bytes)

// Коэффициенты для MPU6500 (±2g, ±250 dps)
#define ACCEL_SCALE_FACTOR  16384.0f
#define GYRO_SCALE_FACTOR   131.0f

static float yaw_angle = 0.0f;      // интегральный угол (градусы)
static float gyro_z_offset = 0.0f;  // смещение гироскопа (dps)
static bool is_calibrated = false;

// Чтение 14 байт (акселерометр + температура + гироскоп)
static HAL_StatusTypeDef mpu_read_all(uint8_t *buffer) {
    return HAL_I2C_Mem_Read(&hi2c1, MPU6500_ADDR, ACCEL_OUT_REG, I2C_MEMADD_SIZE_8BIT, buffer, 14, 100);
}

// Запись одного байта в регистр
static HAL_StatusTypeDef mpu_write_reg(uint8_t reg, uint8_t value) {
    return HAL_I2C_Mem_Write(&hi2c1, MPU6500_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 100);
}

// Калибровка смещения гироскопа (усреднение 100 измерений)
static void calibrate_gyro(void) {
    int32_t sum_z = 0;
    uint8_t raw[14];
    int16_t gz;
    const int samples = 100;

    for (int i = 0; i < samples; i++) {
        if (mpu_read_all(raw) == HAL_OK) {
            gz = (int16_t)((raw[12] << 8) | raw[13]);
            sum_z += gz;
        }
        HAL_Delay(5);
    }
    float avg = (float)sum_z / samples;
    gyro_z_offset = avg / GYRO_SCALE_FACTOR;  // преобразуем в dps
    is_calibrated = true;
}

bool imu_init(void) {
    uint8_t whoami = 0;
    if (HAL_I2C_Mem_Read(&hi2c1, MPU6500_ADDR, WHO_AM_I_REG, I2C_MEMADD_SIZE_8BIT, &whoami, 1, 100) != HAL_OK)
        return false;
    if (whoami != 0x70 && whoami != 0x68)  // MPU6500 обычно 0x70, но бывает 0x68
        return false;

    // 2. Вывести из сна (PWR_MGMT_1 = 0)
    if (mpu_write_reg(PWR_MGMT_1_REG, 0x00) != HAL_OK)
        return false;
    HAL_Delay(10);

    // 3. Калибровка гироскопа (робот должен быть неподвижен)
    calibrate_gyro();

    yaw_angle = 0.0f;
    return true;
}

void imu_reset_yaw(void) {
    yaw_angle = 0.0f;
}

float imu_get_yaw(void) {
    static uint32_t last_time = 0;
    uint32_t now = HAL_GetTick();
    float dt = (last_time == 0) ? 0.01f : (now - last_time) / 1000.0f;
    last_time = now;
    if (dt > 0.05f) dt = 0.02f;  // защита от слишком большого dt

    // Получаем сырые данные
    uint8_t raw[14];
    if (mpu_read_all(raw) != HAL_OK) return yaw_angle;

    int16_t gz = (int16_t)((raw[12] << 8) | raw[13]);
    float gyro_z_dps = (float)gz / GYRO_SCALE_FACTOR - gyro_z_offset;

    // Интегрируем (простое накопление, для поворотов достаточно)
    yaw_angle += gyro_z_dps * dt;

    // Нормализация в пределах -180..+180 (опционально)
    // if (yaw_angle > 180.0f) yaw_angle -= 360.0f;
    // if (yaw_angle < -180.0f) yaw_angle += 360.0f;

    return yaw_angle;
}

float imu_get_gyro_z_dps(void) {
    uint8_t raw[14];
    if (mpu_read_all(raw) != HAL_OK) return 0.0f;
    int16_t gz = (int16_t)((raw[12] << 8) | raw[13]);
    return (float)gz / GYRO_SCALE_FACTOR - gyro_z_offset;
}

void imu_get_raw(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz) {
    uint8_t raw[14];
    if (mpu_read_all(raw) != HAL_OK) {
        *ax = *ay = *az = *gx = *gy = *gz = 0;
        return;
    }
    *ax = (int16_t)((raw[0] << 8) | raw[1]);
    *ay = (int16_t)((raw[2] << 8) | raw[3]);
    *az = (int16_t)((raw[4] << 8) | raw[5]);
    *gx = (int16_t)((raw[8] << 8) | raw[9]);
    *gy = (int16_t)((raw[10] << 8) | raw[11]);
    *gz = (int16_t)((raw[12] << 8) | raw[13]);
}

void imu_get_scaled(float *ax_g, float *ay_g, float *az_g, float *gx_dps, float *gy_dps, float *gz_dps) {
    int16_t ax, ay, az, gx, gy, gz;
    imu_get_raw(&ax, &ay, &az, &gx, &gy, &gz);
    *ax_g = (float)ax / ACCEL_SCALE_FACTOR;
    *ay_g = (float)ay / ACCEL_SCALE_FACTOR;
    *az_g = (float)az / ACCEL_SCALE_FACTOR;
    *gx_dps = (float)gx / GYRO_SCALE_FACTOR - gyro_z_offset; // по оси Z коррекция
    *gy_dps = (float)gy / GYRO_SCALE_FACTOR;
    *gz_dps = (float)gz / GYRO_SCALE_FACTOR - gyro_z_offset;
}
