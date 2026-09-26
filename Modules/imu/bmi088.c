/* BMI088 寄存器配置与量程参考原工程 src/modules/imu/bmi088。
 * SPI dummy、温度字节顺序按 Bosch BMI088 数据手册核对。
 * 不沿用其他板子的硬编码 gyro_offset / gNORM，后续对本机标定。 */
#include "bmi088.h"
#include "bsp_imu.h"
volatile Bmi088Debug debug_bmi088;
static bool identify(Bmi088State *imu) // 作为在线判断
{
  return bsp_imu_read(BMI088_ACCEL, 0x00, &imu->accel_id, 1) &&
         bsp_imu_read(BMI088_GYRO, 0x00, &imu->gyro_id, 1) &&
         imu->accel_id == 0x1e && imu->gyro_id == 0x0f;
}
static bool write_checked(Bmi088Chip chip, uint8_t reg, uint8_t value) {
  uint8_t actual = 0;
  /* GYRO_BANDWIDTH bit7 恒为1，只核对低3位带宽字段。其他配置仍严格校验。 */
  uint8_t mask = (chip == BMI088_GYRO && reg == 0x10) ? 0x07 : 0xff;
  debug_bmi088.reg = reg;
  debug_bmi088.expected = value;
  debug_bmi088.mask = mask;
  if (!bsp_imu_write(chip, reg, value))
    return false;
  bsp_imu_delay_ms(1); /* 满足寄存器写入间隔，初始化阶段不追求极限速度。 */
  bool ok = bsp_imu_read(chip, reg, &actual, 1);
  debug_bmi088.actual = actual;
  return ok && (actual & mask) == (value & mask);
}
static bool initialize(Bmi088State *imu) {
  imu->initialized = imu->valid = false;
  imu->accel_id = imu->gyro_id = 0;
  if (!bsp_imu_ready())
    return false;
  uint8_t ignored;
  /* 首次事务唤醒加速度计 SPI 接口，其数据不能用于识别。 */
  if (!bsp_imu_read(BMI088_ACCEL, 0, &ignored, 1))
    return false;
  bsp_imu_delay_ms(2);
  if (!identify(imu))
    return false;
  // software reset
  if (!bsp_imu_write(BMI088_ACCEL, 0x7e, 0xb6) ||
      !bsp_imu_write(BMI088_GYRO, 0x14, 0xb6))
    return false;
  bsp_imu_delay_ms(40); /* 包含陀螺仪复位的 >30ms 等待。 */
  // continue to wake the BMI088
  if (!bsp_imu_read(BMI088_ACCEL, 0, &ignored, 1))
    return false;
  bsp_imu_delay_ms(2);
  if (!identify(imu))
    return false;

  if (!write_checked(BMI088_ACCEL, 0x7c, 0x00))
    return false; /* active */
  if (!write_checked(BMI088_ACCEL, 0x7d, 0x04))
    return false; /* enable */
  bsp_imu_delay_ms(55);

  if (!write_checked(BMI088_ACCEL, 0x41, 0x01) || /* +-6g */
      !write_checked(BMI088_ACCEL, 0x40, 0xab) || /* normal BW, 800Hz */
      !write_checked(BMI088_GYRO, 0x0f, 0x00) ||  /* +-2000 deg/s */
      !write_checked(BMI088_GYRO, 0x10, 0x01) ||  /* 2000Hz, 230Hz BW */
      !write_checked(BMI088_GYRO, 0x11, 0x00))
    return false;
  bsp_imu_delay_ms(5);
  imu->initialized = true;
  return true;
}
bool bmi088_init(Bmi088State *imu) {
  debug_bmi088.init_attempts++;
  debug_bmi088.reg = 0;
  debug_bmi088.actual = 0;
  bool ok = initialize(imu);
  debug_bmi088.accel_id = imu->accel_id;
  debug_bmi088.gyro_id = imu->gyro_id;
  if (!ok)
    debug_bmi088.init_failures++;
  return ok;
}
static int16_t signed_le16(const uint8_t *p) {
  uint16_t v = (uint16_t)p[0] | ((uint16_t)p[1] << 8);
  return (int16_t)(v & 0x8000u ? (int32_t)v - 65536 : v);
}
bool bmi088_read(Bmi088State *imu) {
  imu->valid = false;
  if (!imu->initialized)
    return false;
  uint8_t accel[6], gyro[6], temp[2];
  /* HAL_OK 只表示 SPI 事务完成，读回芯片ID能识别常见的断线全0/全FF。
   * 芯片ID不是自检或数据就绪判定；姿态有效性由 INS 层继续检查。 */
  if (!identify(imu) || !bsp_imu_read(BMI088_ACCEL, 0x12, accel, 6) ||
      !bsp_imu_read(BMI088_GYRO, 0x02, gyro, 6) ||
      !bsp_imu_read(BMI088_ACCEL, 0x22, temp, 2)) {
    imu->errors++;
    imu->initialized = false;
    return false;
  }
  for (unsigned i = 0; i < 3; ++i) {
    imu->accel[i] = signed_le16(accel + 2 * i) * (6.0f * 9.80665f / 32768.0f);
    imu->gyro[i] =
        signed_le16(gyro + 2 * i) * (2000.0f * 0.01745329252f / 32768.0f);
  }
  /* TEMP_MSB 在前：不是普通的16位小端数，只有11个有效位。 */
  int temperature = (temp[0] << 3) | (temp[1] >> 5);
  if (temperature > 1023)
    temperature -= 2048;
  imu->temperature_c = temperature * 0.125f + 23.0f;
  imu->samples++;
  imu->valid = true;
  return true;
}
