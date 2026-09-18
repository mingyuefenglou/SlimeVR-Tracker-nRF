/* ICM-40608 6-axis IMU driver, written to order per TDK InvenSense datasheet
   DS-000251 (Rev 0.4, 2018-08-14). See ICM40608.h for why this is a separate
   driver from the ICM-42688/42686 in this tree (16-byte FIFO, +/-16g/+/-2000dps
   max, no CLKIN).

   Driver structure mirrors the existing Tlera Corporation / Kris Winer
   ICM-4268x drivers (MIT-friendly). All register values come from DS-000251;
   no third-party code was copied.
*/
#include <math.h>

#include <zephyr/logging/log.h>
#include <hal/nrf_gpio.h>

#include "ICM40608.h"
#include "sensor/sensor_none.h"

// FIFO Packet 3 = header(1) + accel(6) + gyro(6) + temp(1) + timestamp(2) = 16 bytes.
// The ICM-40608 has 16-bit ADCs and NO high-resolution FIFO (unlike ICM-42688/42686).
#define PACKET_SIZE 16

static const float accel_sensitivity = 16.0f / 32768.0f;   // Always +/-16g (2048 LSB/g)
static const float gyro_sensitivity = 2000.0f / 32768.0f;  // Always +/-2000dps (16.4 LSB/(deg/s))

// ICM-40608 UI ODRs (Low-Noise), high -> low. The trailing 0 is a sentinel that
// also clamps sub-12.5 Hz requests to the lowest supported rate. Codes per
// DS-000251 §14.36/§14.37 (CONFIG0[3:0]); shared by accel and gyro.
static const float odr_times[] = {
	8000, 4000, 2000, 1000, 500, 200, 100, 50, 25, 12.5f, 0
};
static const uint8_t odr_codes[] = {
	ICM40608_AODR_8kHz, ICM40608_AODR_4kHz, ICM40608_AODR_2kHz, ICM40608_AODR_1kHz,
	ICM40608_AODR_500Hz, ICM40608_AODR_200Hz, ICM40608_AODR_100Hz, ICM40608_AODR_50Hz,
	ICM40608_AODR_25Hz, ICM40608_AODR_12_5Hz
};

static uint8_t last_accel_odr = 0xff;
static uint8_t last_gyro_odr = 0xff;

#define FIFO_MULT 0.00075f     // assuming i2c fast mode
#define FIFO_MULT_SPI 0.0001f  // ~24MHz

static float fifo_multiplier_factor = FIFO_MULT;
static float fifo_multiplier = 0;

LOG_MODULE_REGISTER(ICM40608, LOG_LEVEL_DBG);

int icm40608_init(float clock_rate, float accel_time, float gyro_time,
				  float *accel_actual_time, float *gyro_actual_time)
{
	// The ICM-40608 has no external CLKIN input (INTF_CONFIG5 has no CLKIN bit),
	// so clock_rate is intentionally ignored and the part runs on its PLL.
	ARG_UNUSED(clock_rate);

	// setup interface for SPI
	if (!sensor_interface_spi_configure(SENSOR_INTERFACE_DEV_IMU, MHZ(24), 0))
		fifo_multiplier_factor = FIFO_MULT_SPI; // SPI mode
	else
		fifo_multiplier_factor = FIFO_MULT; // I2C mode

	int err = 0;

	// Optional WHO_AM_I read for debug.
#ifdef ICM40608_WHO_AM_I
	uint8_t whoami = 0;
	err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU, ICM40608_WHO_AM_I, &whoami);
	LOG_INF("ICM40608 WHO_AM_I = 0x%02X (expected 0x39)", whoami);
#endif

	// Report FIFO_COUNT and FIFO_WM in records (1 record = 16-byte A+G+temp+tmst packet).
	err |= ssi_reg_update_byte(SENSOR_INTERFACE_DEV_IMU,
							   ICM40608_INTF_CONFIG0,
							   0x40,
							   0x40);

	last_accel_odr = 0xff;
	last_gyro_odr = 0xff;

	// --- 以下三段滤波配置全部在「双传感器 OFF 窗口」写入：PWR_MGMT0 复位为 0
	// （两轴皆 OFF），且 shutdown() 为软复位、寄存器回到复位值，重初始化路径
	// 同样满足 DS §12.9（运行中仅 ODR/FSR/MODE 允许修改，其余须先关再改）。---

	// UI low-pass filter (GYRO_ACCEL_CONFIG0 / 0x52). Reset default (0x11 =
	// code 1) leaves max(400,ODR)/4 on both axes; we narrow gyro to ODR/16 and
	// accel to ODR/8 for the drift/jitter fix. Readback confirms it latched.
	// Byte = (ACCEL_UI_FILT_BW<<4) | GYRO_UI_FILT_BW = (0x03<<4)|0x05 = 0x35.
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU,
				  ICM40608_GYRO_ACCEL_CONFIG0,
				  (ICM40608_ACCEL_UI_FILT_BW << 4) | ICM40608_GYRO_UI_FILT_BW);

	uint8_t ui_filt_rb = 0;
	err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU,
				 ICM40608_GYRO_ACCEL_CONFIG0, &ui_filt_rb);
	LOG_INF("ICM40608 UI filter 0x52 readback = 0x%02X (wrote 0x35 => accel ODR/8, gyro ODR/16)",
		ui_filt_rb);

	// Gyro Anti-Alias Filter bandwidth (DS-000251 §5.3 / §15.2-15.5, bank 1).
	// Reset leaves the AAF ENABLED but at DELT=63 = 995Hz (widest, ~no
	// filtering); we narrow it below the gyro Nyquist. Notch stays
	// factory-trimmed; range/ODR untouched. Bank switch mirrors setup_WOM
	// (REG_BANK_SEL -> bank, write/readback, -> bank 0).
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU,
				  ICM40608_REG_BANK_SEL,
				  0x01); // select register bank 1

	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU,
				  ICM40608_GYRO_CONFIG_STATIC3,
				  ICM40608_GYRO_AAF_DELT & 0x3F);

	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU,
				  ICM40608_GYRO_CONFIG_STATIC4,
				  ICM40608_GYRO_AAF_DELTSQR & 0xFF);

	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU,
				  ICM40608_GYRO_CONFIG_STATIC5,
				  ((ICM40608_GYRO_AAF_BITSHIFT & 0x0F) << 4) |
				  ((ICM40608_GYRO_AAF_DELTSQR >> 8) & 0x0F));

	// 三个 AAF 寄存器全部回读（只读 DELT 不足以确认 DELTSQR/BITSHIFT 落位）。
	uint8_t gyro_aaf_rb[3] = {0};
	err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU,
				 ICM40608_GYRO_CONFIG_STATIC3, &gyro_aaf_rb[0]);
	err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU,
				 ICM40608_GYRO_CONFIG_STATIC4, &gyro_aaf_rb[1]);
	err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU,
				 ICM40608_GYRO_CONFIG_STATIC5, &gyro_aaf_rb[2]);

	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU,
				  ICM40608_REG_BANK_SEL,
				  0x00); // select register bank 0

	LOG_INF("ICM40608 gyro AAF readback = DELT 0x%02X DELTSQR 0x%02X%02X (wrote 16/0x100/7 => 3dB BW ~184Hz)",
		gyro_aaf_rb[0], gyro_aaf_rb[2] & 0x0F, gyro_aaf_rb[1]);

	// Accel Anti-Alias Filter (DS-000251 §5.3 / §16.1-16.4, bank 2)。复位 0x7E
	// => DELT=63 = 995Hz 全开；旧版驱动从未配置（曾误信“accel 无 AAF”）。取
	// DELT=12 => 134Hz，与 gyro 同窗口写入。
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU,
				  ICM40608_REG_BANK_SEL,
				  0x02); // select register bank 2

	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU,
				  ICM40608_ACCEL_CONFIG_STATIC2,
				  ((ICM40608_ACCEL_AAF_DELT & 0x3F) << 1)); // bit0 DIS=0（使能 AAF）

	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU,
				  ICM40608_ACCEL_CONFIG_STATIC3,
				  ICM40608_ACCEL_AAF_DELTSQR & 0xFF);

	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU,
				  ICM40608_ACCEL_CONFIG_STATIC4,
				  ((ICM40608_ACCEL_AAF_BITSHIFT & 0x0F) << 4) |
				  ((ICM40608_ACCEL_AAF_DELTSQR >> 8) & 0x0F));

	uint8_t accel_aaf_rb[3] = {0};
	err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU,
				 ICM40608_ACCEL_CONFIG_STATIC2, &accel_aaf_rb[0]);
	err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU,
				 ICM40608_ACCEL_CONFIG_STATIC3, &accel_aaf_rb[1]);
	err |= ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU,
				 ICM40608_ACCEL_CONFIG_STATIC4, &accel_aaf_rb[2]);

	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU,
				  ICM40608_REG_BANK_SEL,
				  0x00); // select register bank 0

	LOG_INF("ICM40608 accel AAF readback = STATIC2 0x%02X DELTSQR 0x%02X%02X (wrote DELT 12/0x090/8 => 3dB BW ~134Hz)",
		accel_aaf_rb[0], accel_aaf_rb[2] & 0x0F, accel_aaf_rb[1]);

	err |= icm40608_update_odr(accel_time, gyro_time,
							   accel_actual_time, gyro_actual_time);

	k_msleep(1);

	// FIFO Packet 3: accel + gyro + temperature + timestamp (16 bytes, big-endian).
	// FIFO_TMST_FSYNC_EN "must be set to 1 for all use cases" (DS-000251 §14.44).
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU,
							  ICM40608_FIFO_CONFIG1,
							  ICM40608_FIFO_ACCEL_EN | ICM40608_FIFO_GYRO_EN |
							  ICM40608_FIFO_TEMP_EN | ICM40608_FIFO_TMST_FSYNC_EN);

	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU,
							  ICM40608_FIFO_CONFIG,
							  1 << 6); // begin FIFO stream (Stream-to-FIFO mode)

	if (err)
		LOG_ERR("Communication error");

	return (err < 0 ? err : 0);
}

void icm40608_shutdown(void)
{
	last_accel_odr = 0xff;
	last_gyro_odr = 0xff;

	int err = ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU,
								 ICM40608_DEVICE_CONFIG,
								 0x01); // soft reset

	if (err)
		LOG_ERR("Communication error");
}

void icm40608_update_fs(float accel_range, float gyro_range,
						float *accel_actual_range, float *gyro_actual_range)
{
	ARG_UNUSED(accel_range);
	ARG_UNUSED(gyro_range);

	// ICM-40608 maximum ranges (no +/-32g or +/-4000dps available).
	*accel_actual_range = 16;    // always +/-16g
	*gyro_actual_range = 2000;   // always +/-2000dps
}

int icm40608_update_odr(float accel_time, float gyro_time,
						float *accel_actual_time, float *gyro_actual_time)
{
	int ODR;

	uint8_t Ascale = ICM40608_AFS_16G;     // +/-16g
	uint8_t Gscale = ICM40608_GFS_2000DPS; // +/-2000dps

	uint8_t aMode;
	uint8_t gMode;

	uint8_t AODR = 0;
	uint8_t GODR = 0;

	// Calculate accel
	if (accel_time <= 0 || accel_time == INFINITY)
	{
		aMode = ICM40608_aMode_OFF;
		accel_time = 0;
	}
	else
	{
		aMode = ICM40608_aMode_LN;
		ODR = 1 / accel_time;

		for (int i = 1; i < ARRAY_SIZE(odr_times); i++)
		{
			if (ODR <= odr_times[i])
				continue;

			AODR = odr_codes[i - 1];
			accel_time = 1.0f / odr_times[i - 1];
			break;
		}
	}

	// Calculate gyro
	if (gyro_time <= 0)
	{
		gMode = ICM40608_gMode_OFF;
		gyro_time = 0;
	}
	else if (gyro_time == INFINITY)
	{
		gMode = ICM40608_gMode_SBY;
		gyro_time = 0;
	}
	else
	{
		gMode = ICM40608_gMode_LN;
		ODR = 1 / gyro_time;

		for (int i = 1; i < ARRAY_SIZE(odr_times); i++)
		{
			if (ODR <= odr_times[i])
				continue;

			GODR = odr_codes[i - 1];
			gyro_time = 1.0f / odr_times[i - 1];
			break;
		}
	}

	if (last_accel_odr == AODR && last_gyro_odr == GODR)
		return 1;

	int err = 0;

	// only if the power mode has changed
	if (last_accel_odr == 0xff ||
		last_gyro_odr == 0xff ||
		(last_accel_odr == 0 ? 0 : 1) != (AODR == 0 ? 0 : 1) ||
		(last_gyro_odr == 0 ? 0 : 1) != (GODR == 0 ? 0 : 1))
	{
		err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU,
								  ICM40608_PWR_MGMT0,
								  gMode << 2 | aMode);

		k_busy_wait(250);
	}

	last_accel_odr = AODR;
	last_gyro_odr = GODR;

	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU,
							  ICM40608_ACCEL_CONFIG0,
							  Ascale << 5 | AODR);

	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU,
							  ICM40608_GYRO_CONFIG0,
							  Gscale << 5 | GODR);

	if (err)
		LOG_ERR("Communication error");

	*accel_actual_time = accel_time;
	*gyro_actual_time = gyro_time;

	// extra read packets by ODR time
	if (accel_time == 0 && gyro_time != 0)
		fifo_multiplier = fifo_multiplier_factor / gyro_time;
	else if (accel_time != 0 && gyro_time == 0)
		fifo_multiplier = fifo_multiplier_factor / accel_time;
	else if (gyro_time > accel_time)
		fifo_multiplier = fifo_multiplier_factor / accel_time;
	else if (accel_time > gyro_time)
		fifo_multiplier = fifo_multiplier_factor / gyro_time;
	else
		fifo_multiplier = 0;

	return 0;
}

uint16_t icm40608_fifo_read(uint8_t *data, uint16_t len)
{
	int err = 0;
	uint16_t total = 0;
	uint16_t packets = UINT16_MAX;

	while (packets > 0 && len >= PACKET_SIZE)
	{
		uint8_t rawCount[2];

		err |= ssi_burst_read(SENSOR_INTERFACE_DEV_IMU,
							  ICM40608_FIFO_COUNTH,
							  &rawCount[0],
							  2);

		packets = (uint16_t)(rawCount[0] << 8 | rawCount[1]);

		if (!packets)
			break;

		float extra_read_packets = packets * fifo_multiplier;
		packets += extra_read_packets;

		uint16_t count = packets * PACKET_SIZE;
		uint16_t limit = len / PACKET_SIZE;

		if (packets > limit)
		{
			LOG_WRN("FIFO read buffer limit reached, %d packets dropped",
					packets - limit);

			packets = limit;
			count = packets * PACKET_SIZE;
		}

		err |= ssi_burst_read_interval(SENSOR_INTERFACE_DEV_IMU,
									   ICM40608_FIFO_DATA,
									   data,
									   count,
									   PACKET_SIZE);

		if (err)
			LOG_ERR("Communication error");

		data += packets * PACKET_SIZE;
		len -= packets * PACKET_SIZE;
		total += packets;
	}

	return total;
}

// 0x8000 in every axis = the "no sample available" marker (DS-000251 §6.2).
static const uint8_t invalid[6] = {
	0x80, 0x00,
	0x80, 0x00,
	0x80, 0x00
};

int icm40608_fifo_process(uint16_t index, uint8_t *data, float a[3], float g[3])
{
	index *= PACKET_SIZE;

	if ((data[index] & 0x80) == 0x80)
		return 1; // HEADER_MSG: FIFO is empty for this slot

	if ((data[index] & 0x7F) == 0x7F)
		return 1; // Skip empty packets

	float a_raw[3] = {0};
	float g_raw[3] = {0};

	// Accel: bytes [1..6], big-endian X/Y/Z
	if (memcmp(&data[index + 1], invalid, sizeof(invalid)))
	{
		for (int i = 0; i < 3; i++)
		{
			a_raw[i] = (int16_t)((((uint16_t)data[index + 1 + (i * 2)]) << 8) |
								 data[index + 2 + (i * 2)]);
		}
	}

	// Gyro: bytes [7..12], big-endian X/Y/Z
	if (memcmp(&data[index + 7], invalid, sizeof(invalid)))
	{
		for (int i = 0; i < 3; i++)
		{
			g_raw[i] = (int16_t)((((uint16_t)data[index + 7 + (i * 2)]) << 8) |
								 data[index + 8 + (i * 2)]);
		}
	}
	else if (!memcmp(&data[index + 1], invalid, sizeof(invalid)))
	{
		return 1;
	}

	for (int i = 0; i < 3; i++)
	{
		a_raw[i] *= accel_sensitivity;
		g_raw[i] *= gyro_sensitivity;
	}

	memcpy(a, a_raw, sizeof(a_raw));
	memcpy(g, g_raw, sizeof(g_raw));

	return 0;
}

void icm40608_accel_read(float a[3])
{
	uint8_t rawAccel[6];

	int err = ssi_burst_read(SENSOR_INTERFACE_DEV_IMU,
							 ICM40608_ACCEL_DATA_X1,
							 &rawAccel[0],
							 6);

	if (err)
		LOG_ERR("Communication error");

	for (int i = 0; i < 3; i++)
	{
		a[i] = (int16_t)((((uint16_t)rawAccel[i * 2]) << 8) |
						 rawAccel[1 + (i * 2)]);

		a[i] *= accel_sensitivity;
	}
}

void icm40608_gyro_read(float g[3])
{
	uint8_t rawGyro[6];

	int err = ssi_burst_read(SENSOR_INTERFACE_DEV_IMU,
							 ICM40608_GYRO_DATA_X1,
							 &rawGyro[0],
							 6);

	if (err)
		LOG_ERR("Communication error");

	for (int i = 0; i < 3; i++)
	{
		g[i] = (int16_t)((((uint16_t)rawGyro[i * 2]) << 8) |
						 rawGyro[1 + (i * 2)]);

		g[i] *= gyro_sensitivity;
	}
}

float icm40608_temp_read(void)
{
	uint8_t rawTemp[2];

	int err = ssi_burst_read(SENSOR_INTERFACE_DEV_IMU,
							 ICM40608_TEMP_DATA1,
							 &rawTemp[0],
							 2);

	if (err)
		LOG_ERR("Communication error");

	// DS-000251 §14.5: degC = (TEMP_DATA / 132.48) + 25
	float temp = (int16_t)((((uint16_t)rawTemp[0]) << 8) | rawTemp[1]);

	temp /= 132.48f;
	temp += 25;

	return temp;
}

uint8_t icm40608_setup_DRDY(uint16_t threshold)
{
	uint8_t buf[2];

	buf[0] = threshold & 0xFF;
	buf[1] = (threshold >> 8) & 0x0F;

	int err = ssi_burst_write(SENSOR_INTERFACE_DEV_IMU,
							  ICM40608_FIFO_CONFIG2,
							  buf,
							  2);

	// Route the FIFO threshold interrupt to INT1 (INT_SOURCE0 bit2).
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU,
							  ICM40608_INT_SOURCE0,
							  0x04);

	if (err)
		LOG_ERR("Communication error");

	return NRF_GPIO_PIN_PULLUP << 4 | NRF_GPIO_PIN_SENSE_LOW;
}

uint8_t icm40608_setup_WOM(void)
{
	uint8_t interrupts;

	int err = ssi_reg_read_byte(SENSOR_INTERFACE_DEV_IMU,
								ICM40608_INT_STATUS,
								&interrupts);

	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU,
							  ICM40608_INT_SOURCE0,
							  0x00); // disable default interrupt

	// +/-8g（ACCEL_FS_SEL=001；40608 无 32g 档，编码与 4268x 的 8g 档同为 001），200 Hz
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU,
							  ICM40608_ACCEL_CONFIG0,
							  ICM40608_AFS_8G << 5 |
							  ICM40608_AODR_200Hz);

	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU,
							  ICM40608_PWR_MGMT0,
							  ICM40608_aMode_LP);

	// 注：这里【不写】INTF_CONFIG1。复位默认（0x91）已是正确配置——
	// ACCEL_LP_CLK_SEL=0（LP 模式用唤醒时钟，§14.34 默认即此）、CLKSEL=01
	// （有 PLL 用 PLL，否则 RC）。旧版曾整字节写 0x00：既踩了 bits7:4 保留位
	// （DS 要求保留位保持默认值），又把 CLKSEL 钉死为“永远 RC”——首次 WOM
	// 睡眠唤醒后芯片在 LN 模式下永久跑 RC 时钟（PLL 精度/抖动更优）。

	k_msleep(1);

	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU,
							  ICM40608_REG_BANK_SEL,
							  0x04); // select register bank 4

	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU,
							  ICM40608_ACCEL_WOM_X_THR,
							  0x08);

	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU,
							  ICM40608_ACCEL_WOM_Y_THR,
							  0x08);

	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU,
							  ICM40608_ACCEL_WOM_Z_THR,
							  0x08);

	k_msleep(1);

	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU,
							  ICM40608_REG_BANK_SEL,
							  0x00); // select register bank 0

	// Route WOM X/Y/Z interrupts to INT1 (INT_SOURCE1 bits[2:0]).
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU,
							  ICM40608_INT_SOURCE1,
							  0x07);

	k_msleep(50);

	// Enable the Wake-on-Motion feature (SMD_CONFIG SMD_MODE=01 => WOM mode).
	err |= ssi_reg_write_byte(SENSOR_INTERFACE_DEV_IMU,
							  ICM40608_SMD_CONFIG,
							  0x01);

	if (err)
		LOG_ERR("Communication error");

	return NRF_GPIO_PIN_PULLUP << 4 | NRF_GPIO_PIN_SENSE_LOW;
}

const sensor_imu_t sensor_imu_icm40608 = {
	icm40608_init,
	icm40608_shutdown,

	icm40608_update_fs,
	icm40608_update_odr,

	icm40608_fifo_read,
	icm40608_fifo_process,
	icm40608_accel_read,
	icm40608_gyro_read,
	icm40608_temp_read,

	icm40608_setup_DRDY,
	icm40608_setup_WOM,

	imu_none_ext_setup
};
