/* ICM-40608 6-axis IMU driver, written to order per the TDK InvenSense
   datasheet DS-000251 (Rev 0.4, 2018-08-14).

   The register family is ICM-426xx-compatible (DEVICE_CONFIG / PWR_MGMT0 /
   INTF_CONFIG0 / GYRO_CONFIG0 / ACCEL_CONFIG0 / REG_BANK_SEL bank switching),
   but the ICM-40608 differs from the ICM-42688 / ICM-42686 already in this
   tree in three important ways and therefore cannot share their drivers:

     - 16-bit ADCs only; there is NO high-resolution 20-byte FIFO. The combined
       FIFO packet (Packet 3) is 16 bytes: header + accel(6) + gyro(6) + temp(1)
       + timestamp(2), big-endian.
     - Full-scale tops out at +/-16g and +/-2000dps (no +/-32g, no +/-4000dps).
     - No external CLKIN support: INTF_CONFIG5 (bank 1) has no CLKIN bit, so
       the part runs on its internal PLL. (DS-000251: PIN9_FUNCTION 10b/11b
       and INTF_CONFIG1 bit2 are Reserved; full text has no "CLKIN".)

   Driver structure mirrors the existing Tlera Corporation / Kris Winer
   ICM-4268x drivers in this tree (MIT-friendly). No third-party code was
   copied; every register address, bit field, full-scale code and ODR code is
   taken directly from DS-000251.

   Identification: WHO_AM_I = 0x39 @ register 0x75; I2C slave address b110100X =
   0x68/0x69 (AP_AD0 pin selects the LSB).
*/
#ifndef ICM40608_h
#define ICM40608_h

#include "sensor/sensor.h"

// ICM-40608 Datasheet DS-000251 — User Bank 0 register map
#define ICM40608_DEVICE_CONFIG            0x11
#define ICM40608_DRIVE_CONFIG             0x13
#define ICM40608_INT_CONFIG               0x14
#define ICM40608_FIFO_CONFIG              0x16

#define ICM40608_TEMP_DATA1               0x1D
#define ICM40608_ACCEL_DATA_X1            0x1F
#define ICM40608_GYRO_DATA_X1             0x25

#define ICM40608_INT_STATUS               0x2D
#define ICM40608_FIFO_COUNTH              0x2E
#define ICM40608_FIFO_DATA                0x30

#define ICM40608_SIGNAL_PATH_RESET        0x4B
#define ICM40608_INTF_CONFIG0             0x4C
#define ICM40608_INTF_CONFIG1             0x4D
#define ICM40608_PWR_MGMT0                0x4E
#define ICM40608_GYRO_CONFIG0             0x4F
#define ICM40608_ACCEL_CONFIG0            0x50
#define ICM40608_GYRO_ACCEL_CONFIG0       0x52
#define ICM40608_TMST_CONFIG              0x54
#define ICM40608_SMD_CONFIG               0x57

#define ICM40608_FIFO_CONFIG1             0x5F
#define ICM40608_FIFO_CONFIG2             0x60
#define ICM40608_FIFO_CONFIG3             0x61

#define ICM40608_INT_CONFIG0              0x63
#define ICM40608_INT_CONFIG1              0x64
#define ICM40608_INT_SOURCE0              0x65
#define ICM40608_INT_SOURCE1              0x66

#define ICM40608_WHO_AM_I                 0x75
#define ICM40608_REG_BANK_SEL             0x76

// User Bank 1 (gyro AAF)
#define ICM40608_GYRO_CONFIG_STATIC3      0x0C   // bank1: GYRO_AAF_DELT[5:0]
#define ICM40608_GYRO_CONFIG_STATIC4      0x0D   // bank1: GYRO_AAF_DELTSQR[7:0]
#define ICM40608_GYRO_CONFIG_STATIC5      0x0E   // bank1: GYRO_AAF_BITSHIFT[7:4] | DELTSQR[11:8]

// User Bank 2 (accel AAF)
#define ICM40608_ACCEL_CONFIG_STATIC2     0x03   // bank2: ACCEL_AAF_DELT[6:1], bit0=ACCEL_AAF_DIS
#define ICM40608_ACCEL_CONFIG_STATIC3     0x04   // bank2: ACCEL_AAF_DELTSQR[7:0]
#define ICM40608_ACCEL_CONFIG_STATIC4     0x05   // bank2: ACCEL_AAF_BITSHIFT[7:4] | DELTSQR[11:8]

// User Bank 4
#define ICM40608_ACCEL_WOM_X_THR          0x4A
#define ICM40608_ACCEL_WOM_Y_THR          0x4B
#define ICM40608_ACCEL_WOM_Z_THR          0x4C

// FIFO_CONFIG1 enable bits (DS-000251 §14.44)
#define ICM40608_FIFO_ACCEL_EN      0x01
#define ICM40608_FIFO_GYRO_EN       0x02
#define ICM40608_FIFO_TEMP_EN       0x04
#define ICM40608_FIFO_TMST_FSYNC_EN 0x08

// Accelerometer full-scale select, ACCEL_CONFIG0[7:5] (DS-000251 §14.37).
// ⚠️ 编码与 ICM-40609 不同（40609 有 32G 档=000）；40608 从 16g 起：
// 000=±16g（复位默认）/ 001=±8g / 010=±4g / 011=±2g。
#define ICM40608_AFS_16G   0x00   // 2048 LSB/g
#define ICM40608_AFS_8G    0x01   // 4096 LSB/g
#define ICM40608_AFS_4G    0x02   // 8192 LSB/g
#define ICM40608_AFS_2G    0x03   // 16384 LSB/g

// Gyroscope full-scale select, GYRO_CONFIG0[7:5] (DS-000251 §14.36)
#define ICM40608_GFS_2000DPS    0x00   // 16.4 LSB/(deg/s)
#define ICM40608_GFS_1000DPS    0x01   // 32.8
#define ICM40608_GFS_500DPS     0x02   // 65.5
#define ICM40608_GFS_250DPS     0x03   // 131
#define ICM40608_GFS_125DPS     0x04   // 262
#define ICM40608_GFS_62_5DPS    0x05   // 524.3
#define ICM40608_GFS_31_25DPS   0x06   // 1048.6
#define ICM40608_GFS_15_625DPS  0x07   // 2097.2

// Accel/Gyro UI ODR select, CONFIG0[3:0] (DS-000251 §14.36/§14.37).
// Codes are shared by accel and gyro; only the Low-Noise rates are listed.
#define ICM40608_AODR_8kHz    0x03
#define ICM40608_AODR_4kHz    0x04
#define ICM40608_AODR_2kHz    0x05
#define ICM40608_AODR_1kHz    0x06
#define ICM40608_AODR_500Hz   0x0F
#define ICM40608_AODR_200Hz   0x07
#define ICM40608_AODR_100Hz   0x08
#define ICM40608_AODR_50Hz    0x09
#define ICM40608_AODR_25Hz    0x0A
#define ICM40608_AODR_12_5Hz  0x0B

// UI low-pass filter bandwidth, GYRO_ACCEL_CONFIG0 (0x52): ACCEL_UI_FILT_BW =
// bits[7:4], GYRO_UI_FILT_BW = bits[3:0] (DS-000251 §14.39). The code selects
// the -3dB bandwidth as a divisor of max(400Hz, ODR). Reset value 0x11 = 码1
// （max(400,ODR)/4）——不是最宽档；0x00 = ODR/2 才是最宽。
#define ICM40608_UI_FILT_BW_ODR_DIV_2   0x00   // widest
#define ICM40608_UI_FILT_BW_ODR_DIV_4   0x01   // reset default
#define ICM40608_UI_FILT_BW_ODR_DIV_5   0x02
#define ICM40608_UI_FILT_BW_ODR_DIV_8   0x03
#define ICM40608_UI_FILT_BW_ODR_DIV_10  0x04
#define ICM40608_UI_FILT_BW_ODR_DIV_16  0x05
#define ICM40608_UI_FILT_BW_ODR_DIV_20  0x06
#define ICM40608_UI_FILT_BW_ODR_DIV_40  0x07

// Selected bandwidth (range-INDEPENDENT drift/jitter fix; ±2000dps / ±16g kept).
// Tuned per on-board feedback: gyro ODR/16 (~31Hz@500) cut static yaw drift
// ~4x (89°/min -> ~22°/min); accel stays ODR/8 (~62Hz@500) for the gravity
// vector. 信号带（人体运动 <20Hz）远低于两者。
#define ICM40608_ACCEL_UI_FILT_BW  ICM40608_UI_FILT_BW_ODR_DIV_8
#define ICM40608_GYRO_UI_FILT_BW   ICM40608_UI_FILT_BW_ODR_DIV_16

// Gyro Anti-Alias Filter (AAF) — DS-000251 §5.3 / §15.2-15.5, register bank 1.
// The AAF sits at the gyro ADC output, BEFORE the 8kHz->ODR decimation: it is
// the programmable stage that rejects out-of-band vibration before that
// vibration aliases + rectifies (VRE) into the gyro passband as DC bias.
// Reset state: GYRO_AAF_DIS=0 (AAF ENABLED) but GYRO_AAF_DELT=63 => 3dB BW
// 995Hz (widest, ~no filtering). We narrow it below the gyro Nyquist
// (ODR/2 = 250Hz at the 500Hz gyro ODR).
// ⚠️ 40608 的 AAF 带宽表（§5.3：10~995Hz）与 ICM-4268x（42~3979Hz）不同——
// 同一 DELT 三元组在 4268x 上的带宽约为本芯片的 4 倍，严禁跨芯片照抄数值。
// R3/W3 on-board tuning: DELT=16 => 3dB BW ~184Hz。若振动漂移仍复现可再收：
// DELT 14->158Hz, 12->134Hz, 10->110Hz（改 DELTSQR/BITSHIFT 须随表联动）。
#define ICM40608_GYRO_AAF_DELT      16    // 3dB BW ~184Hz
#define ICM40608_GYRO_AAF_DELTSQR   256   // table row DELT=16 -> DELTSQR 256
#define ICM40608_GYRO_AAF_BITSHIFT  7     // table row DELT=16 -> BITSHIFT 7

// Accel Anti-Alias Filter — DS-000251 §5.3 / §16.1-16.4, register bank 2。
// 加速度计同样有 AAF（§5.3 表 accel/gyro 共用；复位 0x7E => DELT=63 => 995Hz
// 全开）。旧版驱动曾误信“accel 无 AAF”而从未配置——振动经 accel 路径混叠/
// 整流会污染 VQF 的重力向量。取 DELT=12 => 134Hz：accel 信号带 ≤30Hz 且重力
// 为直流，窄一档换更强的振动抑制（如需更宽可换 DELT=16 -> 184Hz）。
#define ICM40608_ACCEL_AAF_DELT      12    // 3dB BW ~134Hz
#define ICM40608_ACCEL_AAF_DELTSQR   144   // table row DELT=12 -> DELTSQR 144
#define ICM40608_ACCEL_AAF_BITSHIFT  8     // table row DELT=12 -> BITSHIFT 8

// Power modes, PWR_MGMT0 (DS-000251 §14.35)
#define ICM40608_aMode_OFF    0x00   // ACCEL_MODE[1:0]
#define ICM40608_aMode_LP     0x02
#define ICM40608_aMode_LN     0x03
#define ICM40608_gMode_OFF    0x00   // GYRO_MODE[3:2]
#define ICM40608_gMode_SBY    0x01
#define ICM40608_gMode_LN     0x03

int icm40608_init(float clock_rate, float accel_time, float gyro_time,
                  float *accel_actual_time, float *gyro_actual_time);
void icm40608_shutdown(void);

void icm40608_update_fs(float accel_range, float gyro_range,
                        float *accel_actual_range, float *gyro_actual_range);
int icm40608_update_odr(float accel_time, float gyro_time,
                        float *accel_actual_time, float *gyro_actual_time);

uint16_t icm40608_fifo_read(uint8_t *data, uint16_t len);
int icm40608_fifo_process(uint16_t index, uint8_t *data, float a[3], float g[3]);
void icm40608_accel_read(float a[3]);
void icm40608_gyro_read(float g[3]);
float icm40608_temp_read(void);

uint8_t icm40608_setup_DRDY(uint16_t threshold);
uint8_t icm40608_setup_WOM(void);

extern const sensor_imu_t sensor_imu_icm40608;

#endif
