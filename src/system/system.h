#ifndef SLIMENRF_SYSTEM
#define SLIMENRF_SYSTEM

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "led.h"
#include "power.h"
#include "status.h"

#define RBT_CNT_ID 1
#define PAIRED_ID 2
#define MAIN_ACCEL_BIAS_ID 3
#define MAIN_GYRO_BIAS_ID 4
#define MAIN_MAG_BIAS_ID 5
#define MAIN_GYRO_SENS_ID 6
#define MAIN_ACC_6_BIAS_ID 7

#define BATT_STATS_LAST_RUN_ID 8
#define BATT_STATS_INTERVAL_0 9 // ID 9 to 28 (20 intervals)
#define BATT_STATS_CURVE_ID 29

#define MAIN_SENSOR_DATA_ID 30
#define RF_CHANNEL_ID 31
#define MAG_ENABLED_ID 37
#define TCAL_ENABLED_ID 38
#define MAG_ONLINE_CALIBRATION_ID 39

#define MAG_ONLINE_CALIBRATION_DEFAULT 0
#define MAG_ONLINE_CALIBRATION_ENABLED 1
#define MAG_ONLINE_CALIBRATION_DISABLED 2

#if CONFIG_SENSOR_USE_TCAL
#define MAIN_GYRO_TEMP_ID 32
#define MAIN_GYRO_TCAL_POINTS_ID 33
#define MAIN_GYRO_TCAL_COEFFS_ID 34
#define MAIN_GYRO_TCAL_STATE_ID  35
#define MAIN_GYRO_TCAL_CORRECTION_ID 36
#endif

void configure_sense_pins(void);

/* Complete RESETREAS snapshot captured once at PRE_KERNEL_1, before W1C. */
uint32_t sys_get_reset_reason(void);

uint8_t reboot_counter_read(void);
void reboot_counter_write(uint8_t reboot_counter);

/* Eager retained + NVS write, thread context only.
 * Returns 0 on persistence success, -EIO if NVS cannot initialize, or the
 * negative NVS write error. On failure retained RAM is still updated/sealed.
 */
int sys_write(uint16_t id, void *ptr, const void *data, size_t len);
void sys_write_warm(uint16_t id, void *retained_ptr, const void *data, size_t len);
void sys_warm_transaction_begin(void);
void sys_warm_transaction_mark(uint16_t id, void *retained_ptr, size_t len);
void sys_warm_transaction_end(bool retained_changed);
void sys_flush_warm(void);
bool sys_warm_is_dirty(void);
void sys_read(uint16_t id, void *data, size_t len);
/* Confirmation-gated reset; cancels deferred IMU writes before clearing storage.
 * Storage errors are logged; live runtime settings still require a reboot. */
void sys_clear(void);
void sys_nvs_stats(void);

int set_sensor_clock(bool enable, float rate, float* actual_rate);

// IMUCLK（pwmclock，32.768kHz）手动/自动门控（nini 注入，通用可上游）。
// 两部分机制：①手动 pwmclock on/off 覆盖自动逻辑（重启回 auto）；
// ②自动——探测到「需外部时钟」的 IMU（42688/42686/45686）才开启，
// 并由各驱动自带的 CLKIN 探活（10ms FIFO 计数）验证时钟真正生效。
enum sensor_clock_user_mode {
	SENSOR_CLOCK_AUTO = 0,
	SENSOR_CLOCK_FORCE_ON,
	SENSOR_CLOCK_FORCE_OFF,
};
int sys_sensor_clock_apply(bool auto_want, float *actual_rate);
void sys_sensor_clock_set_user_mode(enum sensor_clock_user_mode mode);
enum sensor_clock_user_mode sys_sensor_clock_get_user_mode(void);
bool sys_sensor_clock_is_applied(void);
float sys_sensor_clock_last_rate(void);

bool button_read(void);
bool button_read_filtered(void);

bool dock_read(void);
bool chg_read(void);
bool stby_read(void);

/* 0: power request accepted; positive: deliberate long-hold cancellation;
 * negative: admission rejected (not a pairing request). */
int sys_user_shutdown(void);
/* 0: asynchronous OFF request accepted; negative: admission rejected. */
int sys_command_shutdown(void);
void sys_enter_dfu(bool ota);
void sys_skip_dfu(void);
void sys_reset_mode(uint8_t mode);

#endif
