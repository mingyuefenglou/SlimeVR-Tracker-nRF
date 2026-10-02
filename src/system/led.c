#include "globals.h"

#include <math.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/pm/device.h>

#include "led.h"

#include "sensor/sensor.h"    // sensor_ms_since_motion（静止 3 分钟暗化）
#include "system/esb_ota.h"   // esb_ota_is_active（OTA 中豁免暗化）

LOG_MODULE_REGISTER(led, LOG_LEVEL_INF);

static void led_thread(void);
K_THREAD_DEFINE(led_thread_id, 512, led_thread, NULL, NULL, NULL, LED_THREAD_PRIORITY, 0, 0);

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, led_en_gpios)
#define LED_EN_EXISTS true
static const struct gpio_dt_spec led_en = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, led_en_gpios);
#endif

#if CONFIG_LED_STRIP
#define LED_STRIP_EXISTS true
#include <zephyr/drivers/led_strip.h>
#define STRIP_NODE DT_ALIAS(led_strip)
static const struct device *const strip = DEVICE_DT_GET(STRIP_NODE);
#endif

#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, led_gpios)
#define LED_EXISTS true
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, led_gpios);
#endif
#if DT_NODE_EXISTS(DT_ALIAS(led0))
#ifndef LED_EXISTS
#define LED_EXISTS true
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
#else
#define LED0_EXISTS true
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
#endif
#endif
#ifndef LED_EXISTS
#ifndef LED_STRIP_EXISTS
#warning "LED GPIO does not exist"
// static const struct gpio_dt_spec led = {0};
#endif
#endif
#if DT_NODE_EXISTS(DT_ALIAS(led1))
#define LED1_EXISTS true
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
#endif
#if DT_NODE_EXISTS(DT_ALIAS(led2))
#define LED2_EXISTS true
static const struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);
#endif
#if DT_NODE_EXISTS(DT_ALIAS(led3))
#define LED3_EXISTS true
static const struct gpio_dt_spec led3 = GPIO_DT_SPEC_GET(DT_ALIAS(led3), gpios);
#endif

#if DT_NODE_EXISTS(DT_ALIAS(pwm_led0))
#define PWM_LED_EXISTS true
static const struct pwm_dt_spec pwm_led = PWM_DT_SPEC_GET(DT_ALIAS(pwm_led0));
#else
#ifndef LED_STRIP_EXISTS
#warning "PWM LED node does not exist"
#endif
#endif
#if DT_NODE_EXISTS(DT_ALIAS(pwm_led1))
#define PWM_LED1_EXISTS true
static const struct pwm_dt_spec pwm_led1 = PWM_DT_SPEC_GET(DT_ALIAS(pwm_led1));
#endif
#if DT_NODE_EXISTS(DT_ALIAS(pwm_led2))
#define PWM_LED2_EXISTS true
static const struct pwm_dt_spec pwm_led2 = PWM_DT_SPEC_GET(DT_ALIAS(pwm_led2));
#endif

static enum sys_led_pattern current_led_pattern;
static int current_priority;

// 磁校准进度（0-10000；校准线程写、LED 线程读）——全局定义，
// 非三色板（传统路径）也要存在以供 cal_mag.c 链接（值不被消费，仅占位）
volatile uint16_t led_cal_progress;

#if LED_EXISTS || LED_STRIP_EXISTS
static enum sys_led_pattern led_patterns[SYS_LED_PATTERN_DEPTH]
	= {[0 ...(SYS_LED_PATTERN_DEPTH - 1)] = SYS_LED_PATTERN_OFF};
static int led_pattern_state;

static int led_pin_init(void)
{
	LOG_DBG("led_pin_init");
#if LED_EXISTS
	gpio_pin_configure_dt(&led, GPIO_OUTPUT);
	gpio_pin_set_dt(&led, 0);
#endif
#if LED0_EXISTS
	gpio_pin_configure_dt(&led0, GPIO_OUTPUT);
	gpio_pin_set_dt(&led0, 0);
#endif
#if LED1_EXISTS
	gpio_pin_configure_dt(&led1, GPIO_OUTPUT);
	gpio_pin_set_dt(&led1, 0);
#endif
#if LED2_EXISTS
	gpio_pin_configure_dt(&led2, GPIO_OUTPUT);
	gpio_pin_set_dt(&led2, 0);
#endif
#if LED3_EXISTS
	gpio_pin_configure_dt(&led3, GPIO_OUTPUT);
	gpio_pin_set_dt(&led3, 0);
#endif
	return 0;
}

SYS_INIT(led_pin_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

static void led_pin_reset(void)
{
	LOG_DBG("led_pin_reset");
#if LED_EXISTS
	gpio_pin_configure_dt(&led, GPIO_DISCONNECTED);
#endif
#if LED0_EXISTS
	gpio_pin_configure_dt(&led0, GPIO_DISCONNECTED);
#endif
#if LED1_EXISTS
	gpio_pin_configure_dt(&led1, GPIO_DISCONNECTED);
#endif
#if LED2_EXISTS
	gpio_pin_configure_dt(&led2, GPIO_DISCONNECTED);
#endif
#if LED3_EXISTS
	gpio_pin_configure_dt(&led3, GPIO_DISCONNECTED);
#endif
}

static void led_suspend(void)
{
	LOG_DBG("led_suspend");
#ifdef LED_STRIP_EXISTS
	pm_device_action_run(strip, PM_DEVICE_ACTION_SUSPEND);
#endif
#ifdef PWM_LED_EXISTS
	pm_device_action_run(pwm_led.dev, PM_DEVICE_ACTION_SUSPEND);
#endif
#ifdef PWM_LED1_EXISTS
	pm_device_action_run(pwm_led1.dev, PM_DEVICE_ACTION_SUSPEND);
#endif
#ifdef PWM_LED2_EXISTS
	pm_device_action_run(pwm_led2.dev, PM_DEVICE_ACTION_SUSPEND);
#endif
	led_pin_reset();
	// disable power
#if LED_EN_EXISTS
	gpio_pin_configure_dt(&led_en, GPIO_OUTPUT);
	gpio_pin_set_dt(&led_en, 0);
#endif
}

static void led_resume(void)
{
	LOG_DBG("led_resume");
	// enable power
#if LED_EN_EXISTS
	gpio_pin_configure_dt(&led_en, GPIO_OUTPUT);
	gpio_pin_set_dt(&led_en, 1);
#endif
#ifdef LED_STRIP_EXISTS
	pm_device_action_run(strip, PM_DEVICE_ACTION_RESUME);
#endif
#ifdef PWM_LED_EXISTS
	pm_device_action_run(pwm_led.dev, PM_DEVICE_ACTION_RESUME);
#endif
#ifdef PWM_LED1_EXISTS
	pm_device_action_run(pwm_led1.dev, PM_DEVICE_ACTION_RESUME);
#endif
#ifdef PWM_LED2_EXISTS
	pm_device_action_run(pwm_led2.dev, PM_DEVICE_ACTION_RESUME);
#endif
	led_pin_init();
}

#ifdef LED_STRIP_EXISTS
#define LED_RGB_COLOR
#else
#ifdef CONFIG_LED_RGB_COLOR
#define LED_RGB_COLOR
#define LED_RG_COLOR
#endif

#if PWM_LED_EXISTS && PWM_LED1_EXISTS && PWM_LED2_EXISTS
#define LED_TRI_COLOR
#else
#undef LED_RGB_COLOR
#undef LED_TRI_COLOR
#if PWM_LED_EXISTS && PWM_LED1_EXISTS
#define LED_DUAL_COLOR
#else
#undef LED_RG_COLOR
#undef LED_DUAL_COLOR
#endif
#endif
#endif

#ifdef LED_RGB_COLOR
static int led_pwm_period[5][3] = {
	{CONFIG_LED_DEFAULT_COLOR_R, CONFIG_LED_DEFAULT_COLOR_G, CONFIG_LED_DEFAULT_COLOR_B}, // Default
	{0, 10000, 0},                                                                        // Success
	{10000, 0, 0},                                                                        // Error
	{8000, 2000, 0},                                                                      // Charging
	{0, 0, 10000},                                                                        // Pairing
};
#elif defined(LED_TRI_COLOR)
static int led_pwm_period[5][3] = {
	{0, 0, 10000},   // Default
	{0, 10000, 0},   // Success
	{10000, 0, 0},   // Error
	{6000, 4000, 0}, // Charging
	{0, 0, 10000},   // Pairing
};
#elif defined(LED_RG_COLOR)
static int led_pwm_period[5][2] = {
	{CONFIG_LED_DEFAULT_COLOR_R, CONFIG_LED_DEFAULT_COLOR_G}, // Default
	{0, 10000},                                               // Success
	{10000, 0},                                               // Error
	{8000, 2000},                                             // Charging
	{4000, 6000},                                             // Pairing
};
#elif defined(LED_DUAL_COLOR)
static int led_pwm_period[5][2] = {
	{0, 10000},   // Default
	{0, 10000},   // Success
	{10000, 0},   // Error
	{6000, 4000}, // Charging
	{0, 10000},   // Pairing
};
#else
static int led_pwm_period[5][1] = {
	{10000}, // Default
	{10000}, // Success
	{10000}, // Error
	{10000}, // Charging
	{10000}, // Pairing
};
#endif

// Using brightness and value if PWM is supported, otherwise value is coerced to on/off
// TODO: use computed constants for high/low brightness and color values
static void led_pin_set(enum sys_led_color color, int brightness_pptt, int value_pptt)
{
	LOG_DBG("led_pin_set: color %d, brightness %d, value %d", color, brightness_pptt, value_pptt);
	if (brightness_pptt < 0) {
		brightness_pptt = 0;
	} else if (brightness_pptt > 10000) {
		brightness_pptt = 10000;
	}
	if (value_pptt < 0) {
		value_pptt = 0;
	} else if (value_pptt > 10000) {
		value_pptt = 10000;
	}
#if LED_STRIP_EXISTS
	static struct led_rgb pixel[1];
	value_pptt = value_pptt * brightness_pptt / 10000;
	pixel[0].r = 255 * (led_pwm_period[color][0] * value_pptt / 10000) / 10000;
	pixel[0].g = 255 * (led_pwm_period[color][1] * value_pptt / 10000) / 10000;
	pixel[0].b = 255 * (led_pwm_period[color][2] * value_pptt / 10000) / 10000;
	led_strip_update_rgb(strip, pixel, 1);
#elif PWM_LED_EXISTS
	value_pptt = value_pptt * brightness_pptt / 10000;
	// only supporting color if PWM is supported
	pwm_set_pulse_dt(&pwm_led, pwm_led.period / 10000 * (led_pwm_period[color][0] * value_pptt / 10000));
#if PWM_LED1_EXISTS
	pwm_set_pulse_dt(&pwm_led1, pwm_led1.period / 10000 * (led_pwm_period[color][1] * value_pptt / 10000));
#if PWM_LED2_EXISTS
	pwm_set_pulse_dt(&pwm_led2, pwm_led2.period / 10000 * (led_pwm_period[color][2] * value_pptt / 10000));
#endif
#endif
#else
	gpio_pin_set_dt(&led, value_pptt > 5000);
#endif
}
#endif

/* =====================================================================
 * 三通道并行渲染（LED_TRI_COLOR 板专用）
 * 红/绿/蓝各自独立仲裁+相位机：「工作绿呼吸」与「链路蓝心跳」可同时呈现。
 * 分配表（日常=呼吸族/调试=闪烁族）由 pattern_mask + led_compute 承载。
 * 命脉：非暗态分支的 led_resume() 不可省略（PWM 设备必须 ACTIVE 才有输出）。
 * ===================================================================== */
#if defined(LED_TRI_COLOR) && (LED_EXISTS || LED_STRIP_EXISTS)

enum led_ch { LED_CH_R = 0, LED_CH_G = 1, LED_CH_B = 2, LED_CH_COUNT };

#define CH_R (1u << LED_CH_R)
#define CH_G (1u << LED_CH_G)
#define CH_B (1u << LED_CH_B)
#define CH_ALL (CH_R | CH_G | CH_B)

struct led_channel {
	enum sys_led_pattern pattern; // 本通道当前渲染的 pattern（OFF 表示暗）
	uint32_t state;               // 相位状态（oneshot 步进计数）
	uint32_t last_value;          // 输出值缓存（去抖）
};

static struct led_channel chans[LED_CH_COUNT];
static enum led_display_mode led_mode = LED_MODE_DAILY;
static uint16_t led_brightness_pptt = 2500; // 全局亮度乘数=全域最大亮度（默认 25%，ledbright 0-100 可调；所有灯效受它缩放，此值即满刻度图案的峰顶占空）

/* LED 绑定表：物理位 LED1/2/3（=dts pwm-led0/1/2）上各是什么语义色（0=R 1=G 2=B）。
 * 默认恒等（pwm-led0=红/1=绿/2=蓝，与 dts 色序约定一致）。
 * ledmap 命令改写并持久化（retained CRC 外）——换灯/色序不同免重编。 */
static uint8_t led_phys_color[LED_CH_COUNT] = {LED_CH_R, LED_CH_G, LED_CH_B};

// 语义通道 → 物理位（led_channels_apply 路由用；表中必为排列，查找必命中）
static uint8_t bind_of_semantic(enum led_ch sem)
{
	for (uint8_t i = 0; i < LED_CH_COUNT; i++) {
		if (led_phys_color[i] == (uint8_t)sem) {
			return i;
		}
	}
	return (uint8_t)sem; // 防御：表损坏时回恒等
}

// 物理位 → PWM 设备（与 dts pwm-led0/1/2 一一对应）
static const struct pwm_dt_spec *const led_pwms[LED_CH_COUNT] = {&pwm_led, &pwm_led1, &pwm_led2};

/* 全暗睡眠信号量（丢唤醒竞态修复）：旧实现「外部线程 suspend→resume→wakeup +
 * led 线程自 suspend」存在唤醒丢失窗口——led 线程算出全暗准备挂起的瞬间被
 * 唤醒，恢复后继续走挂起路径 → LED 全黑卡死直到下次 set_led（"莫名好了"）。
 * 计数信号量无此窗口：give 在线程 take 之前也会计数，线程随后立即通过。 */
static K_SEM_DEFINE(led_wake_sem, 0, 1);

/* 通道语义定稿（第六轮废除琥珀混色——琥珀与单色肉眼难辨）：
 * 红=电力域（充电/低电），蓝=链路/升级域（心跳/OTA 快呼吸），绿=生命体征/确认。 */

// pattern → 通道掩码（分配表·通道语义；两模式相同，手法差异在 led_compute）
static uint32_t pattern_mask(enum sys_led_pattern p)
{
	switch (p) {
	case SYS_LED_PATTERN_OFF_FORCE:
	case SYS_LED_PATTERN_OFF:
	case SYS_LED_PATTERN_ONESHOT_POWEROFF:
	case SYS_LED_PATTERN_ONESHOT_COMPLETE: // 完成灯=全彩（对频/校准：三通道同帧闪烁）
	case SYS_LED_PATTERN_ERROR_A:
	case SYS_LED_PATTERN_ERROR_B:
	case SYS_LED_PATTERN_ERROR_C:
	case SYS_LED_PATTERN_ERROR_D:
		return CH_ALL; // 错误独占三灯 / 全局关
	case SYS_LED_PATTERN_ON:
	case SYS_LED_PATTERN_ONESHOT_PING:
	case SYS_LED_PATTERN_SHORT:
	case SYS_LED_PATTERN_CONNECT_HEARTBEAT:
		return CH_B; // 蓝=链路/即时反馈
	case SYS_LED_PATTERN_ONESHOT_POWERON:
		return CH_G; // 开机确认=绿（1.2s 常亮）
	case SYS_LED_PATTERN_LONG:
	case SYS_LED_PATTERN_FLASH:
	case SYS_LED_PATTERN_ONESHOT_PROGRESS:
	case SYS_LED_PATTERN_ON_PERSIST:
	case SYS_LED_PATTERN_ACTIVE_PERSIST:
		return CH_G; // 绿=生命体征/确认
	case SYS_LED_PATTERN_LONG_PERSIST:
	case SYS_LED_PATTERN_PULSE_PERSIST:
		return CH_R; // 红=电力域（充电/低电，纯色）
	case SYS_LED_PATTERN_DFU:
		return CH_B; // 蓝=升级域（OTA 纯色快呼吸）
	case SYS_LED_PATTERN_CAL_PROGRESS:
		return CH_R | CH_G; // 红→绿进度渐变
	case SYS_LED_PATTERN_FAST_GREEN:
	case SYS_LED_PATTERN_FAST_BLUE:
	case SYS_LED_PATTERN_FAST_RED:
		return CH_ALL; // 长按档位预告独占三灯（非目标通道渲染 0，压制一切残亮）
	case SYS_LED_PATTERN_ONESHOT_X2:
		return CH_G; // 2 连击确认=绿双闪
	case SYS_LED_PATTERN_ONESHOT_X3:
		return CH_B; // 3 连击确认=蓝三闪
	default:
		return CH_ALL;
	}
}

// 三角呼吸：phase 在 [0,period)，[0,up) 线性升至 peak，[up,up+down) 线性降回 0，其余 0
static uint32_t breath_shape(uint32_t phase, uint32_t period, uint32_t up, uint32_t down, uint32_t peak)
{
	if (phase < up) {
		return peak * phase / up;
	}
	if (phase < up + down) {
		return peak * (up + down - phase) / down;
	}
	return 0;
}

// oneshot 完成自清理（定义在 led_thread 旁；led_compute 内使用，前置声明）
static void led_oneshot_done(enum sys_led_pattern done);

// 计算某通道在 pattern 下的输出（0-10000 pptt）与建议刷新步距
static uint32_t led_compute(enum led_ch ch, enum sys_led_pattern p, uint32_t *state, uint32_t *step_ms)
{
	const uint32_t now = k_uptime_get_32();
	const bool daily = (led_mode == LED_MODE_DAILY);
	uint32_t v = 0;
	uint32_t st = 5; // 默认步距

	switch (p) {
	case SYS_LED_PATTERN_OFF_FORCE:
	case SYS_LED_PATTERN_OFF:
		v = 0;
		st = 100;
		break;

	case SYS_LED_PATTERN_ON: // 按住反馈（蓝在 1kΩ 下弱，80% 起步）
		v = daily ? 8000 : 10000;
		st = 200;
		break;

	case SYS_LED_PATTERN_ACTIVE_PERSIST: { // 绿·工作
		if (daily) {
			// 20s 墙钟栅格起伏 @0s：1.2s 升+1.2s 降，峰与链路心跳同（50%）——两灯等权交替
			v = breath_shape(now % 20000, 20000, 1200, 1200, 5000);
			st = 20;
		} else {
			v = (now % 10000) < 300 ? 10000 : 0; // 300ms blip/10s
			st = 50;
		}
		break;
	}

	case SYS_LED_PATTERN_CONNECT_HEARTBEAT: { // 蓝·链路心跳（20s 栅格 @10s 反相，与绿起伏永不错峰重叠）
		uint32_t phase = (now + 10000) % 20000; // 相位后移 10s → 与绿起伏互为反相
		if (daily) {
			v = breath_shape(phase, 20000, 1200, 1200, 5000);
			st = 20;
		} else {
			v = phase < 300 ? 10000 : 0;
			st = 50;
		}
		break;
	}

	case SYS_LED_PATTERN_SHORT: { // 蓝·未配对/搜台
		if (daily) {
			// 20s 栅格双击心跳：@10s 与 @13s 各一次 1.2s 起伏（中隔 0.6s 暗 =「定-定」双击感）
			uint32_t phase = now % 20000;
			if (phase >= 10000 && phase < 12400) {
				v = breath_shape(phase - 10000, 2400, 1200, 1200, 5000);
			} else if (phase >= 13000 && phase < 15400) {
				v = breath_shape(phase - 13000, 2400, 1200, 1200, 5000);
			}
			st = 20;
		} else {
			v = (now % 1000) < 100 ? 10000 : 0; // 100/900 快闪
			st = 50;
		}
		break;
	}

	case SYS_LED_PATTERN_LONG:
		v = (now % 1000) < 500 ? 10000 : 0;
		st = 50;
		break;
	case SYS_LED_PATTERN_FLASH:
		v = (now % 400) < 200 ? 10000 : 0;
		st = 40;
		break;

	case SYS_LED_PATTERN_ONESHOT_POWERON: { // 开机确认：日常=绿常亮 1.2s（顶格，绿在 1kΩ 下最弱）；调试=3 连闪
		// 按经过时间判定（旧实现按步数：步距被其他通道拉小时平顶会缩水，如 DFU 15ms → 0.9s）
		if (*state == 0) {
			*state = now + 1; // 存"起始+1"（0 保留作未开始标记；+1 使同 tick elapsed=0）
		}
		uint32_t elapsed = (now + 1) - *state;
		if (daily) {
			if (elapsed < 1200) {
				v = 10000;
				st = 20;
			} else {
				led_oneshot_done(p);
				v = 0;
				st = 100;
			}
		} else {
			v = ((elapsed / 200) % 2) ? 0 : 10000; // 200ms 方波 3 连闪
			if (elapsed >= 1400) {
				led_oneshot_done(p);
			}
			st = 200;
		}
		break;
	}

	case SYS_LED_PATTERN_ONESHOT_POWEROFF: { // 全彩渐灭（两模式同款）：250ms 全灭后 ~1s 线性渐灭
		uint32_t i = (*state)++;
		if (i == 0) {
			v = 0;
			st = 100; // 100ms 全黑（原 250ms 易被误读为"先灭了一下"）
		} else if (i <= 200) {
			v = (201 - i) * 50; // 10000 → 0，每 5ms 一步
			st = 5;
		} else {
			set_led(SYS_LED_PATTERN_OFF_FORCE, SYS_LED_PRIORITY_HIGHEST);
			v = 0;
			st = 100;
		}
		break;
	}

	case SYS_LED_PATTERN_ONESHOT_PROGRESS: { // 确认：日常=单次渐亮渐灭；调试=2 连闪
		uint32_t i = (*state)++;
		if (daily) {
			if (i <= 60) { // 1.2s：600 升 + 600 降
				v = (i <= 30) ? 7000 * i / 30 : 7000 * (60 - i) / 30;
				st = 20;
			} else {
				led_oneshot_done(p);
				st = 100;
			}
		} else {
			v = !(i % 2) * 10000;
			if (i == 5) {
				led_oneshot_done(p);
			}
			st = 200;
		}
		break;
	}

	case SYS_LED_PATTERN_ONESHOT_COMPLETE: { // 完成（对频/校准）：日常=全彩闪烁 1.2s；调试=全彩连闪
		if (daily) {
			// 全彩闪烁：150ms 亮/150ms 灭 ×4 = 1.2s。按经过时间判定（防步距被其他通道拉小缩水）
			if (*state == 0) {
				*state = now + 1; // 存"起始+1"（POWERON 同款，0=未开始标记）
			}
			uint32_t elapsed = (now + 1) - *state;
			if (elapsed < 1200) {
				v = ((elapsed / 150) % 2 == 0) ? 10000 : 0;
				st = 20;
			} else {
				led_oneshot_done(p);
				v = 0;
				st = 100;
			}
		} else {
			uint32_t i = (*state)++;
			v = !(i % 2) * 10000; // 值不引用 ch → CH_ALL 下三通道同帧全彩
			if (i >= 9) { // >=（非 ==）：防 daily→debug 穿越时 state 持旧时间戳永不命中
				led_oneshot_done(p);
			}
			st = 200;
		}
		break;
	}

	case SYS_LED_PATTERN_ONESHOT_PING: {
		uint32_t i = (*state)++;
		v = (i % 2) * 10000;
		if (i == 20) {
			led_oneshot_done(p);
		}
		st = 200;
		break;
	}

	case SYS_LED_PATTERN_FAST_GREEN: // 长按档位预告（独占三通道）：仅目标色 100/100 快闪
	case SYS_LED_PATTERN_FAST_BLUE:
	case SYS_LED_PATTERN_FAST_RED: {
		enum led_ch target = (p == SYS_LED_PATTERN_FAST_GREEN) ? LED_CH_G :
				     (p == SYS_LED_PATTERN_FAST_BLUE) ? LED_CH_B : LED_CH_R;
		if (ch == target) {
			v = (now % 200) < 100 ? 10000 : 0; // 墙钟相位：步距被其它通道压缩也不失真
			st = 20;
		} else {
			v = 0; // 非目标通道恒灭——呼吸/心跳/琥珀全被压制，"只有那一个灯在闪"
			st = 100;
		}
		break;
	}

	case SYS_LED_PATTERN_ONESHOT_X2: // 连击确认：双闪/三闪 150/150（按经过时间，防步距压缩缩水）
	case SYS_LED_PATTERN_ONESHOT_X3: {
		if (*state == 0) {
			*state = now + 1; // 存"起始+1"（POWERON 同款，0=未开始标记）
		}
		uint32_t elapsed = (now + 1) - *state;
		uint32_t total = (p == SYS_LED_PATTERN_ONESHOT_X2) ? 600 : 900; // 双闪 600ms / 三闪 900ms
		if (elapsed < total) {
			v = (elapsed % 300) < 150 ? 10000 : 0;
			st = 50;
		} else {
			led_oneshot_done(p);
			v = 0;
			st = 100;
		}
		break;
	}

	case SYS_LED_PATTERN_ON_PERSIST: // 充满：绿常亮（1kΩ 下绿弱，40% 起步）
		v = daily ? 4000 : 6000;
		st = 500;
		break;

	case SYS_LED_PATTERN_PULSE_PERSIST: { // 充电中：纯红低亮微呼吸直到充满
		if (daily) {
			uint32_t wob = breath_shape(now % 3000, 3000, 1500, 1500, 500);
			v = 1200 + wob / 2; // 12%±2.5% 微呼吸防死板（红 1kΩ 电流大，压亮度）
			st = 40;
		} else {
			v = 2000;
			st = 200;
		}
		break;
	}

	case SYS_LED_PATTERN_LONG_PERSIST: { // 低电：纯红虚弱呼吸 / 双闪
		const uint32_t peak = 800; // 红电气压到 8%（红峰值电流 ~1.1mA 是绿蓝 4-5 倍）
		if (daily) {
			v = breath_shape(now % 6000, 6000, 750, 750, peak);
			st = 20;
		} else {
			uint32_t phase = now % 1050;
			bool on = (phase < 150) || (phase >= 300 && phase < 450);
			v = on ? peak : 0;
			st = 30;
		}
		break;
	}

	case SYS_LED_PATTERN_DFU: { // OTA：纯蓝快呼吸（1.2s 周期，峰 70%）/ 快闪
		if (daily) {
			v = 7000 * breath_shape(now % 1200, 1200, 600, 600, 10000) / 10000;
			st = 15;
		} else {
			v = (now % 200) < 100 ? 7000 : 0;
			st = 50;
		}
		break;
	}

	case SYS_LED_PATTERN_CAL_PROGRESS: { // 磁校准：红→绿随进度插值（日常招牌）
		uint32_t prog = led_cal_progress;
		if (prog > 10000) {
			prog = 10000;
		}
		// 红电流效率是绿 4-5 倍——红分量减半防起步刺眼，绿满格保证进度可见
		uint32_t base = (ch == LED_CH_R) ? (10000 - prog) / 2 : prog;
		if (daily) {
			uint32_t wob = breath_shape(now % 2000, 2000, 1000, 1000, 1000); // ±10% 微呼吸
			v = base * (9000 + wob) / 10000;
			st = 20;
		} else {
			v = ((now % 1000) < 500) ? base : 0; // 离散硬闪，颜色即进度
			st = 50;
		}
		break;
	}

	case SYS_LED_PATTERN_ERROR_A:
	case SYS_LED_PATTERN_ERROR_B:
	case SYS_LED_PATTERN_ERROR_C:
	case SYS_LED_PATTERN_ERROR_D: { // 错误独占三灯
		if (daily) {
			// 每 5s 一次红深呼吸（1.2s 起伏，峰 50%）——克制但绝不错过
			if (ch == LED_CH_R) {
				v = breath_shape(now % 5000, 5000, 600, 600, 4000);
			}
			st = 20;
		} else {
			// 三色轮播：红→绿→蓝各 500ms 硬切（通道即颜色）
			uint32_t seg = (now / 500) % 3;
			v = (seg == (uint32_t)ch) ? 10000 : 0;
			st = 50;
		}
		break;
	}

	default:
		v = 0;
		st = 100;
		break;
	}

	*step_ms = st;
	return v;
}

// 某通道的当前归属 pattern（自槽 0 向下扫；OFF 让位；掩码命中才 claim）
static enum sys_led_pattern resolve_channel(enum led_ch ch)
{
	for (int prio = 0; prio < SYS_LED_PATTERN_DEPTH; prio++) {
		enum sys_led_pattern p = led_patterns[prio];
		if (p == SYS_LED_PATTERN_OFF) {
			continue; // yield
		}
		if (pattern_mask(p) & (1u << ch)) {
			return p;
		}
	}
	return SYS_LED_PATTERN_OFF;
}

// 渐灭需要把 value 转 PWM 占空（含全局亮度乘数 + 静止暗化）；按绑定表路由到物理位
static void led_channels_apply(void)
{
	/* 静止 3 分钟减半（防佩戴者睡着后刺眼）：活动即瞬时恢复（无迟滞）；
	 * OTA 中豁免——升级进度要看清楚。充电/错误/手势预告一并适用（用户："所有LED"）。 */
	uint32_t br = led_brightness_pptt;
	if (sensor_ms_since_motion() > 180000 && !esb_ota_is_active()) {
		br = br / 2;
	}
	for (int sem = 0; sem < LED_CH_COUNT; sem++) {
		uint32_t v = chans[sem].last_value * br / 10000;
		if (v > 10000) {
			v = 10000;
		}
		uint8_t phys = bind_of_semantic((enum led_ch)sem);
		if (phys >= LED_CH_COUNT) {
			phys = sem; // 防御：绑定表损坏时回恒等
		}
		const struct pwm_dt_spec *spec = led_pwms[phys];
		pwm_set_pulse_dt(spec, spec->period / 10000 * v);
	}
}

static bool led_any_on;

static void led_thread(void)
{
	// 开机从 retained 恢复显示偏好（0xFF=未初始化 → 默认日常/80%/恒等绑定）
	if (retained->led_mode == (uint8_t)LED_MODE_DEBUG) {
		led_mode = LED_MODE_DEBUG;
	}
	if (retained->led_bright <= 100) { // 0-100 皆合法（0=全灭）；0xFF 未初始化不落此分支
		led_brightness_pptt = retained->led_bright * 100;
	}
	if (retained->led_bind[0] <= LED_CH_B && retained->led_bind[1] <= LED_CH_B &&
	    retained->led_bind[2] <= LED_CH_B) {
		memcpy(led_phys_color, retained->led_bind, LED_CH_COUNT);
	}
	for (int i = 0; i < LED_CH_COUNT; i++) { // 防呆：PWM 设备就绪性显式上报（硬件层问题一眼定位）
		if (!device_is_ready(led_pwms[i]->dev)) {
			LOG_ERR("LED pwm device %d not ready", i);
		}
	}
	led_resume(); // 进线程即确保 PWM 设备 ACTIVE（命脉）
	while (1) {
		uint32_t min_step = 100;
		for (int ch = 0; ch < LED_CH_COUNT; ch++) {
			enum sys_led_pattern p = resolve_channel(ch);
			if (p != chans[ch].pattern) {
				chans[ch].pattern = p;
				chans[ch].state = 0; // 相位重置
			}
			uint32_t step = 100;
			uint32_t v = led_compute(ch, p, &chans[ch].state, &step);
			chans[ch].last_value = v;
			if (step < min_step) {
				min_step = step;
			}
		}
		led_channels_apply();

		const bool on = chans[LED_CH_R].pattern > SYS_LED_PATTERN_OFF ||
				chans[LED_CH_G].pattern > SYS_LED_PATTERN_OFF ||
				chans[LED_CH_B].pattern > SYS_LED_PATTERN_OFF;
		led_any_on = on;
		if (!on) {
			led_suspend();
			k_sem_take(&led_wake_sem, K_FOREVER); // 全暗睡眠：计数信号量，无丢唤醒
			led_resume(); // 命脉：唤醒后 PWM 必须回 ACTIVE
			continue;     // 醒来重新评估（含伪唤醒——give 先于 take 的计数场景）
		}
		k_msleep(min_step);
	}
}

// oneshot 完成自清理（只清承载该 pattern 的槽——旧实现 HIGHEST 全清会误伤
// 其他在播 oneshot，如 PING 与校准/配对反馈并发时后者被一起抹掉）
static void led_oneshot_done(enum sys_led_pattern done)
{
	for (int prio = 0; prio < SYS_LED_PATTERN_DEPTH; prio++) {
		if (led_patterns[prio] == done) {
			led_patterns[prio] = SYS_LED_PATTERN_OFF;
		}
	}
}

void set_led(enum sys_led_pattern led_pattern, int priority)
{
	if (k_current_get() == led_thread_id && led_pattern == SYS_LED_PATTERN_OFF_FORCE) {
		// OFF_FORCE 语义：压制一切（清所有槽并写进槽 0）
		for (int prio = 0; prio < SYS_LED_PATTERN_DEPTH; prio++) {
			led_patterns[prio] = SYS_LED_PATTERN_OFF;
		}
		led_patterns[SYS_LED_PRIORITY_HIGHEST] = led_pattern;
	} else {
		led_patterns[priority] = led_pattern;
	}

	if (k_current_get() != led_thread_id) {
		k_sem_give(&led_wake_sem);   // 唤醒全暗睡眠（计数信号量无丢唤醒；伪唤醒无害）
		k_wakeup(led_thread_id);     // 打断渲染步距睡眠，尽快响应新 pattern
	}
}

void set_led_mode(enum led_display_mode mode)
{
	led_mode = mode;
	retained->led_mode = (uint8_t)mode;
	retained_update();
}

enum led_display_mode get_led_mode(void)
{
	return led_mode;
}

void set_led_brightness(uint8_t percent)
{
	if (percent > 100) {
		percent = 100;
	}
	led_brightness_pptt = percent * 100; // 0-100（0=全灭可用）
	retained->led_bright = percent;
	retained_update();
}

uint8_t get_led_brightness(void)
{
	return led_brightness_pptt / 100;
}

/* LED 绑定：phys_colors[3] = 物理位 LED1/2/3 上各是什么色（0=R 1=G 2=B），
 * 必须是 R/G/B 的排列（调用方校验，此处防御再查一次） */
bool set_led_binding(const uint8_t phys_colors[LED_CH_COUNT])
{
	bool seen[LED_CH_COUNT] = {false, false, false};
	for (int i = 0; i < LED_CH_COUNT; i++) {
		if (phys_colors[i] >= LED_CH_COUNT || seen[phys_colors[i]]) {
			return false; // 越界或重复（如三个位全指蓝）→ 拒绝
		}
		seen[phys_colors[i]] = true;
	}
	memcpy(led_phys_color, phys_colors, LED_CH_COUNT);
	memcpy(retained->led_bind, phys_colors, LED_CH_COUNT);
	retained_update();
	return true;
}

void get_led_binding(uint8_t phys_colors[LED_CH_COUNT])
{
	memcpy(phys_colors, led_phys_color, LED_CH_COUNT);
}

void reset_led_binding(void)
{
	static const uint8_t identity[LED_CH_COUNT] = {LED_CH_R, LED_CH_G, LED_CH_B};
	set_led_binding(identity);
}

/* ==== 传统单仲裁路径（非三色板：行为与上游一致） ==== */
#elif LED_EXISTS || LED_STRIP_EXISTS /* 传统单仲裁路径（非三色板：行为与上游一致） */

// 双模式/全局亮度仅三通道板实现；传统板提供桩（console 可用、值不生效）
static enum led_display_mode led_mode_legacy = LED_MODE_DAILY;
static uint16_t led_brightness_pptt_legacy = 10000;

void set_led_mode(enum led_display_mode mode)
{
	led_mode_legacy = mode;
}

enum led_display_mode get_led_mode(void)
{
	return led_mode_legacy;
}

void set_led_brightness(uint8_t percent)
{
	if (percent < 5) {
		percent = 5;
	}
	if (percent > 100) {
		percent = 100;
	}
	led_brightness_pptt_legacy = percent * 100;
}

uint8_t get_led_brightness(void)
{
	return led_brightness_pptt_legacy / 100;
}

/* LED 绑定仅三通道板实现；传统板提供桩（console 可链接、查询回 R/G/B、指派拒绝不生效） */
bool set_led_binding(const uint8_t phys_colors[3])
{
	ARG_UNUSED(phys_colors);
	return false; // 非三通道板无绑定可改
}

void get_led_binding(uint8_t phys_colors[3])
{
	static const uint8_t identity[3] = {0, 1, 2}; // R G B
	memcpy(phys_colors, identity, 3);
}

void reset_led_binding(void)
{
}

void set_led(enum sys_led_pattern led_pattern, int priority)
{
	LOG_DBG("set_led: current_led_pattern %d, current_priority %d", current_led_pattern, current_priority);
	LOG_DBG("set_led: pattern %d, priority %d", led_pattern, priority);
	if (led_pattern <= SYS_LED_PATTERN_OFF && k_current_get() == led_thread_id) {
		led_patterns[current_priority] = led_pattern;
	} else {
		led_patterns[priority] = led_pattern;
	}
	for (priority = 0; priority < SYS_LED_PATTERN_DEPTH; priority++) {
		if (led_patterns[priority] == SYS_LED_PATTERN_OFF) {
			continue;
		}
		led_pattern = led_patterns[priority];
		break;
	}
	if (led_pattern == current_led_pattern && led_pattern > SYS_LED_PATTERN_OFF) {
		return;
	}
	current_led_pattern = led_pattern;
	current_priority = priority;
	led_pattern_state = 0;
	if (current_led_pattern <= SYS_LED_PATTERN_OFF) {
		led_suspend();
		k_thread_suspend(led_thread_id);
		LOG_DBG("set_led: suspended led_thread_id");
	} else if (k_current_get() != led_thread_id) // do not suspend if called from thread
	{
		k_thread_suspend(led_thread_id);
		LOG_DBG("set_led: suspended led_thread_id");
		led_resume();
		k_thread_resume(led_thread_id);
		k_wakeup(led_thread_id);
		LOG_DBG("set_led: resumed led_thread_id");
	} else {
		led_resume();
		k_thread_resume(led_thread_id);
		k_wakeup(led_thread_id);
		LOG_DBG("set_led: resumed led_thread_id");
	}
}

static void led_thread(void)
{
	while (1) {
		LOG_DBG("led_thread: current_led_pattern %d", current_led_pattern);
		switch (current_led_pattern) {
		case SYS_LED_PATTERN_ON:
			led_pin_set(SYS_LED_COLOR_DEFAULT, 10000, 10000);
			k_thread_suspend(led_thread_id);
			break;
		case SYS_LED_PATTERN_SHORT:
			led_pattern_state = (led_pattern_state + 1) % 2;
			led_pin_set(SYS_LED_COLOR_PAIRING, 10000, led_pattern_state * 10000);
			k_msleep(led_pattern_state == 1 ? 100 : 900);
			break;
		case SYS_LED_PATTERN_LONG:
			led_pattern_state = (led_pattern_state + 1) % 2;
			led_pin_set(SYS_LED_COLOR_DEFAULT, 10000, led_pattern_state * 10000);
			k_msleep(500);
			break;
		case SYS_LED_PATTERN_FLASH:
			led_pattern_state = (led_pattern_state + 1) % 2;
			led_pin_set(SYS_LED_COLOR_DEFAULT, 10000, led_pattern_state * 10000);
			k_msleep(200);
			break;

		case SYS_LED_PATTERN_ONESHOT_POWERON:
			led_pattern_state++;
			led_pin_set(SYS_LED_COLOR_DEFAULT, 10000, !(led_pattern_state % 2) * 10000);
			if (led_pattern_state == 7) {
				set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_HIGHEST);
			} else {
				k_msleep(200);
			}
			break;
		case SYS_LED_PATTERN_ONESHOT_POWEROFF:
			if (led_pattern_state++ > 0) {
				led_pin_set(
					SYS_LED_COLOR_DEFAULT,
					(202 - led_pattern_state) * 50,
					(led_pattern_state != 202 ? 10000 : 0)
				);
			} else {
				led_pin_set(SYS_LED_COLOR_DEFAULT, 10000, 0);
			}
			if (led_pattern_state == 202) {
				set_led(SYS_LED_PATTERN_OFF_FORCE, SYS_LED_PRIORITY_HIGHEST);
			} else if (led_pattern_state == 1) {
				k_msleep(250);
			} else {
				k_msleep(5);
			}
			break;
		case SYS_LED_PATTERN_ONESHOT_PROGRESS:
			led_pattern_state++;
			led_pin_set(SYS_LED_COLOR_SUCCESS, 10000, !(led_pattern_state % 2) * 10000);
			if (led_pattern_state == 5) {
				set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_HIGHEST);
			} else {
				k_msleep(200);
			}
			break;
		case SYS_LED_PATTERN_ONESHOT_COMPLETE:
			led_pattern_state++;
			led_pin_set(SYS_LED_COLOR_SUCCESS, 10000, !(led_pattern_state % 2) * 10000);
			if (led_pattern_state == 9) {
				set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_HIGHEST);
			} else {
				k_msleep(200);
			}
			break;
		case SYS_LED_PATTERN_ONESHOT_PING:
			led_pattern_state++;
			led_pin_set(SYS_LED_COLOR_DEFAULT, 10000, (led_pattern_state % 2) * 10000);
			if (led_pattern_state == 20) { // 10 flashes (states 1-20), turn off at 20
				set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_HIGHEST);
			} else {
				k_msleep(200);
			}
			break;

		case SYS_LED_PATTERN_FAST_GREEN: // 单灯板兜底：绿/蓝→默认色、红→错误色，均 100/100 快闪
		case SYS_LED_PATTERN_FAST_BLUE:
			led_pattern_state = (led_pattern_state + 1) % 2;
			led_pin_set(SYS_LED_COLOR_DEFAULT, 10000, led_pattern_state * 10000);
			k_msleep(100);
			break;
		case SYS_LED_PATTERN_FAST_RED:
			led_pattern_state = (led_pattern_state + 1) % 2;
			led_pin_set(SYS_LED_COLOR_ERROR, 10000, led_pattern_state * 10000);
			k_msleep(100);
			break;

		case SYS_LED_PATTERN_ONESHOT_X2: // 单灯板兜底：双闪/三闪后交还
		case SYS_LED_PATTERN_ONESHOT_X3:
			led_pattern_state++;
			led_pin_set(SYS_LED_COLOR_DEFAULT, 10000, !(led_pattern_state % 2) * 10000);
			if (led_pattern_state ==
			    (current_led_pattern == SYS_LED_PATTERN_ONESHOT_X2 ? 5 : 7)) { // 2/3 次亮-灭循环后清
				set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_HIGHEST);
			} else {
				k_msleep(150);
			}
			break;

		case SYS_LED_PATTERN_ON_PERSIST:
			led_pin_set(SYS_LED_COLOR_SUCCESS, 2000, 10000);
			k_thread_suspend(led_thread_id);
			break;
		case SYS_LED_PATTERN_LONG_PERSIST:
			led_pattern_state = (led_pattern_state + 1) % 2;
			led_pin_set(SYS_LED_COLOR_CHARGING, 2000, led_pattern_state * 10000);
			k_msleep(500);
			break;
		case SYS_LED_PATTERN_PULSE_PERSIST:
			led_pattern_state = (led_pattern_state + 1) % 1000;
			//			float led_value = sinf(led_pattern_state * (M_PI / 1000));
			//			led_pin_set(SYS_LED_COLOR_CHARGING, 10000, led_value * 10000);
			int led_value = led_pattern_state > 500 ? 1000 - led_pattern_state : led_pattern_state;
			if (led_value < 200) {
				led_value = (led_value) * 30;
			} else if (led_value < 300) {
				led_value = (led_value - 200) * 20 + 6000;
			} else if (led_value < 400) {
				led_value = (led_value - 300) * 15 + 8000;
			} else {
				led_value = (led_value - 400) * 5 + 9500;
			}
			led_pin_set(SYS_LED_COLOR_CHARGING, 10000, led_value);
			k_msleep(5);
			break;
		case SYS_LED_PATTERN_ACTIVE_PERSIST: // off duration first because the device may turn on multiple times rapidly
											 // and waste battery power
			led_pattern_state = (led_pattern_state + 1) % 2;
			led_pin_set(SYS_LED_COLOR_DEFAULT, 10000, !led_pattern_state * 10000);
			k_msleep(led_pattern_state ? 9700 : 300);
			break;

		case SYS_LED_PATTERN_ERROR_A: // TODO: should this use 20% duty cycle?
			led_pattern_state = (led_pattern_state + 1) % 10;
			led_pin_set(SYS_LED_COLOR_ERROR, 10000, (led_pattern_state < 4 && led_pattern_state % 2) * 10000);
			k_msleep(500);
			break;
		case SYS_LED_PATTERN_ERROR_B:
			led_pattern_state = (led_pattern_state + 1) % 10;
			led_pin_set(SYS_LED_COLOR_ERROR, 10000, (led_pattern_state < 6 && led_pattern_state % 2) * 10000);
			k_msleep(500);
			break;
		case SYS_LED_PATTERN_ERROR_C:
			led_pattern_state = (led_pattern_state + 1) % 10;
			led_pin_set(SYS_LED_COLOR_ERROR, 10000, (led_pattern_state < 8 && led_pattern_state % 2) * 10000);
			k_msleep(500);
			break;
		case SYS_LED_PATTERN_ERROR_D:
			led_pattern_state = (led_pattern_state + 1) % 2;
			led_pin_set(SYS_LED_COLOR_ERROR, 10000, led_pattern_state * 10000);
			k_msleep(500);
			break;

		case SYS_LED_PATTERN_DFU:
			/* Fast yellow pulse: 100 ms on, 100 ms off. */
			led_pattern_state = (led_pattern_state + 1) % 2;
			led_pin_set(SYS_LED_COLOR_CHARGING, 10000, led_pattern_state * 10000);
			k_msleep(100);
			break;

		default:
			LOG_DBG("led_thread: suspending led_thread_id");
			k_thread_suspend(led_thread_id);
		}
	}
}

#else
static void led_thread(void)
{
	LOG_WRN("LED GPIO does not exist");
}
#endif /* LED_TRI_COLOR */
