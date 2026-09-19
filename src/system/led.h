#ifndef SLIMENRF_SYSTEM_LED
#define SLIMENRF_SYSTEM_LED

/*
LED priorities (0 is highest)
0: boot/power
1: sensor
2: connection (esb)
3: status
4: system (persist)
*/

#define SYS_LED_PRIORITY_HIGHEST 0
#define SYS_LED_PRIORITY_BOOT 0
#define SYS_LED_PRIORITY_SENSOR 1
#define SYS_LED_PRIORITY_CONNECTION 2
#define SYS_LED_PRIORITY_STATUS 3
#define SYS_LED_PRIORITY_SYSTEM 4
#define SYS_LED_PATTERN_DEPTH 5

// RGB
// Red, Green, Blue

// Tri-color
// Red/Amber, Green, YellowGreen/White

// RG
// Red, Green

// Dual color
// Red/Amber, YellowGreen/White

// 三通道渲染（仅 LED_TRI_COLOR 板生效；其余板走传统单仲裁路径，行为不变）：
// 红/绿/蓝各自独立仲裁与相位机——「工作绿呼吸」与「链路蓝心跳」可同时呈现。
enum sys_led_pattern {
	SYS_LED_PATTERN_OFF_FORCE, // ignores lower priority patterns

	SYS_LED_PATTERN_OFF,   // yield to lower priority patterns
	SYS_LED_PATTERN_ON,    // Default | indicates busy
	SYS_LED_PATTERN_SHORT, // 100ms on 900ms off									// Pairing | indicates waiting
						   // (pairing)
	SYS_LED_PATTERN_LONG,  // 500ms on 500ms off										// Default | indicates waiting
	SYS_LED_PATTERN_FLASH, // 200ms on 200ms off									// Default | indicates readiness

	SYS_LED_PATTERN_ONESHOT_POWERON,  // 200ms on 200ms off, 3 times					// Default
	SYS_LED_PATTERN_ONESHOT_POWEROFF, // 250ms off, 1000ms fade to off				// Default
	SYS_LED_PATTERN_ONESHOT_PROGRESS, // 200ms on 200ms off, 2 times				// Success
	SYS_LED_PATTERN_ONESHOT_COMPLETE, // 200ms on 200ms off, 4 times				// Success
	SYS_LED_PATTERN_ONESHOT_PING,     // 200ms on 200ms off, 10 times				// Ping

	SYS_LED_PATTERN_ON_PERSIST,     // 20% duty cycle									// Success | indicates charged
	SYS_LED_PATTERN_LONG_PERSIST,   // 20% duty cycle, 500ms on 500ms off				// Charging| indicates low battery
	SYS_LED_PATTERN_PULSE_PERSIST,  // 5000ms pulsing								// Charging| indicates charging
	SYS_LED_PATTERN_ACTIVE_PERSIST, // 300ms on 9700ms off							// Default | indicates normal
									 // operation

	SYS_LED_PATTERN_ERROR_A, // 500ms on 500ms off, 2 times, every 5000ms			// Error
	SYS_LED_PATTERN_ERROR_B, // 500ms on 500ms off, 3 times, every 5000ms			// Error
	SYS_LED_PATTERN_ERROR_C, // 500ms on 500ms off, 4 times, every 5000ms			// Error
	SYS_LED_PATTERN_ERROR_D, // 500ms on 500ms off (same as SYS_LED_PATTERN_LONG)	// Error
	SYS_LED_PATTERN_DFU,       // Fast yellow pulse								// DFU/OTA update mode

	// ---- 三通道新增 pattern（旧调用点零改动，新语义由分配表承载）----
	SYS_LED_PATTERN_CONNECT_HEARTBEAT, // 蓝·链路心跳（配对成功后由 esb 设回）
	SYS_LED_PATTERN_CAL_PROGRESS,      // 红→绿随进度插值（进度经 led_cal_progress 全局）

	SYS_LED_PATTERN__COUNT,
};

enum sys_led_color {
	SYS_LED_COLOR_DEFAULT,
	SYS_LED_COLOR_SUCCESS,
	SYS_LED_COLOR_ERROR,
	SYS_LED_COLOR_CHARGING,
	SYS_LED_COLOR_PAIRING,
};

// LED 分配表（三通道板）：日常=呼吸族（美观），调试=闪烁族（明确）
enum led_display_mode {
	LED_MODE_DAILY = 0,
	LED_MODE_DEBUG = 1,
};

void set_led(enum sys_led_pattern led_pattern, int priority);

// 模式与全局亮度（三通道板；持久化于 retained）
void set_led_mode(enum led_display_mode mode);
enum led_display_mode get_led_mode(void);
void set_led_brightness(uint8_t percent); // 5-100，全局乘数（一改全改）
uint8_t get_led_brightness(void);

// 磁校准进度（0-10000；校准线程写、LED 线程读）
extern volatile uint16_t led_cal_progress;

#endif
