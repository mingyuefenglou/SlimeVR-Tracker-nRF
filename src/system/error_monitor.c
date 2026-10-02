/*
 * Copyright (c) 2025 SlimeVR Contributors
 *
 * SPDX-License-Identifier: MIT
 */

/* 错误自愈监控：持续硬件/系统错误（传感器错误、系统级错误等）自动重启自愈，
 * 镜像 watchdog 的 loop-guard 模式防止「反复/不断重启」：
 *  - 只监控 SYS_STATUS_ERROR（SENSOR|SYSTEM）；CONNECTION_ERROR 属正常瞬断不触发。
 *  - 错误须连续存在超过 10s 才动作（滤掉瞬时抖动，也给开机初始化阶段留时间）。
 *  - OTA 传输中（active 或 suppressed）冻结监控——绝不在升级中重启。
 *  - 每次自愈重启 error_reboot.count++（retained 持久化、CRC 外、跨重启保留）；
 *    达到 5 次上限后停止自动重启，保持错误红灯状态等手动恢复（长按 12s 重启/断电）。
 *  - 连续 60s 无错误后计数清零——瞬时故障自愈不累积封禁。
 * 用 k_work_delayable 挂系统 workqueue，不新建线程（tracker RAM 已近告警线）。 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>

#include "globals.h"
#include "status.h"
#include "led.h"
#include "system.h"
#include "esb_ota.h"
#include "connection/connection.h"

LOG_MODULE_REGISTER(error_monitor, LOG_LEVEL_INF);

#define ERRMON_POLL_MS        1000  /* 轮询周期 */
#define ERRMON_ERROR_HOLD_MS  10000 /* 错误连续存在超过该时长才自愈重启 */
#define ERRMON_CLEAN_RESET_MS 60000 /* 连续无错误该时长后自愈计数清零 */
#define ERRMON_FADE_MS        1350  /* 告别渐灭时长（与按键路径同款动画） */
#define ERRMON_MAX_REBOOTS    5     /* 自愈重启次数上限（loop guard） */

static struct k_work_delayable error_monitor_work;
static bool reboot_pending; /* 已触发自愈，等渐灭动画播完再提交重启 */

static void error_monitor_tick(struct k_work *work)
{
	static int64_t error_since; /* 错误起始时刻；0=当前无错误 */
	static int64_t clean_since; /* 连续无错误起始时刻；0=尚未开始计时 */
	static bool guard_logged;   /* loop-guard 告警只打一次 */
	int64_t now = k_uptime_get();

	if (reboot_pending) {
		/* 渐灭动画已播完：错误仍在则提交重启（异步请求，不在 workqueue 里睡眠） */
		reboot_pending = false;
		if (!get_status(SYS_STATUS_ERROR)) {
			LOG_INF("Error cleared during fade; self-heal reboot canceled");
			k_work_reschedule(&error_monitor_work, K_MSEC(ERRMON_POLL_MS));
		} else if (esb_ota_is_active() || connection_get_ota_suppressed()) {
			LOG_INF("Self-heal reboot deferred by OTA");
			k_work_reschedule(&error_monitor_work, K_MSEC(ERRMON_POLL_MS));
		} else {
			if (retained->error_reboot.magic != ERROR_REBOOT_STATE_MAGIC) {
				/* retained 状态无效（首次上电/脏数据）：重置计数 */
				retained->error_reboot.magic = ERROR_REBOOT_STATE_MAGIC;
				retained->error_reboot.count = 0;
			}
			retained->error_reboot.count++;
			LOG_ERR("Self-heal reboot %d/%d", retained->error_reboot.count,
				ERRMON_MAX_REBOOTS);
			int err = sys_request_system_reboot();
			if (err) {
				LOG_WRN("Self-heal reboot rejected: %d", err);
				error_since = 0; /* 重新计满 10s 窗口，避免立刻再次触发 */
				k_work_reschedule(&error_monitor_work, K_MSEC(ERRMON_POLL_MS));
			}
			/* 成功则设备即将重启，不再调度 */
		}
		return;
	}

	if (esb_ota_is_active() || connection_get_ota_suppressed()) {
		/* OTA 传输中冻结计时（既不累积也不触发） */
		k_work_reschedule(&error_monitor_work, K_MSEC(ERRMON_POLL_MS));
		return;
	}

	if (get_status(SYS_STATUS_ERROR)) {
		clean_since = 0;
		if (!error_since) {
			error_since = now;
		} else if (now - error_since >= ERRMON_ERROR_HOLD_MS) {
			if (retained->error_reboot.magic != ERROR_REBOOT_STATE_MAGIC) {
				retained->error_reboot.magic = ERROR_REBOOT_STATE_MAGIC;
				retained->error_reboot.count = 0;
			}
			if (retained->error_reboot.count >= ERRMON_MAX_REBOOTS) {
				/* loop guard：疑似 boot-loop，停止自动重启，保持错误灯态等手动恢复 */
				if (!guard_logged) {
					guard_logged = true;
					LOG_ERR("Error persists after %d self-heal reboots, auto-reboot disabled",
						retained->error_reboot.count);
				}
			} else {
				LOG_ERR("Error held %d s, initiating self-heal reboot",
					(int)((now - error_since) / 1000));
				/* 播告别渐灭，下个 tick 再提交重启（若错误期间消失则取消） */
				set_led(SYS_LED_PATTERN_ONESHOT_POWEROFF, SYS_LED_PRIORITY_HIGHEST);
				reboot_pending = true;
				k_work_reschedule(&error_monitor_work, K_MSEC(ERRMON_FADE_MS));
				return;
			}
		}
	} else {
		error_since = 0;
		if (!clean_since) {
			clean_since = now;
		} else if (retained->error_reboot.magic == ERROR_REBOOT_STATE_MAGIC &&
			   retained->error_reboot.count != 0 &&
			   now - clean_since >= ERRMON_CLEAN_RESET_MS) {
			LOG_INF("Error-free for 60 s: self-heal reboot count reset");
			retained->error_reboot.count = 0;
			guard_logged = false;
		}
	}

	k_work_reschedule(&error_monitor_work, K_MSEC(ERRMON_POLL_MS));
}

static int error_monitor_init(void)
{
	k_work_init_delayable(&error_monitor_work, error_monitor_tick);
	k_work_reschedule(&error_monitor_work, K_MSEC(ERRMON_POLL_MS));
	return 0;
}

SYS_INIT(error_monitor_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
