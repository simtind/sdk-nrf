/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/debug/stack.h>
#include <zephyr/device.h>
#include <string.h>

#include "macros_common.h"
#include "fw_info_app.h"
#include "led.h"
#include "button_handler.h"
#include "button_assignments.h"
#include "nrfx_clock.h"
#include "streamctrl.h"
#include "ble_core.h"
#include "pmic.h"
#include "power_module.h"
#include "sd_card.h"
#include "board_version.h"
#include "audio_datapath.h"
#include "audio_i2s.h"
#include "channel_assignment.h"
#include "hw_codec.h"
#include "audio_usb.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, CONFIG_LOG_MAIN_LEVEL);

#if defined(CONFIG_INIT_STACKS)
/* Used for printing stack usage */
extern struct k_thread z_main_thread;
#endif /* defined(CONFIG_INIT_STACKS) */

static atomic_t ble_core_is_ready = (atomic_t) false;
static struct board_version board_rev;

static int hfclock_config_and_start(void)
{
	int ret;

	/* Use this to turn on 128 MHz clock for cpu_app */
	ret = nrfx_clock_divider_set(NRF_CLOCK_DOMAIN_HFCLK, NRF_CLOCK_HFCLK_DIV_1);

	ret -= NRFX_ERROR_BASE_NUM;
	if (ret) {
		return ret;
	}

	nrfx_clock_hfclk_start();
	while (!nrfx_clock_hfclk_is_running()) {
	}

	return 0;
}

static int leds_set(void)
{
	int ret;

	/* Blink LED 3 to indicate that APP core is running */
	ret = led_blink(LED_APP_3_GREEN);
	if (ret) {
		return ret;
	}

#if (CONFIG_AUDIO_DEV == HEADSET)
	enum audio_channel channel;

	ret = channel_assignment_get(&channel);
	if (ret) {
		/* Channel is not assigned yet: use default */
		channel = AUDIO_CHANNEL_DEFAULT;
	}

	if (channel == AUDIO_CHANNEL_LEFT) {
		ret = led_on(LED_APP_RGB, LED_COLOR_BLUE);
	} else {
		ret = led_on(LED_APP_RGB, LED_COLOR_MAGENTA);
	}

	if (ret) {
		return ret;
	}
#elif (CONFIG_AUDIO_DEV == GATEWAY)
	ret = led_on(LED_APP_RGB, LED_COLOR_GREEN);
	if (ret) {
		return ret;
	}
#endif /* (CONFIG_AUDIO_DEV == HEADSET) */

	return 0;
}

static int channel_assign_check(void)
{
#if (CONFIG_AUDIO_DEV == HEADSET) && CONFIG_AUDIO_HEADSET_CHANNEL_RUNTIME
	if (!channel_assignment_get(&(enum audio_channel){ AUDIO_CHANNEL_LEFT })) {
		/* Channel already assigned */
		return 0;
	}

	int ret;
	bool pressed;

	ret = button_pressed(BUTTON_VOLUME_DOWN, &pressed);
	if (ret) {
		return ret;
	}

	if (pressed) {
		return channel_assignment_set(AUDIO_CHANNEL_LEFT);
	}

	ret = button_pressed(BUTTON_VOLUME_UP, &pressed);
	if (ret) {
		return ret;
	}

	if (pressed) {
		return channel_assignment_set(AUDIO_CHANNEL_RIGHT);
	}
#endif

	return 0;
}

/* Callback from ble_core when the ble subsystem is ready */
void on_ble_core_ready(void)
{
	(void)atomic_set(&ble_core_is_ready, (atomic_t) true);
}

void main(void)
{
	int ret;

	LOG_DBG("nRF5340 APP core started");

	ret = hfclock_config_and_start();
	ERR_CHK(ret);

	ret = led_init();
	ERR_CHK(ret);

	ret = button_handler_init();
	ERR_CHK(ret);

	ret = channel_assign_check();
	ERR_CHK(ret);

	ret = fw_info_app_print();
	ERR_CHK(ret);

	ret = board_version_valid_check();
	ERR_CHK(ret);

	ret = board_version_get(&board_rev);
	ERR_CHK(ret);

	if (board_rev.mask & BOARD_REVISION_VALID_MSK_MAX14690_PMIC) {
		ret = pmic_init();
		ERR_CHK(ret);

		ret = pmic_defaults_set();
		ERR_CHK(ret);
	}

	if (board_rev.mask & BOARD_VERSION_VALID_MSK_SD_CARD) {
		ret = sd_card_init();
		if (ret != -ENODEV) {
			ERR_CHK(ret);
		}
	}

	ret = power_module_init();
	ERR_CHK(ret);

#if ((CONFIG_AUDIO_DEV == GATEWAY) && (CONFIG_AUDIO_SOURCE_USB))
	ret = audio_usb_init();
	ERR_CHK(ret);
#else
	ret = audio_datapath_init();
	ERR_CHK(ret);
	audio_i2s_init();
	ret = hw_codec_init();
	ERR_CHK(ret);
#endif

	/* Initialize BLE, with callback for when BLE is ready */
	ret = ble_core_init(on_ble_core_ready);
	ERR_CHK(ret);

	/* Wait until ble_core/NET core is ready */
	while (!(bool)atomic_get(&ble_core_is_ready)) {
		(void)k_sleep(K_MSEC(100));
	}

	ret = leds_set();
	ERR_CHK(ret);

	ret = streamctrl_start();
	ERR_CHK(ret);

	ret = audio_datapath_tone_play(440, 500, 0.2);
	ERR_CHK(ret);

	while (1) {
		streamctrl_event_handler();
		STACK_USAGE_PRINT("main", &z_main_thread);
	}
}
