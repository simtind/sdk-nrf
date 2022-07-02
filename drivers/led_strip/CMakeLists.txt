# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_WS2812_STRIP_NRFX_PWM   ws2812_nrfx_pwm.c)
