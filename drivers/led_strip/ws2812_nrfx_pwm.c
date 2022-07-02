/*
 * Copyright (c) 2017 Linaro Limited
 * Copyright (c) 2019, Nordic Semiconductor ASA
 * Copyright (c) 2021 Seagate Technology LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT worldsemi_ws2812_nrfx_pwm

#include <drivers/led_strip.h>
#include <drivers/pinctrl.h>

#include <string.h>

#define LOG_LEVEL CONFIG_LED_STRIP_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(ws2812_pwm);

#include <string.h>
#include <stdlib.h>

#include <zephyr.h>
#include <device.h>
#include <sys/math_extras.h>
#include <sys/util.h>
#include <dt-bindings/led/led.h>
#include <nrfx.h>
#include <nrfx_pwm.h>

/* Internal defines */
#define PWM_POLARITY        0x8000              // First change in PWM period is HIGH to LOW
#define PWM_1_SEQ_VAL       PWM_POLARITY | 13UL // 13 for 16MHz is high period of 1, 0.8125us
#define PWM_0_SEQ_VAL       PWM_POLARITY | 6UL  // 6 for 16MHz is high period of 0, 0.375us
#define PWM_COMPARE_TOP_AT_16MHZ     20UL       // Compare 20 at 16MHz gives 1.25us period

struct ws2812_nrfx_pwm_cfg {
    struct k_sem * running;
	uint16_t *px_buf;
	size_t px_buf_size;  
    uint8_t num_leds;
	uint8_t num_colors;
	const uint8_t *color_mapping;
	uint16_t reset_delay;
	nrfx_pwm_t pwm;
	nrfx_pwm_config_t initial_config;
    void (* irq_setup)(void);
#ifdef CONFIG_PINCTRL
	const struct pinctrl_dev_config *pcfg;
#endif
};

static const struct ws2812_nrfx_pwm_cfg *dev_cfg(const struct device *dev)
{
	return dev->config;
}

/*
 * Serialize an 8-bit color channel value into an equivalent sequence
 * of PWM frames, MSbit first, where a one bit becomes PWM frame
 * one_frame, and zero bit becomes zero_frame.
 */
static void ws2812_nrfx_pwm_interrupt_handler(nrfx_pwm_evt_type_t pwm_evt, void * context) {
    if (pwm_evt == NRFX_PWM_EVT_STOPPED) {
        k_sem_give(((struct ws2812_nrfx_pwm_cfg *)context)->running);
    }    
}

/*
 * Serialize an 8-bit color channel value into an equivalent sequence
 * of PWM frames, MSbit first, where a one bit becomes PWM frame
 * one_frame, and zero bit becomes zero_frame.
 */
static inline void ws2812_nrfx_pwm_ser(uint16_t buf[8], uint8_t color)
{    
    int i;

	for (i = 0; i < 8; i++) {
		buf[i] = color & BIT(7 - i) ? PWM_1_SEQ_VAL : PWM_0_SEQ_VAL;
	}
}

/*
 * Returns true if and only if cfg->px_buf is big enough to convert
 * num_pixels RGB color values into SPI frames.
 */
static inline bool num_pixels_ok(const struct ws2812_nrfx_pwm_cfg *cfg,
				 size_t num_pixels)
{
	size_t nbytes;
	bool overflow;

	overflow = size_mul_overflow(num_pixels, cfg->num_colors * 8, &nbytes);
	return !overflow && (nbytes <= cfg->px_buf_size);
}

/*
 * Latch current color values on strip and reset its state machines.
 */
static inline void ws2812_reset_delay(uint16_t delay)
{
	k_usleep(delay);
}

static int ws2812_strip_update_rgb(const struct device *dev,
				   struct led_rgb *pixels,
				   size_t num_pixels)
{
	const struct ws2812_nrfx_pwm_cfg *cfg = dev_cfg(dev);
	uint16_t *px_buf = cfg->px_buf;
	size_t i;

	if (!num_pixels_ok(cfg, num_pixels)) {
		return -ENOMEM;
	}

    if (!nrfx_pwm_is_stopped(&cfg->pwm)) {
        return -EBUSY;
    }

	/*
	 * Convert pixel data into PWM frames. Each frame has pixel data
	 * in color mapping on-wire format (e.g. GRB, GRBW, RGB, etc).
	 */
	for (i = 0; i < num_pixels; i++) {
		uint8_t j;

		for (j = 0; j < cfg->num_colors; j++) {
			uint8_t pixel;

			switch (cfg->color_mapping[j]) {
			/* White channel is not supported by LED strip API. */
			case LED_COLOR_ID_WHITE:
				pixel = 0;
				break;
			case LED_COLOR_ID_RED:
				pixel = pixels[i].r;
				break;
			case LED_COLOR_ID_GREEN:
				pixel = pixels[i].g;
				break;
			case LED_COLOR_ID_BLUE:
				pixel = pixels[i].b;
				break;
			default:
				return -EINVAL;
			}
			ws2812_nrfx_pwm_ser(px_buf, pixel);
			px_buf += 8;
		}
	}

	/*
	 * Display the pixel data.
	 */
    
    nrf_pwm_sequence_t const seq =
    {
        .values.p_common = cfg->px_buf,
        .length          = cfg->px_buf_size,
        .repeats         = 0,
        .end_delay       = 40
    };
    
    nrfx_err_t err_code = nrfx_pwm_simple_playback(&cfg->pwm, &seq, 1, NRFX_PWM_FLAG_STOP);
    
    // PWM peripheral takes 1us / slot, wait max 10 times longer than neccessary
    uint32_t timeout_us = cfg->px_buf_size * 10;
    if (err_code == NRFX_SUCCESS && k_sem_take(cfg->running, K_USEC(timeout_us)) != 0)
    {
        LOG_ERR("%s: Timeout waiting for PWM transfer to finish after %i us",
            dev->name, timeout_us);
        return -ETIMEDOUT;
    }

	ws2812_reset_delay(cfg->reset_delay);
    
    if (err_code != NRFX_SUCCESS)
    {
		return -err_code;
    }   
    return 0;
}

static int ws2812_strip_update_channels(const struct device *dev,
					uint8_t *channels,
					size_t num_channels)
{
	LOG_ERR("update_channels not implemented");
	return -ENOTSUP;
}


static int ws2812_nrfx_pwm_init(const struct device *dev)
{
	const struct ws2812_nrfx_pwm_cfg *config = dev->config;
    
    // Initialize nrfx interrupts
    config->irq_setup();
    
#ifdef CONFIG_PINCTRL
	int ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);

	if (ret < 0) {
		return ret;
	}
#endif
    
    config->px_buf[config->px_buf_size - 1] = PWM_POLARITY;
    
    nrfx_err_t err_code = nrfx_pwm_init(&config->pwm, 
        &config->initial_config,
        ws2812_nrfx_pwm_interrupt_handler, 
        (void *)config);
    if (err_code != NRFX_SUCCESS)
    {
		LOG_ERR("Failed to initialize device: %s, %i", dev->name, err_code);
		return -EBUSY;
    }    

	for (int i = 0; i < config->num_colors; i++) {
		switch (config->color_mapping[i]) {
		case LED_COLOR_ID_WHITE:
		case LED_COLOR_ID_RED:
		case LED_COLOR_ID_GREEN:
		case LED_COLOR_ID_BLUE:
			break;
		default:
			LOG_ERR("%s: invalid channel to color mapping."
				"Check the color-mapping DT property",
				dev->name);
			return -EINVAL;
		}
	}

	return 0;
}

static const struct led_strip_driver_api ws2812_nrfx_pwm_api = {
	.update_rgb = ws2812_strip_update_rgb,
	.update_channels = ws2812_strip_update_channels,
};

/* WS2812 configuration getters */
#define WS2812_PWM_NUM_PIXELS(idx) \
	(DT_INST_PROP(idx, chain_length))
#define WS2812_PWM_HAS_WHITE(idx) \
	(DT_INST_PROP(idx, has_white_channel) == 1)
#define WS2812_PWM_BUFSZ(idx) \
	((WS2812_NUM_COLORS(idx) * 8 * WS2812_PWM_NUM_PIXELS(idx) + 1))

/* PWM configuration getters */
#define PWM(dev_idx) DT_NODELABEL(pwm##dev_idx)
#define PWM_PROP(dev_idx, prop) DT_PROP(PWM(dev_idx), prop)

#define PWM_CH_INVERTED(dev_idx, ch_idx) \
	PWM_PROP(dev_idx, ch##ch_idx##_inverted)

#define PWM_OUTPUT_PIN(dev_idx, ch_idx)					\
	COND_CODE_1(DT_NODE_HAS_PROP(PWM(dev_idx), ch##ch_idx##_pin),	\
		(PWM_PROP(dev_idx, ch##ch_idx##_pin) |			\
			(PWM_CH_INVERTED(dev_idx, ch_idx)		\
			 ? NRFX_PWM_PIN_INVERTED : 0)),			\
		(NRFX_PWM_PIN_NOT_USED))

#define PWM_IRQ_SETUP_NAME(dev_idx) pwm_##dev_idx##_irq_setup

#define PWM_IRQ_SETUP(dev_idx) \
    static void PWM_IRQ_SETUP_NAME(dev_idx)(){ \
        IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_PWM##dev_idx),		       \
            DT_IRQ(PWM(dev_idx), priority),		       \
            nrfx_isr, nrfx_pwm_##dev_idx##_irq_handler, 0);       \
    }

/*
 * Retrieve the channel to color mapping (e.g. RGB, BGR, GRB, ...) from the
 * "color-mapping" DT property.
 */
#define WS2812_COLOR_MAPPING(idx)				  \
	static const uint8_t ws2812_nrfx_pwm_##idx##_color_mapping[] = \
		DT_INST_PROP(idx, color_mapping)

#define WS2812_NUM_COLORS(idx) (DT_INST_PROP_LEN(idx, color_mapping))

/* Get the latch/reset delay from the "reset-delay" DT property. */
#define WS2812_RESET_DELAY(idx) DT_INST_PROP(idx, reset_delay)

#define WS2812_PWM_DEVICE(idx)						 \
	NRF_DT_CHECK_PIN_ASSIGNMENTS(PWM(idx), 1,			      \
				     ch0_pin, ch1_pin, ch2_pin, ch3_pin);     \
									 \
	static uint16_t ws2812_nrfx_pwm_##idx##_px_buf[WS2812_PWM_BUFSZ(idx)]; \
									 \
	WS2812_COLOR_MAPPING(idx);					 \
    PWM_IRQ_SETUP(idx);  \
    K_SEM_DEFINE(ws2812_nrfx_pwm_##idx##_running_sem, 0, 1)  \
									 \
	IF_ENABLED(CONFIG_PINCTRL, (PINCTRL_DT_DEFINE(PWM(idx))));	      \
	static const struct ws2812_nrfx_pwm_cfg ws2812_nrfx_pwm_##idx##_cfg = {	 \
		.running = &ws2812_nrfx_pwm_##idx##_running_sem,     \
        .px_buf = ws2812_nrfx_pwm_##idx##_px_buf,			 \
		.px_buf_size = WS2812_PWM_BUFSZ(idx),			 \
		.num_colors = WS2812_NUM_COLORS(idx),			 \
		.color_mapping = ws2812_nrfx_pwm_##idx##_color_mapping,	 \
		.reset_delay = WS2812_RESET_DELAY(idx),			 \
		.pwm = NRFX_PWM_INSTANCE(idx),				      \
		.initial_config = {					      \
			COND_CODE_1(CONFIG_PINCTRL,			      \
				(.skip_gpio_cfg = true,			      \
				 .skip_psel_cfg = true,),		      \
				(.output_pins = {			      \
					PWM_OUTPUT_PIN(idx, 0),		      \
					PWM_OUTPUT_PIN(idx, 1),		      \
					PWM_OUTPUT_PIN(idx, 2),		      \
					PWM_OUTPUT_PIN(idx, 3),		      \
				 },))					      \
            .irq_priority = NRFX_PWM_DEFAULT_CONFIG_IRQ_PRIORITY, \
			.base_clock = NRF_PWM_CLK_16MHz,			      \
			.count_mode = NRF_PWM_MODE_UP,		      \
			.top_value = 20,				      \
			.load_mode = NRF_PWM_LOAD_COMMON,		      \
			.step_mode = NRF_PWM_STEP_AUTO,		      \
		},							      \
        .irq_setup = PWM_IRQ_SETUP_NAME(idx), \
		IF_ENABLED(CONFIG_PINCTRL,				      \
			(.pcfg = PINCTRL_DT_DEV_CONFIG_GET(PWM(idx)),))	      \
	};				                         \
									 \
	DEVICE_DT_INST_DEFINE(idx,					 \
			      ws2812_nrfx_pwm_init,				 \
			      NULL,					 \
			      NULL,					 \
			      &ws2812_nrfx_pwm_##idx##_cfg,			 \
			      POST_KERNEL,				 \
			      CONFIG_LED_STRIP_INIT_PRIORITY,		 \
			      &ws2812_nrfx_pwm_api);

DT_INST_FOREACH_STATUS_OKAY(WS2812_PWM_DEVICE)

