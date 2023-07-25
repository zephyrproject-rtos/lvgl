/*
 * Copyright (c) 2023 Fabian Blatz <fabianblatz@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "lvgl_input_sink.h"

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/drivers/display.h>
#include <zephyr/sys/iterable_sections.h>
#ifdef CONFIG_LV_Z_POINTER_KSCAN
#include <zephyr/drivers/kscan.h>
#endif
#if defined(CONFIG_LV_Z_POINTER_INPUT) || defined(CONFIG_LV_Z_ENCODER)
#include <zephyr/input/input.h>
#endif
#include "lvgl_display.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(lvgl);

typedef void (*lv_indev_read_cb)(struct _lv_indev_drv_t *indev_drv, lv_indev_data_t *indev_data);

struct lvgl_input_device {
	const struct device *dev;
	lv_indev_drv_t indev_drv;
	lv_indev_t *indev;
	lv_indev_type_t indev_type;
	lv_indev_read_cb indev_read_cb;
};

#define LVGL_INPUT_DEVICE_DEFINE(_dev, _indev_type, _read_cb)                                      \
	static const STRUCT_SECTION_ITERABLE(lvgl_input_device,                                    \
					     _lvgl_input_device__##_read_cb) = {                   \
		.dev = _dev, .indev_type = _indev_type, .indev_read_cb = _read_cb}

#ifdef CONFIG_LV_Z_POINTER

#ifdef CONFIG_LV_Z_POINTER_KSCAN
#define POINTER_DEV_NODE DT_CHOSEN(zephyr_keyboard_scan)
#elif CONFIG_LV_Z_POINTER_INPUT
#define POINTER_DEV_NODE DT_CHOSEN(zephyr_touch_input)
#endif

K_MSGQ_DEFINE(pointer_msgq, sizeof(lv_indev_data_t), CONFIG_LV_Z_POINTER_DRIVER_MSGQ_COUNT, 4);

#ifdef CONFIG_LV_Z_POINTER_KSCAN

static void lvgl_pointer_kscan_callback(const struct device *dev, uint32_t row, uint32_t col,
					bool pressed)
{
	lv_indev_data_t data = {
		.point.x = col,
		.point.y = row,
		.state = pressed ? LV_INDEV_STATE_PR : LV_INDEV_STATE_REL,
	};

	if (k_msgq_put(&pointer_msgq, &data, K_NO_WAIT) != 0) {
		LOG_DBG("Could not put input data into queue");
	}
}

#endif /* CONFIG_LV_Z_POINTER_KSCAN */

#ifdef CONFIG_LV_Z_POINTER_INPUT
static void lvgl_pointer_input_callback(struct input_event *evt)
{
	static lv_indev_data_t pointer_data = {0};

	switch (evt->code) {
	case INPUT_ABS_X:
		pointer_data.point.x = evt->value;
		break;
	case INPUT_ABS_Y:
		pointer_data.point.y = evt->value;
		break;
	case INPUT_BTN_TOUCH:
		pointer_data.state = evt->value ? LV_INDEV_STATE_PR : LV_INDEV_STATE_REL;
		break;
	}

	if (evt->sync) {
		LOG_ERR("input event: %3d %3d %d", pointer_data.point.x, pointer_data.point.y,
			pointer_data.state);
		if (k_msgq_put(&pointer_msgq, &pointer_data, K_NO_WAIT) != 0) {
			LOG_DBG("Could not put input data into queue");
		}
	}
}

INPUT_LISTENER_CB_DEFINE(DEVICE_DT_GET(POINTER_DEV_NODE), lvgl_pointer_input_callback);

#endif /* CONFIG_LV_Z_POINTER_INPUT */

static void lvgl_pointer_read(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
	lv_disp_t *disp;
	struct lvgl_disp_data *disp_data;
	struct display_capabilities *cap;
	lv_indev_data_t curr;

	static lv_indev_data_t prev = {
		.point.x = 0,
		.point.y = 0,
		.state = LV_INDEV_STATE_REL,
	};

	if (k_msgq_get(&pointer_msgq, &curr, K_NO_WAIT) != 0) {
		goto set_and_release;
	}

	prev = curr;

	disp = lv_disp_get_default();
	disp_data = disp->driver->user_data;
	cap = &disp_data->cap;

	/* adjust kscan coordinates */
	if (IS_ENABLED(CONFIG_LV_Z_POINTER_DRIVER_SWAP_XY)) {
		lv_coord_t x;

		x = prev.point.x;
		prev.point.x = prev.point.y;
		prev.point.y = x;
	}

	if (IS_ENABLED(CONFIG_LV_Z_POINTER_DRIVER_INVERT_X)) {
		if (cap->current_orientation == DISPLAY_ORIENTATION_NORMAL ||
		    cap->current_orientation == DISPLAY_ORIENTATION_ROTATED_180) {
			prev.point.x = cap->x_resolution - prev.point.x;
		} else {
			prev.point.x = cap->y_resolution - prev.point.x;
		}
	}

	if (IS_ENABLED(CONFIG_LV_Z_POINTER_DRIVER_INVERT_Y)) {
		if (cap->current_orientation == DISPLAY_ORIENTATION_NORMAL ||
		    cap->current_orientation == DISPLAY_ORIENTATION_ROTATED_180) {
			prev.point.y = cap->y_resolution - prev.point.y;
		} else {
			prev.point.y = cap->x_resolution - prev.point.y;
		}
	}

	/* rotate touch point to match display rotation */
	if (cap->current_orientation == DISPLAY_ORIENTATION_ROTATED_90) {
		lv_coord_t x;

		x = prev.point.x;
		prev.point.x = prev.point.y;
		prev.point.y = cap->y_resolution - x;
	} else if (cap->current_orientation == DISPLAY_ORIENTATION_ROTATED_180) {
		prev.point.x = cap->x_resolution - prev.point.x;
		prev.point.y = cap->y_resolution - prev.point.y;
	} else if (cap->current_orientation == DISPLAY_ORIENTATION_ROTATED_270) {
		lv_coord_t x;

		x = prev.point.x;
		prev.point.x = cap->x_resolution - prev.point.y;
		prev.point.y = x;
	}

	/* filter readings within display */
	if (prev.point.x <= 0) {
		prev.point.x = 0;
	} else if (prev.point.x >= cap->x_resolution) {
		prev.point.x = cap->x_resolution - 1;
	}

	if (prev.point.y <= 0) {
		prev.point.y = 0;
	} else if (prev.point.y >= cap->y_resolution) {
		prev.point.y = cap->y_resolution - 1;
	}

set_and_release:
	*data = prev;

	data->continue_reading = k_msgq_num_used_get(&pointer_msgq) > 0;
}

LVGL_INPUT_DEVICE_DEFINE(DEVICE_DT_GET(POINTER_DEV_NODE), LV_INDEV_TYPE_POINTER, lvgl_pointer_read);
#endif /* CONFIG_LV_Z_POINTER */

#ifdef CONFIG_LV_Z_ENCODER

#define ENCODER_MSGQ(n) encoder_msgq_##n

#define LV_Z_ENCODER_DEFINE(n)                                                                     \
	K_MSGQ_DEFINE(encoder_msgq_##n, sizeof(lv_indev_data_t),                                   \
		      CONFIG_LV_Z_ENCODER_DRIVER_MSGQ_COUNT, 4);                                   \
	static void encoder_input_cb_##n(struct input_event *evt)                                  \
	{                                                                                          \
		lv_indev_data_t data = {.enc_diff = evt->value};                                   \
		if (k_msgq_put(&ENCODER_MSGQ(n), &data, K_NO_WAIT) != 0)                           \
			LOG_DBG("Could not put encoder data into queue");                          \
	}                                                                                          \
	void encoder_read_##n(lv_indev_drv_t *drv, lv_indev_data_t *data)                          \
	{                                                                                          \
		k_msgq_get(&ENCODER_MSGQ(n), data, K_NO_WAIT);                                     \
		data->continue_reading = k_msgq_num_used_get(&ENCODER_MSGQ(n)) > 0;                \
	}                                                                                          \
	INPUT_LISTENER_CB_DEFINE(DEVICE_DT_GET(n), encoder_input_cb_##n);                          \
	LVGL_INPUT_DEVICE_DEFINE(DEVICE_DT_GET(n), LV_INDEV_TYPE_ENCODER, encoder_read_##n);

DT_FOREACH_STATUS_OKAY(gpio_qdec, LV_Z_ENCODER_DEFINE)

#endif /* CONFIG_LV_Z_ENCODER */

lv_indev_t *lvgl_get_mapped_indev(const struct device *dev)
{
	lv_indev_t *indev = NULL;

	STRUCT_SECTION_FOREACH(lvgl_input_device, lvgl_mapped_input_dev) {
		if (lvgl_mapped_input_dev->dev == dev) {
			indev = lvgl_mapped_input_dev->indev;
		}
	}

	return indev;
}

static int lvgl_input_sink_init(void)
{
	STRUCT_SECTION_FOREACH(lvgl_input_device, lvgl_mapped_input_dev) {
		LOG_DBG("Intializing %s", lvgl_mapped_input_dev->dev->name);

		if (!device_is_ready(lvgl_mapped_input_dev->dev)) {
			LOG_ERR("Input device not ready");
			return -ENODEV;
		}

		lv_indev_drv_init(&lvgl_mapped_input_dev->indev_drv);
		lvgl_mapped_input_dev->indev_drv.type = lvgl_mapped_input_dev->indev_type;
		lvgl_mapped_input_dev->indev_drv.read_cb = lvgl_mapped_input_dev->indev_read_cb;
		lvgl_mapped_input_dev->indev =
			lv_indev_drv_register(&lvgl_mapped_input_dev->indev_drv);

		if (lvgl_mapped_input_dev->indev == NULL) {
			LOG_ERR("Could not register input device driver");
			return -EPERM;
		}
#ifdef CONFIG_LV_Z_POINTER_KSCAN
		if (lvgl_mapped_input_dev->indev_type == LV_INDEV_TYPE_POINTER) {

			if (kscan_config(lvgl_mapped_input_dev->dev, lvgl_pointer_kscan_callback) <
			    0) {
				LOG_ERR("Could not configure keyboard scan device.");
				return -ENODEV;
			}
			kscan_enable_callback(lvgl_mapped_input_dev->dev);
		}
#endif /* CONFIG_LV_Z_POINTER_KSCAN */
	}
	return 0;
}

SYS_INIT(lvgl_input_sink_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
