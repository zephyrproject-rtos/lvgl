/*
 * Copyright (c) 2023 Fabian Blatz <fabianblatz@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_LIB_GUI_LVGL_LVGL_INPUT_SINK_H_
#define ZEPHYR_LIB_GUI_LVGL_LVGL_INPUT_SINK_H_

#include <lvgl.h>
#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

lv_indev_t* lvgl_get_mapped_indev(const struct device* dev);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_LIB_GUI_LVGL_LVGL_INPUT_SINK_H_ */
