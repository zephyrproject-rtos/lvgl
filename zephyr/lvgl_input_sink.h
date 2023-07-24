/*
 * Copyright (c) 2023 Fabian Blatz <fabianblatz@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_LIB_GUI_LVGL_LVGL_INPUT_SINK_H_
#define ZEPHYR_LIB_GUI_LVGL_LVGL_INPUT_SINK_H_

#ifdef __cplusplus
extern "C" {
#endif

#ifdef CONFIG_LV_Z_POINTER
int lvgl_pointer_drv_init(void);
#endif /* CONFIG_LV_Z_POINTER */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_LIB_GUI_LVGL_LVGL_INPUT_SINK_H_ */