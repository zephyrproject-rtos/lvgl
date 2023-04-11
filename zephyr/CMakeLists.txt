# Copyright (c) 2019 jan.van_winkel@dxplore.eu
# Copyright (c) 2020 Teslabs Engineering S.L.
#
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_LVGL)

set(ZEPHYR_CURRENT_LIBRARY lvgl)
include_directories(${ZEPHYR_BASE}/lib/gui/lvgl)
zephyr_include_directories(../src/)
zephyr_include_directories_ifdef(CONFIG_LV_USE_DEMO_BENCHMARK 
    ../demos/
    ../demos/benchmark
)

zephyr_include_directories_ifdef(CONFIG_LV_USE_DEMO_KEYPAD_AND_ENCODER
    ../demos/
    ../demos/keypad_encoder
)

zephyr_include_directories_ifdef(CONFIG_LV_USE_DEMO_MUSIC
    ../demos/
    ../demos/music
)

zephyr_include_directories_ifdef(CONFIG_LV_USE_DEMO_STRESS
    ../demos/
    ../demos/stress
)

zephyr_include_directories_ifdef(CONFIG_LV_USE_DEMO_WIDGETS
    ../demos/
    ../demos/widgets
)

zephyr_interface_library_named(LVGL)

zephyr_compile_definitions(LV_CONF_INCLUDE_SIMPLE=1)
zephyr_compile_definitions(LV_CONF_PATH=../zephyr/lv_conf.h)

zephyr_library()

zephyr_library_sources(
    ../src/core/lv_refr.c
    ../src/core/lv_indev.c
    ../src/core/lv_group.c
    ../src/core/lv_obj_style.c
    ../src/core/lv_obj.c
    ../src/core/lv_theme.c
    ../src/core/lv_obj_scroll.c
    ../src/core/lv_obj_class.c
    ../src/core/lv_obj_draw.c
    ../src/core/lv_obj_pos.c
    ../src/core/lv_indev_scroll.c
    ../src/core/lv_disp.c
    ../src/core/lv_obj_tree.c
    ../src/core/lv_obj_style_gen.c
    ../src/core/lv_event.c

    ../src/hal/lv_hal_tick.c
    ../src/hal/lv_hal_disp.c
    ../src/hal/lv_hal_indev.c

    ../src/widgets/lv_canvas.c
    ../src/widgets/lv_table.c
    ../src/widgets/lv_slider.c
    ../src/widgets/lv_label.c
    ../src/widgets/lv_roller.c
    ../src/widgets/lv_btn.c
    ../src/widgets/lv_switch.c
    ../src/widgets/lv_dropdown.c
    ../src/widgets/lv_arc.c
    ../src/widgets/lv_img.c
    ../src/widgets/lv_bar.c
    ../src/widgets/lv_textarea.c
    ../src/widgets/lv_checkbox.c
    ../src/widgets/lv_btnmatrix.c
    ../src/widgets/lv_objx_templ.c
    ../src/widgets/lv_line.c

    ../src/misc/lv_area.c
    ../src/misc/lv_bidi.c
    ../src/misc/lv_color.c
    ../src/misc/lv_style_gen.c
    ../src/misc/lv_async.c
    ../src/misc/lv_timer.c
    ../src/misc/lv_gc.c
    ../src/misc/lv_math.c
    ../src/misc/lv_printf.c
    ../src/misc/lv_style.c
    ../src/misc/lv_txt.c
    ../src/misc/lv_templ.c
    ../src/misc/lv_anim_timeline.c
    ../src/misc/lv_mem.c
    ../src/misc/lv_lru.c
    ../src/misc/lv_utils.c
    ../src/misc/lv_anim.c
    ../src/misc/lv_log.c
    ../src/misc/lv_txt_ap.c
    ../src/misc/lv_fs.c
    ../src/misc/lv_ll.c
    ../src/misc/lv_tlsf.c

    ../src/draw/lv_draw_label.c
    ../src/draw/lv_draw_triangle.c
    ../src/draw/lv_img_decoder.c
    ../src/draw/lv_draw_img.c
    ../src/draw/lv_img_cache.c
    ../src/draw/nxp_vglite/lv_gpu_nxp_vglite.c
    ../src/draw/lv_draw_line.c
    ../src/draw/nxp_pxp/lv_gpu_nxp_pxp.c
    ../src/draw/nxp_pxp/lv_gpu_nxp_pxp_osa.c
    ../src/draw/lv_draw.c
    ../src/draw/sdl/lv_draw_sdl_img.c
    ../src/draw/sdl/lv_draw_sdl_composite.c
    ../src/draw/sdl/lv_draw_sdl_polygon.c
    ../src/draw/sdl/lv_draw_sdl_utils.c
    ../src/draw/sdl/lv_draw_sdl_texture_cache.c
    ../src/draw/sdl/lv_draw_sdl.c
    ../src/draw/sdl/lv_draw_sdl_bg.c
    ../src/draw/sdl/lv_draw_sdl_arc.c
    ../src/draw/sdl/lv_draw_sdl_rect.c
    ../src/draw/sdl/lv_draw_sdl_label.c
    ../src/draw/sdl/lv_draw_sdl_stack_blur.c
    ../src/draw/sdl/lv_draw_sdl_mask.c
    ../src/draw/sdl/lv_draw_sdl_line.c
    ../src/draw/lv_img_buf.c
    ../src/draw/lv_draw_mask.c
    ../src/draw/stm32_dma2d/lv_gpu_stm32_dma2d.c
    ../src/draw/lv_draw_arc.c
    ../src/draw/lv_draw_rect.c
    ../src/draw/sw/lv_draw_sw_img.c
    ../src/draw/sw/lv_draw_sw.c
    ../src/draw/sw/lv_draw_sw_arc.c
    ../src/draw/sw/lv_draw_sw_line.c
    ../src/draw/sw/lv_draw_sw_polygon.c
    ../src/draw/sw/lv_draw_sw_rect.c
    ../src/draw/sw/lv_draw_sw_letter.c
    ../src/draw/sw/lv_draw_sw_blend.c
    ../src/draw/sw/lv_draw_sw_dither.c
    ../src/draw/sw/lv_draw_sw_gradient.c

    ../src/font/lv_font_montserrat_32.c
    ../src/font/lv_font.c
    ../src/font/lv_font_montserrat_8.c
    ../src/font/lv_font_unscii_8.c
    ../src/font/lv_font_montserrat_46.c
    ../src/font/lv_font_montserrat_12_subpx.c
    ../src/font/lv_font_montserrat_24.c
    ../src/font/lv_font_montserrat_40.c
    ../src/font/lv_font_loader.c
    ../src/font/lv_font_montserrat_28_compressed.c
    ../src/font/lv_font_fmt_txt.c
    ../src/font/lv_font_montserrat_18.c
    ../src/font/lv_font_montserrat_20.c
    ../src/font/lv_font_montserrat_34.c
    ../src/font/lv_font_montserrat_42.c
    ../src/font/lv_font_montserrat_30.c
    ../src/font/lv_font_montserrat_28.c
    ../src/font/lv_font_dejavu_16_persian_hebrew.c
    ../src/font/lv_font_montserrat_38.c
    ../src/font/lv_font_montserrat_16.c
    ../src/font/lv_font_montserrat_10.c
    ../src/font/lv_font_simsun_16_cjk.c
    ../src/font/lv_font_montserrat_26.c
    ../src/font/lv_font_montserrat_48.c
    ../src/font/lv_font_montserrat_14.c
    ../src/font/lv_font_unscii_16.c
    ../src/font/lv_font_montserrat_36.c
    ../src/font/lv_font_montserrat_44.c
    ../src/font/lv_font_montserrat_22.c
    ../src/font/lv_font_montserrat_12.c

    ../src/extra/lv_extra.c
    ../src/extra/themes/basic/lv_theme_basic.c
    ../src/extra/themes/mono/lv_theme_mono.c
    ../src/extra/themes/default/lv_theme_default.c
    ../src/extra/libs/sjpg/tjpgd.c
    ../src/extra/libs/sjpg/lv_sjpg.c
    ../src/extra/libs/ffmpeg/lv_ffmpeg.c
    ../src/extra/libs/freetype/lv_freetype.c
    ../src/extra/libs/qrcode/lv_qrcode.c
    ../src/extra/libs/qrcode/qrcodegen.c
    ../src/extra/libs/fsdrv/lv_fs_win32.c
    ../src/extra/libs/fsdrv/lv_fs_fatfs.c
    ../src/extra/libs/fsdrv/lv_fs_posix.c
    ../src/extra/libs/fsdrv/lv_fs_stdio.c
    ../src/extra/libs/gif/lv_gif.c
    ../src/extra/libs/gif/gifdec.c
    ../src/extra/libs/bmp/lv_bmp.c
    ../src/extra/libs/png/lv_png.c
    ../src/extra/libs/png/lodepng.c
    ../src/extra/libs/rlottie/lv_rlottie.c
    ../src/extra/widgets/menu/lv_menu.c
    ../src/extra/widgets/chart/lv_chart.c
    ../src/extra/widgets/spinner/lv_spinner.c
    ../src/extra/widgets/spinbox/lv_spinbox.c
    ../src/extra/widgets/colorwheel/lv_colorwheel.c
    ../src/extra/widgets/meter/lv_meter.c
    ../src/extra/widgets/list/lv_list.c
    ../src/extra/widgets/msgbox/lv_msgbox.c
    ../src/extra/widgets/animimg/lv_animimg.c
    ../src/extra/widgets/win/lv_win.c
    ../src/extra/widgets/tileview/lv_tileview.c
    ../src/extra/widgets/span/lv_span.c
    ../src/extra/widgets/calendar/lv_calendar.c
    ../src/extra/widgets/calendar/lv_calendar_header_dropdown.c
    ../src/extra/widgets/calendar/lv_calendar_header_arrow.c
    ../src/extra/widgets/tabview/lv_tabview.c
    ../src/extra/widgets/led/lv_led.c
    ../src/extra/widgets/imgbtn/lv_imgbtn.c
    ../src/extra/widgets/keyboard/lv_keyboard.c
    ../src/extra/layouts/grid/lv_grid.c
    ../src/extra/layouts/flex/lv_flex.c
    ../src/extra/others/monkey/lv_monkey.c
    ../src/extra/others/gridnav/lv_gridnav.c
    ../src/extra/others/snapshot/lv_snapshot.c

    lvgl.c
    lvgl_display.c
    lvgl_display_mono.c
    lvgl_display_16bit.c
    lvgl_display_24bit.c
    lvgl_display_32bit.c
    lvgl_display_mono.c
)

zephyr_library_sources_ifdef(CONFIG_LV_Z_USE_FILESYSTEM lvgl_fs.c)
zephyr_library_sources_ifdef(CONFIG_LV_Z_MEM_POOL_SYS_HEAP lvgl_mem.c)

zephyr_library_sources_ifdef(CONFIG_LV_USE_DEMO_BENCHMARK
    ../demos/benchmark/lv_demo_benchmark.c
    ../demos/benchmark/assets/img_benchmark_cogwheel_alpha16.c
    ../demos/benchmark/assets/img_benchmark_cogwheel_argb.c
    ../demos/benchmark/assets/img_benchmark_cogwheel_chroma_keyed.c
    ../demos/benchmark/assets/img_benchmark_cogwheel_indexed16.c
    ../demos/benchmark/assets/img_benchmark_cogwheel_rgb.c
    ../demos/benchmark/assets/lv_font_bechmark_montserrat_12_compr_az.c.c
    ../demos/benchmark/assets/lv_font_bechmark_montserrat_16_compr_az.c.c
    ../demos/benchmark/assets/lv_font_bechmark_montserrat_28_compr_az.c.c
)

zephyr_library_sources_ifdef(CONFIG_LV_USE_DEMO_KEYPAD_AND_ENCODER
    ../demos/keypad_encoder/lv_demo_keypad_encoder.c
)

zephyr_library_sources_ifdef(CONFIG_LV_USE_DEMO_MUSIC
    ../demos/music/lv_demo_music_list.c
    ../demos/music/lv_demo_music_main.c
    ../demos/music/lv_demo_music.c
    ../demos/music/assets/img_lv_demo_music_btn_corner_large.c
    ../demos/music/assets/img_lv_demo_music_btn_list_pause.c
    ../demos/music/assets/img_lv_demo_music_btn_list_pause_large.c
    ../demos/music/assets/img_lv_demo_music_btn_list_play.c
    ../demos/music/assets/img_lv_demo_music_btn_list_play_large.c
    ../demos/music/assets/img_lv_demo_music_btn_loop.c
    ../demos/music/assets/img_lv_demo_music_btn_loop_large.c
    ../demos/music/assets/img_lv_demo_music_btn_next.c
    ../demos/music/assets/img_lv_demo_music_btn_next_large.c
    ../demos/music/assets/img_lv_demo_music_btn_pause.c
    ../demos/music/assets/img_lv_demo_music_btn_pause_large.c
    ../demos/music/assets/img_lv_demo_music_btn_play.c
    ../demos/music/assets/img_lv_demo_music_btn_play_large.c
    ../demos/music/assets/img_lv_demo_music_btn_prev.c
    ../demos/music/assets/img_lv_demo_music_btn_prev_large.c
    ../demos/music/assets/img_lv_demo_music_btn_rnd.c
    ../demos/music/assets/img_lv_demo_music_btn_rnd_large.c
    ../demos/music/assets/img_lv_demo_music_corner_left.c
    ../demos/music/assets/img_lv_demo_music_corner_left_large.c
    ../demos/music/assets/img_lv_demo_music_corner_right.c
    ../demos/music/assets/img_lv_demo_music_corner_right_large.c
    ../demos/music/assets/img_lv_demo_music_cover_1.c
    ../demos/music/assets/img_lv_demo_music_cover_1_large.c
    ../demos/music/assets/img_lv_demo_music_cover_2.c
    ../demos/music/assets/img_lv_demo_music_cover_2_large.c
    ../demos/music/assets/img_lv_demo_music_cover_3.c
    ../demos/music/assets/img_lv_demo_music_cover_3_large.c
    ../demos/music/assets/img_lv_demo_music_icon_1.c
    ../demos/music/assets/img_lv_demo_music_icon_1_large.c
    ../demos/music/assets/img_lv_demo_music_icon_2.c
    ../demos/music/assets/img_lv_demo_music_icon_2_large.c
    ../demos/music/assets/img_lv_demo_music_icon_3.c
    ../demos/music/assets/img_lv_demo_music_icon_3_large.c
    ../demos/music/assets/img_lv_demo_music_icon_4.c
    ../demos/music/assets/img_lv_demo_music_icon_4_large.c
    ../demos/music/assets/img_lv_demo_music_list_border.c
    ../demos/music/assets/img_lv_demo_music_list_border_large.c
    ../demos/music/assets/img_lv_demo_music_logo.c
    ../demos/music/assets/img_lv_demo_music_slider_knob.c
    ../demos/music/assets/img_lv_demo_music_slider_knob_large.c
    ../demos/music/assets/img_lv_demo_music_wave_bottom.c
    ../demos/music/assets/img_lv_demo_music_wave_bottom_large.c
    ../demos/music/assets/img_lv_demo_music_wave_top.c
    ../demos/music/assets/img_lv_demo_music_wave_top_large.c
)

zephyr_library_sources_ifdef(CONFIG_LV_USE_DEMO_STRESS
    ../demos/stress/lv_demo_stress.c
    ../demos/stress/assets/lv_font_montserrat_12_compr_az.c
    ../demos/stress/assets/lv_font_montserrat_16_compr_az.c
    ../demos/stress/assets/lv_font_montserrat_28_compr_az.c
)

zephyr_library_sources_ifdef(CONFIG_LV_USE_DEMO_WIDGETS
    ../demos/widgets/lv_demo_widgets.c
    ../demos/widgets/assets/img_clothes.c
    ../demos/widgets/assets/img_demo_widgets_avatar.c
    ../demos/widgets/assets/img_lvgl_logo.c
    ../demos/widgets/assets/img_clothes.c
)

zephyr_library_link_libraries(LVGL)
target_link_libraries(LVGL INTERFACE zephyr_interface)

endif()
