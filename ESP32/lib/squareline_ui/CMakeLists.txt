SET(SOURCES components/ui_comp_indicatorright.c
    components/ui_comp_slider3.c
    components/ui_comp_x32meter.c
    components/ui_comp.c
    screens/ui_Main.c
    ui_theme_manager.c
    ui_themes.c
    ui.c
    components/ui_comp_hook.c
    ui_helpers.c
    images/ui_img_wifi_disconnected_png.c
    images/ui_img_wifi_connecting_png.c
    images/ui_img_wifi_connected_png.c
    images/ui_img_comppedal_png.c
    images/ui_img_delaypedal_png.c
    images/ui_img_reverbpedal_png.c
    images/ui_img_eqpedal_png.c
    images/ui_img_button_250_50_png.c
    images/ui_img_icn_settings_png.c
    images/ui_img_arrow_left_24_png.c
    fonts/ui_font_14_bold.c
    fonts/ui_font_mos_semibold_16.c
    fonts/ui_font_number.c)

add_library(ui ${SOURCES})
