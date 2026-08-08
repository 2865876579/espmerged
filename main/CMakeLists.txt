idf_component_register(
    SRCS
        "main.c"
        "wifi.c"
        "audio_out.c"
        "audio_in.c"
        "ws_client.c"
        "afe_wake_word.c"
        "pump_driver.c"
        "led_strip_driver.c"
        "sensors.c"
        "screen_anim.c"
        "lcd_ili9488.c"
        "avatar_storage.c"
        "avatar_rgb666_frames.c"
        "monitor_printf_filter.c"
        "snore_detector.cpp"
        "font_gb2312_16.c"
    INCLUDE_DIRS "."
    EMBED_FILES
        "avatar_base_rgb666.bin"
        "avatar_head_rgb666_frame_0.bin"
        "avatar_head_rgb666_frame_1.bin"
        "avatar_head_rgb666_frame_2.bin"
        "avatar_head_rgb666_frame_3.bin"
        "avatar_head_rgb666_frame_4.bin"
        "avatar_head_rgb666_frame_5.bin"
        "avatar_head_rgb666_frame_6.bin"
        "avatar_head_rgb666_frame_7.bin"
        "avatar_head_rgb666_frame_8.bin"
        "avatar_head_rgb666_frame_9.bin"
        "avatar_head_rgb666_frame_10.bin"
        "avatar_head_rgb666_frame_11.bin"
        "font_gb2312_16.bin"
    REQUIRES
        esp_wifi
        esp_event
        esp_netif
        nvs_flash
        driver
        esp_lcd
        esp_partition
        esp_http_client
        esp_adc
        esp_websocket_client
        cjson
        mbedtls
        esp-sr
        sensors
        edge_impulse
        esp_timer
        heap
)

target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wno-error=maybe-uninitialized
    -Wno-error=missing-field-initializers
)

# ESP-SR 2.4.6 的预编译 WakeNet 库用裸 printf 输出两条初始化信息，
# 不受 ESP_LOG 等级控制。只包装 printf 并精确过滤这两个格式串，
# 其余比赛监视器事件仍按原样输出。
target_link_libraries(${COMPONENT_LIB} INTERFACE "-Wl,--wrap=printf")
