file(GLOB_RECURSE SOURCES "*.cpp")

idf_component_register(
    SRCS ${SOURCES}
    INCLUDE_DIRS "."
    REQUIRES
        esp_driver_gpio
        esp_wifi
        esp_event
        esp_netif
        nvs_flash
        led_strip
        esp_timer
)