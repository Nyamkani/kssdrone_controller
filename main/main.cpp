// #include <stdio.h>



// /*

// 좌 스틱: KY-023

// // 우 스틱: KY-023

// ARM: 2단 미니 토글 스위치

// MODE: 3단 토글 또는 3단 슬라이드 스위치

// PAIR / MENU / BEEP: 6x6 택트 스위치 1~2개

// POWER: 슬라이드 스위치

// ARM: 미니 2단 토글 1개

// MODE: 미니 3단 토글 1개

// PAIR: 6x6 택트 1개

// BEEP/TEST: 6x6 택트 1개

// POWER: 슬라이드 1개

// */
// extern "C" void app_main(void)
// {

// }


/////////////////////////////////////////////////////
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "esp_log.h"

// #include "ps2_pad.h"
// #include "driver/gpio.h"


// static const char* TAG = "APP";

// extern "C" void app_main(void)
// {
//     // 예시 핀
//     // 수신기 핀:
//     // VCC / GND / CLK / DATA / CMD / ATT
//     Ps2Pad pad(
//         GPIO_NUM_13,  // ATT
//         GPIO_NUM_12,  // CMD
//         GPIO_NUM_11,  // DAT
//         GPIO_NUM_10   // CLK
//     );

//     ESP_ERROR_CHECK(pad.Initialize());

//     printf(""
//            "PS2 Pad Initialized.\n"
//            "Reading state every 20ms...\n");

//     while (true)
//     {
//         Ps2Pad::PadState st{};
//         const esp_err_t ret = pad.Read(st);

//         if (ret == ESP_OK && st.valid)
//         {
//             ESP_LOGI(TAG,
//                      "btn=0x%04X lx=%3u ly=%3u rx=%3u ry=%3u",
//                      st.buttons,
//                      st.lx,
//                      st.ly,
//                      st.rx,
//                      st.ry);
//         }
//         else
//         {
//             ESP_LOGW(TAG, "pad read failed");
//         }

//         vTaskDelay(pdMS_TO_TICKS(20)); // 50Hz
//     }
// }


///////////////////

/*
esp32s3- supermini pinout:
        GPIO_NUM_13,  // ATT
        GPIO_NUM_12,  // CMD
        GPIO_NUM_11,  // DAT
        GPIO_NUM_10   // CLK


#define CONTROLLER_ATT_PIN GPIO_NUM_13
#define CONTROLLER_CMD_PIN GPIO_NUM_12
#define CONTROLLER_DAT_PIN GPIO_NUM_11
#define CONTROLLER_CLK_PIN GPIO_NUM_10 

esp32s3 - seeeed xiao pinout:
        GPIO_NUM_5,  // ATT
        GPIO_NUM_4,  // CMD
        GPIO_NUM_3,  // DAT
        GPIO_NUM_2   // CLK

// #define CONTROLLER_ATT_PIN GPIO_NUM_5
// #define CONTROLLER_CMD_PIN GPIO_NUM_4
// #define CONTROLLER_DAT_PIN GPIO_NUM_3
// #define CONTROLLER_CLK_PIN GPIO_NUM_2   

*/

// #define CONTROLLER_ATT_PIN GPIO_NUM_5
// #define CONTROLLER_CMD_PIN GPIO_NUM_4
// #define CONTROLLER_DAT_PIN GPIO_NUM_3
// #define CONTROLLER_CLK_PIN GPIO_NUM_2   


#define CONTROLLER_ATT_PIN GPIO_NUM_5   //blue
#define CONTROLLER_CMD_PIN GPIO_NUM_4   //orange
#define CONTROLLER_DAT_PIN GPIO_NUM_3   //brown
#define CONTROLLER_CLK_PIN GPIO_NUM_2   //yellow


#include "esp_err.h"

#include "kss_controller.h"
#include "esp_rom_sys.h"

extern "C" void app_main(void)
{
    static KssController controller(
        CONTROLLER_ATT_PIN,  // ATT
        CONTROLLER_CMD_PIN,  // CMD
        CONTROLLER_DAT_PIN,  // DAT
        CONTROLLER_CLK_PIN   // CLK
    );

    ESP_ERROR_CHECK(controller.Initialize());

    vTaskDelete(nullptr);
}

/////////////////////////////////////////////////////
// #include <cstdio>

// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"

// #include "esp_err.h"
// #include "esp_log.h"
// #include "esp_mac.h"
// #include "esp_system.h"

// #include "led_strip.h"

// static const char* TAG = "WS2812_TEST";

// static constexpr gpio_num_t WS2812_GPIO = GPIO_NUM_7;
// static constexpr uint32_t WS2812_LED_COUNT = 2;

// // 밝기 낮게 (0~255)
// static constexpr uint8_t BRIGHTNESS = 24;

// static uint8_t Scale(uint8_t v)
// {
//     return static_cast<uint8_t>((static_cast<uint16_t>(v) * BRIGHTNESS) / 255);
// }

// extern "C" void app_main(void)
// {
//     ESP_LOGI(TAG, "WS2812 test start");

//     led_strip_handle_t led_strip = nullptr;

//     led_strip_config_t strip_config = {
//         .strip_gpio_num = WS2812_GPIO,
//         .max_leds = WS2812_LED_COUNT,
//         .led_model = LED_MODEL_WS2812,
//         .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
//         .flags = {
//             .invert_out = false,
//         },
//     };

//     led_strip_rmt_config_t rmt_config = {
//         .clk_src = RMT_CLK_SRC_DEFAULT,
//         .resolution_hz = 10 * 1000 * 1000,
//         .mem_block_symbols = 64,
//         .flags = {
//             .with_dma = false,
//         },
//     };

//     ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
//     ESP_ERROR_CHECK(led_strip_clear(led_strip));

//     while (true)
//     {
//         // RED
//         ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, Scale(255), Scale(0),   Scale(0)));
//         ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 1, Scale(255), Scale(0),   Scale(0)));
//         ESP_ERROR_CHECK(led_strip_refresh(led_strip));
//         vTaskDelay(pdMS_TO_TICKS(500));

//         // GREEN
//         ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, Scale(0),   Scale(255), Scale(0)));
//         ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 1, Scale(0),   Scale(255), Scale(0)));
//         ESP_ERROR_CHECK(led_strip_refresh(led_strip));
//         vTaskDelay(pdMS_TO_TICKS(500));

//         // BLUE
//         ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, Scale(0),   Scale(0),   Scale(255)));
//         ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 1, Scale(0),   Scale(0),   Scale(255)));
//         ESP_ERROR_CHECK(led_strip_refresh(led_strip));
//         vTaskDelay(pdMS_TO_TICKS(500));

//         // OFF
//         ESP_ERROR_CHECK(led_strip_clear(led_strip));
//         vTaskDelay(pdMS_TO_TICKS(300));
//     }
// }

