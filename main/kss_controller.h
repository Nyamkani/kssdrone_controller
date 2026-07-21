#pragma once

#include <atomic>
#include <cstdint>
#include <cmath>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"

#include "ps2_pad.h"
#include "controller_input.h"
#include "espnow_interface.h"
#include "packets.h"

#include "led_strip.h"


#define DEG2RAD(x) ((x) * 0.01745329252f)
#define RAD2DEG(x) ((x) * 57.2957795131f)`

class KssController
{
public:
    enum class ControllerState : uint8_t
    {
        INIT = 0,
        RUN,
        STOP,
        ERR
    };

    struct ButtonEdge
    {
        bool pressed;
        bool rising;
        bool falling;
    };

public:
    KssController(
        gpio_num_t pin_att,
        gpio_num_t pin_cmd,
        gpio_num_t pin_dat,
        gpio_num_t pin_clk);

    ~KssController() = default;

    esp_err_t Initialize();

    void StartTask();
    void StopTaskRequest();

    bool GetLatestTelemetry(TelemetryPacket& out) const;

private:
    static void MainTask(void* param);
    void MainLoop();

    esp_err_t Init();
    esp_err_t Run();
    esp_err_t Stop();
    esp_err_t Error();

    ButtonEdge UpdateButton(uint16_t buttons,
                            uint16_t prev,
                            uint16_t mask) const;
    void HandleButtons(const Ps2Pad::PadState& st);

    float MapAxis(uint8_t raw, float expo) const;

    // throttle용: 순간값이 아니라 증가/감소 입력으로 해석
    float MapThrottleInput(uint8_t raw) const;
    float UpdateThrottle(float input, float dt);

    ControlPacket BuildPacket(float yaw_in,
                              float throttle,
                              float roll_in,
                              float pitch_in) const;

    float Filter(float input, float& state, float alpha) const;
    float SlewLimit(float target, float& state, float rate, float dt) const;

    void ResetAxisStates();

    // ---------- WS2812 ----------
    void InitializeWs2812();
    void UpdateWs2812();
    void SetLedPixel(uint32_t index, uint8_t r, uint8_t g, uint8_t b);
    void ClearLeds();

private:
    // ---------- Task / Loop ----------
    static constexpr uint32_t TASK_PERIOD_MS = 10;
    static constexpr uint32_t FAIL_THRESHOLD = 10;

    // ---------- Button Masks ----------
    static constexpr uint16_t BTN_SELECT_MASK   = (1u << 0);
    static constexpr uint16_t BTN_L3_MASK       = (1u << 1);
    static constexpr uint16_t BTN_R3_MASK       = (1u << 2);
    static constexpr uint16_t BTN_START_MASK    = (1u << 3);
    static constexpr uint16_t BTN_UP_MASK       = (1u << 4);
    static constexpr uint16_t BTN_RIGHT_MASK    = (1u << 5);
    static constexpr uint16_t BTN_DOWN_MASK     = (1u << 6);
    static constexpr uint16_t BTN_LEFT_MASK     = (1u << 7);
    static constexpr uint16_t BTN_L2_MASK       = (1u << 8);
    static constexpr uint16_t BTN_R2_MASK       = (1u << 9);
    static constexpr uint16_t BTN_L1_MASK       = (1u << 10);
    static constexpr uint16_t BTN_R1_MASK       = (1u << 11);
    static constexpr uint16_t BTN_TRIANGLE_MASK = (1u << 12);
    static constexpr uint16_t BTN_CIRCLE_MASK   = (1u << 13);
    static constexpr uint16_t BTN_CROSS_MASK    = (1u << 14);
    static constexpr uint16_t BTN_SQUARE_MASK   = (1u << 15);

    // ---------- Input Tuning ----------
    static constexpr float EXPO_ROLL  = 0.4f;
    static constexpr float EXPO_PITCH = 0.4f;
    static constexpr float EXPO_YAW   = 0.2f;

    static constexpr int AXIS_DEAD_BAND = 5;
    static constexpr int THROTTLE_INPUT_DEAD_BAND = 8;

    static constexpr float STICK_ALPHA = 0.7f;
    static constexpr float THROTTLE_ALPHA = 0.5f;

    static constexpr float STICK_RATE = 4.0f;
    static constexpr float THROTTLE_RATE = 2.0f;

    // throttle 누적 속도
    static constexpr float THROTTLE_ACC_RATE = 0.4f; //0.8
    static constexpr float THROTTLE_FINE_STEP = 0.005f;

    // ---------- Command Scale ----------
    static constexpr float MAX_ANGLE = DEG2RAD(30.0f);//30.0f-> outer //for inner house
    static constexpr float MAX_YAW_RATE = DEG2RAD(60.0f);//60.0f;-> outer //for inner house

    // ---------- Arm ----------
    static constexpr uint32_t ARM_HOLD_MS = 500;
    static constexpr float ARM_THROTTLE_MAX = 0.05f;
    static constexpr float ARM_STICK_EPS = 0.2f;

    // ---------- WS2812 ----------
    static constexpr gpio_num_t WS2812_GPIO = GPIO_NUM_7;
    static constexpr uint32_t WS2812_LED_COUNT = 2;
    static constexpr uint8_t WS2812_BRIGHTNESS = 24;
    static constexpr uint32_t TELEMETRY_TIMEOUT_MS = 300;

    led_strip_handle_t led_strip_ = nullptr;
    uint32_t last_telemetry_ms_ = 0;

private:
    // ---------- Task ----------
    TaskHandle_t handle_ = nullptr;
    std::atomic<bool> task_stop_{false};
    ControllerState state_ = ControllerState::INIT;

    // ---------- Modules ----------
    Ps2Pad pad_;
    ControllerInput controller_input_;
    EspNowInterface espnow_;

    // ---------- Telemetry ----------
    TelemetryPacket tpkt_{};

    // ---------- Button State ----------
    uint16_t prev_buttons_ = 0xFFFF;
    uint32_t press_start_ms_ = 0;

    // ---------- Control State ----------
    bool armed_ = false;
    DroneMode mode_ = DroneMode::RATE_ACRO;
    uint16_t seq_ = 0;
    uint8_t pending_command_flags_ = CMD_NONE;
    uint8_t command_repeat_count_ = 0;
    uint8_t command_seq_ = 0;
    static constexpr uint8_t COMMAND_REPEAT_FRAMES = 8;

    // ---------- Link State ----------
    uint32_t send_fail_count_ = 0;
    uint32_t success_count_ = 0;
    bool link_lost_ = false;

    // ---------- Axis LPF State ----------
    float lx_f_ = 0.0f;   // yaw input filter
    float rx_f_ = 0.0f;   // roll input filter
    float ry_f_ = 0.0f;   // pitch input filter
    float throttle_f_ = 0.0f; // throttle input filter

    // ---------- Axis Slew State ----------
    float lx_s_ = 0.0f;
    float rx_s_ = 0.0f;
    float ry_s_ = 0.0f;

    // ---------- Throttle State ----------
    float throttle_state_ = 0.0f;

    //-------------test----------------
    float test_log_dt_ = 0.0f;

};