#include "kss_controller.h"

#include <cmath>

#include "esp_log.h"
#include "esp_timer.h"

static const char* TAG = "KssController";

//////////////////////////////////////////////////////////////////////////
// Constructor
//////////////////////////////////////////////////////////////////////////

KssController::KssController(
    gpio_num_t pin_att,
    gpio_num_t pin_cmd,
    gpio_num_t pin_dat,
    gpio_num_t pin_clk)
    : pad_(pin_att, pin_cmd, pin_dat, pin_clk),
      controller_input_(pad_),
      espnow_()
{
}

//////////////////////////////////////////////////////////////////////////
// Public
//////////////////////////////////////////////////////////////////////////

esp_err_t KssController::Initialize()
{
    StartTask();
    return ESP_OK;
}

void KssController::StartTask()
{
    if (handle_ != nullptr)
        return;

    task_stop_.store(false);
    state_ = ControllerState::INIT;

    xTaskCreatePinnedToCore(
        MainTask,
        "kss_controller",
        4096 * 3,
        this,
        5,
        &handle_,
        0
    );
}

void KssController::StopTaskRequest()
{
    task_stop_.store(true);

    controller_input_.StopTaskRequest();
    espnow_.StopTaskRequest();
}

bool KssController::GetLatestTelemetry(TelemetryPacket& out) const
{
    out = tpkt_;
    return true;
}

//////////////////////////////////////////////////////////////////////////
// Task
//////////////////////////////////////////////////////////////////////////

void KssController::MainTask(void* param)
{
    auto* self = static_cast<KssController*>(param);
    if (self == nullptr)
    {
        vTaskDelete(nullptr);
        return;
    }

    self->MainLoop();

    self->handle_ = nullptr;
    vTaskDelete(nullptr);
}

void KssController::MainLoop()
{
    TickType_t last = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(TASK_PERIOD_MS);

    while (true)
    {
        if (task_stop_.load())
            state_ = ControllerState::STOP;

        switch (state_)
        {
            case ControllerState::INIT:
                if (Init() == ESP_OK)
                    state_ = ControllerState::RUN;
                else
                    state_ = ControllerState::ERR;
                break;

            case ControllerState::RUN:
                if (Run() != ESP_OK)
                    state_ = ControllerState::ERR;
                break;

            case ControllerState::STOP:
                Stop();
                return;

            case ControllerState::ERR:
                Error();
                break;

            default:
                break;
        }

        vTaskDelayUntil(&last, period);
    }
}

//////////////////////////////////////////////////////////////////////////
// Init / Run
//////////////////////////////////////////////////////////////////////////

esp_err_t KssController::Init()
{
    if (pad_.Initialize() != ESP_OK)
        return ESP_FAIL;

    if (controller_input_.Initialize() != ESP_OK)
        return ESP_FAIL;

    if (espnow_.Initialize() != ESP_OK)
        return ESP_FAIL;

    if (led_strip_ == nullptr)
    {
        InitializeWs2812();
    }

    armed_ = false;
    mode_ = DroneMode::RATE_ACRO;
    seq_ = 0;
    command_seq_ = 0;
    pending_command_flags_ = CMD_NONE;
    command_repeat_count_ = 0;

    prev_buttons_ = 0xFFFF;
    press_start_ms_ = 0;

    send_fail_count_ = 0;
    success_count_ = 0;
    link_lost_ = false;

    throttle_state_ = 0.0f;
    throttle_f_ = 0.0f;
    last_telemetry_ms_ = 0;

    ResetAxisStates();
    ClearLeds();

    ESP_LOGI(TAG, "Initialized");

    return ESP_OK;
}

esp_err_t KssController::Run()
{
    Ps2Pad::PadState st{};

    if (controller_input_.GetLatestPadState(st))
    {
        HandleButtons(st);

        float yaw_in = 0.0f;
        float roll_in = 0.0f;
        float pitch_in = 0.0f;
        float throttle = 0.0f;

        if (link_lost_)
        {
            throttle_state_ = 0.0f;
            throttle_f_ = 0.0f;
            ResetAxisStates();
        }
        else
        {
            const float dt = static_cast<float>(TASK_PERIOD_MS) / 1000.0f;

            // yaw / roll / pitch: release -> 0 복귀
            yaw_in = SlewLimit(
                Filter(MapAxis(st.lx, EXPO_YAW), lx_f_, STICK_ALPHA),
                lx_s_, STICK_RATE, dt);

            roll_in = SlewLimit(
                Filter(MapAxis(st.rx, EXPO_ROLL), rx_f_, STICK_ALPHA),
                rx_s_, STICK_RATE, dt);

            pitch_in = SlewLimit(
                Filter(MapAxis(st.ry, EXPO_PITCH), ry_f_, STICK_ALPHA),
                ry_s_, STICK_RATE, dt);

            // throttle: 누적형
            const float throttle_input =
                Filter(MapThrottleInput(st.ly), throttle_f_, THROTTLE_ALPHA);

            throttle = UpdateThrottle(throttle_input, dt);
        }

        ControlPacket pkt = BuildPacket(yaw_in, throttle, roll_in, pitch_in);
        // pkt.seq = seq_++;

        espnow_.SetControlCommand(pkt);

        if (command_repeat_count_ > 0)
        {
            --command_repeat_count_;
            if (command_repeat_count_ == 0)
            {
                pending_command_flags_ = CMD_NONE;
            }
        }

        TelemetryPacket tpkt{};
        if (espnow_.GetLatestTelemetry(tpkt))
        {
            if (tpkt.magic == 0xA5A5A5A5)
            {
                tpkt_ = tpkt;
                last_telemetry_ms_ = esp_timer_get_time() / 1000ULL;

                const DroneState dstate = static_cast<DroneState>(tpkt_.state);
                armed_ = (dstate == DroneState::ARMED ||
                        dstate == DroneState::LANDING);
            }
        }
    }

    const bool send_ok = espnow_.GetLastSendOk();

    if (send_ok)
    {
        send_fail_count_ = 0;
        ++success_count_;

        if (link_lost_ && success_count_ >= 5)
        {
            link_lost_ = false;
            ESP_LOGI(TAG, "LINK RECOVERED");
        }
    }
    else
    {
        ++send_fail_count_;
        success_count_ = 0;

        if (!link_lost_ && send_fail_count_ >= FAIL_THRESHOLD)
        {
            link_lost_ = true;
            armed_ = false;
            throttle_state_ = 0.0f;
            throttle_f_ = 0.0f;
            ResetAxisStates();
            ESP_LOGI(TAG, "LINK LOST");
        }
    }

    // WS2812 update: 20Hz 제한
    {
        static uint32_t last_led_update_ms = 0;
        const uint32_t now_ms = esp_timer_get_time() / 1000ULL;
        if (now_ms - last_led_update_ms >= 50)
        {
            UpdateWs2812();
            last_led_update_ms = now_ms;
        }
    }

    //test log: 10Hz 제한
    {
        TelemetryPacket log = this->tpkt_;

        if(log.state == static_cast<uint8_t>(DroneState::ARMED))
        {
            static uint32_t last_log_update_ms = 0;
            const uint32_t now_ms = esp_timer_get_time() / 1000ULL;
            if (now_ms - last_log_update_ms >= 100)
            {
                // ESP_LOGI(TAG, "Throttle_cmd: %.3f " ,log.throttle);


                // ESP_LOGI(TAG, "Throttle_cmd: %.3f | d: %.3f %.3f %.3f %.3f | m: %.3f %.3f %.3f %.3f",
                //                                     log.throttle_cmd,
                //                                     log.dshot1,
                //                                     log.dshot2,
                //                                     log.dshot3,
                //                                     log.dshot4,
                //                                     log.m1,
                //                                     log.m2,
                //                                     log.m3,
                //                                     log.m4);

                //must set time 300ms
                // ESP_LOGI(TAG, "Throttle_cmd: %.3f | state: %.3f %.3f | gz: %.3f |pid_out_roll: %.3f, pid_out_pitch: %.3f, pid_out_yaw: %.3f |mix_in_roll: %.3f, mix_in_pitch: %.3f, mix_in_yaw: %.3f |  m: %.3f %.3f %.3f %.3f",
                //                                     log.throttle_cmd,
                //                                     log.roll_rad,
                //                                     log.pitch_rad,
                //                                     log.gyro_z_rad_s,
                //                                     log.pid_out_roll,
                //                                     log.pid_out_pitch,
                //                                     log.pid_out_yaw,
                //                                     log.mix_in_roll,
                //                                     log.mix_in_pitch,
                //                                     log.mix_in_yaw,
                //                                     log.m1,
                //                                     log.m2,
                //                                     log.m3,
                //                                     log.m4);

                // ESP_LOGI(TAG, "Throttle_cmd: %.3f | disarm_reason: %u | state: %.3f %.3f | gz: %.3f |pid_out_roll: %.3f, pid_out_pitch: %.3f, pid_out_yaw: %.3f | m: %.3f %.3f %.3f %.3f",
                //                                     log.throttle_cmd,
                //                                     static_cast<unsigned int>(log.debug_disarm_reason),
                //                                     log.roll_rad,
                //                                     log.pitch_rad,
                //                                     log.gyro_z_rad_s,
                //                                     log.pid_out_roll,
                //                                     log.pid_out_pitch,
                //                                     log.pid_out_yaw,
                //                                     log.m1,
                //                                     log.m2,
                //                                     log.m3,
                //                                     log.m4);

                last_log_update_ms = now_ms;
            }
        }
    }


    return ESP_OK;
}

//////////////////////////////////////////////////////////////////////////
// Button Handling
//////////////////////////////////////////////////////////////////////////

KssController::ButtonEdge
KssController::UpdateButton(uint16_t buttons,
                            uint16_t prev,
                            uint16_t mask) const
{
    const bool curr = ((buttons & mask) == 0);
    const bool prev_b = ((prev & mask) == 0);

    return {
        curr,
        (curr && !prev_b),
        (!curr && prev_b)
    };
}

void KssController::HandleButtons(const Ps2Pad::PadState& st)
{
    const uint32_t now_ms = esp_timer_get_time() / 1000ULL;

    const auto start  = UpdateButton(st.buttons, prev_buttons_, BTN_START_MASK);
    const auto select = UpdateButton(st.buttons, prev_buttons_, BTN_SELECT_MASK);
    const auto l2     = UpdateButton(st.buttons, prev_buttons_, BTN_L2_MASK);
    const auto l1     = UpdateButton(st.buttons, prev_buttons_, BTN_L1_MASK);
    const auto r1     = UpdateButton(st.buttons, prev_buttons_, BTN_R1_MASK);
    const auto up     = UpdateButton(st.buttons, prev_buttons_, BTN_UP_MASK);
    const auto down   = UpdateButton(st.buttons, prev_buttons_, BTN_DOWN_MASK);

    // soft landing
    if (l1.rising)
    {
        if (armed_)
        {
            pending_command_flags_ = CMD_SOFT_LANDING;
            command_repeat_count_ = COMMAND_REPEAT_FRAMES;
            ++command_seq_;

            ESP_LOGI(TAG, "Soft Landing request");
        }
    }

    // emergency stop
    if (l2.rising)
    {
        throttle_state_ = 0.0f;
        throttle_f_ = 0.0f;
        ResetAxisStates();

        pending_command_flags_ = CMD_EMERGENCY_STOP;
        command_repeat_count_ = COMMAND_REPEAT_FRAMES;
        ++command_seq_;

        ESP_LOGW(TAG, "EMERGENCY STOP REQUEST");
    }

    // mode change request
    if (select.rising)
    {
        if (mode_ == DroneMode::RATE_ACRO)
            mode_ = DroneMode::ANGLE_SELF_LEVEL;
        else
            mode_ = DroneMode::RATE_ACRO;

        pending_command_flags_ = CMD_SET_MODE;
        command_repeat_count_ = COMMAND_REPEAT_FRAMES;
        ++command_seq_;

        ESP_LOGI(TAG, "Mode request -> %u", static_cast<unsigned>(mode_));
    }

    // level calibration
    if (r1.rising)
    {
        pending_command_flags_ = CMD_LEVEL_CALIBRATE;
        command_repeat_count_ = COMMAND_REPEAT_FRAMES;
        ++command_seq_;

        ESP_LOGI(TAG, "LEVEL CALIBRATE REQUEST");
    }

    // arm / disarm request
    if (start.rising)
    {
        press_start_ms_ = now_ms;
    }

    if (start.falling)
    {
        const uint32_t held = now_ms - press_start_ms_;

        if (held >= ARM_HOLD_MS)
        {
            if (!armed_)
            {
                // ARM 요청은 throttle/stick 조건 만족 시에만 보냄
                if (throttle_state_ < ARM_THROTTLE_MAX &&
                    std::fabs(lx_s_) < ARM_STICK_EPS &&
                    std::fabs(rx_s_) < ARM_STICK_EPS &&
                    std::fabs(ry_s_) < ARM_STICK_EPS)
                {
                    pending_command_flags_ = CMD_ARM_REQUEST;
                    command_repeat_count_ = COMMAND_REPEAT_FRAMES;
                    ++command_seq_;

                    ESP_LOGI(TAG, "ARM REQUEST");
                }
                else
                {
                    ESP_LOGW(TAG, "ARM request rejected (stick/throttle not safe)");
                }
            }
            else
            {
                // DISARM 요청
                pending_command_flags_ = CMD_DISARM_REQUEST;
                command_repeat_count_ = COMMAND_REPEAT_FRAMES;
                ++command_seq_;

                throttle_state_ = 0.0f;
                throttle_f_ = 0.0f;
                ResetAxisStates();

                ESP_LOGI(TAG, "DISARM REQUEST");
            }
        }
    }

    // fine throttle up
    if (up.rising && armed_)
    {
        throttle_state_ += THROTTLE_FINE_STEP;
        if (throttle_state_ > 1.0f)
            throttle_state_ = 1.0f;
    }

    // fine throttle down
    if (down.rising)
    {
        throttle_state_ -= THROTTLE_FINE_STEP;
        if (throttle_state_ < 0.0f)
            throttle_state_ = 0.0f;
    }

    prev_buttons_ = st.buttons;
}

//////////////////////////////////////////////////////////////////////////
// Mapping
//////////////////////////////////////////////////////////////////////////

float KssController::MapAxis(uint8_t raw, float expo) const
{
    int v = static_cast<int>(raw) - 128;

    if (std::abs(v) < AXIS_DEAD_BAND)
        v = 0;
    else
        v = (v > 0) ? (v - AXIS_DEAD_BAND) : (v + AXIS_DEAD_BAND);

    float x = static_cast<float>(v) / 127.0f;
    x = x * (1.0f - expo) + x * x * x * expo;

    return x;
}

float KssController::MapThrottleInput(uint8_t raw) const
{
    int v = 128 - static_cast<int>(raw);

    if (std::abs(v) < THROTTLE_INPUT_DEAD_BAND)
        v = 0;
    else
        v = (v > 0)
            ? (v - THROTTLE_INPUT_DEAD_BAND)
            : (v + THROTTLE_INPUT_DEAD_BAND);

    float x = static_cast<float>(v) / 127.0f;

    if (x < -1.0f) x = -1.0f;
    if (x >  1.0f) x =  1.0f;

    return x;
}

float KssController::UpdateThrottle(float input, float dt)
{
    throttle_state_ += input * THROTTLE_ACC_RATE * dt;

    if (throttle_state_ < 0.0f)
        throttle_state_ = 0.0f;
    if (throttle_state_ > 1.0f)
        throttle_state_ = 1.0f;

    return throttle_state_;
}

//////////////////////////////////////////////////////////////////////////
// Packet
//////////////////////////////////////////////////////////////////////////

ControlPacket KssController::BuildPacket(float yaw_in,
                                         float throttle,
                                         float roll_in,
                                         float pitch_in) const
{
    ControlPacket pkt{};

    pkt.magic = 0xA5A5A5A5;
    pkt.mode = mode_;
    pkt.cmd_flags = pending_command_flags_;
    pkt.cmd_seq = command_seq_;

    pkt.throttle = throttle;
    pkt.yaw_rate_rad_s = yaw_in * MAX_YAW_RATE;
    pkt.pitch_rad = -pitch_in * MAX_ANGLE;
    pkt.roll_rad = roll_in * MAX_ANGLE;

    return pkt;
}

//////////////////////////////////////////////////////////////////////////
// LPF / Slew
//////////////////////////////////////////////////////////////////////////

float KssController::Filter(float input, float& state, float alpha) const
{
    state = alpha * state + (1.0f - alpha) * input;
    return state;
}

float KssController::SlewLimit(float target, float& state, float rate, float dt) const
{
    float diff = target - state;
    const float max_delta = rate * dt;

    if (diff > max_delta)
        diff = max_delta;
    if (diff < -max_delta)
        diff = -max_delta;

    state += diff;
    return state;
}

void KssController::ResetAxisStates()
{
    lx_f_ = 0.0f;
    rx_f_ = 0.0f;
    ry_f_ = 0.0f;

    lx_s_ = 0.0f;
    rx_s_ = 0.0f;
    ry_s_ = 0.0f;
}

//////////////////////////////////////////////////////////////////////////
// WS2812
//////////////////////////////////////////////////////////////////////////

void KssController::InitializeWs2812()
{
    led_strip_config_t strip_config = {};
    strip_config.strip_gpio_num = WS2812_GPIO;
    strip_config.max_leds = WS2812_LED_COUNT;
    strip_config.led_model = LED_MODEL_WS2812;
    strip_config.color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB;
    strip_config.flags.invert_out = false;

    led_strip_rmt_config_t rmt_config = {};
    rmt_config.clk_src = RMT_CLK_SRC_DEFAULT;
    rmt_config.resolution_hz = 10 * 1000 * 1000;
    rmt_config.mem_block_symbols = 64;
    rmt_config.flags.with_dma = false;

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip_));
    ESP_ERROR_CHECK(led_strip_clear(led_strip_));
}

void KssController::SetLedPixel(uint32_t index, uint8_t r, uint8_t g, uint8_t b)
{
    if (led_strip_ == nullptr || index >= WS2812_LED_COUNT)
        return;

    auto scale = [](uint8_t v) -> uint8_t
    {
        return static_cast<uint8_t>((static_cast<uint16_t>(v) * WS2812_BRIGHTNESS) / 255);
    };

    led_strip_set_pixel(led_strip_, index, scale(r), scale(g), scale(b));
}

void KssController::ClearLeds()
{
    if (led_strip_ == nullptr)
        return;

    led_strip_clear(led_strip_);
}

void KssController::UpdateWs2812()
{
    if (led_strip_ == nullptr)
        return;

    const uint32_t now_ms = esp_timer_get_time() / 1000ULL;
    const bool telemetry_ok =
        (last_telemetry_ms_ != 0U) &&
        ((now_ms - last_telemetry_ms_) < TELEMETRY_TIMEOUT_MS);

    const bool slow_blink = ((now_ms / 500U) % 2U) != 0U;
    const bool fast_blink = ((now_ms / 150U) % 2U) != 0U;

    SetLedPixel(0, 0, 0, 0);
    SetLedPixel(1, 0, 0, 0);

    if (!telemetry_ok)
    {
        if (fast_blink)
            SetLedPixel(0, 255, 0, 0);   // red blink
        else
            SetLedPixel(0, 0, 0, 0);

        SetLedPixel(1, 0, 0, 0);
        led_strip_refresh(led_strip_);
        return;
    }

    const DroneState dstate = static_cast<DroneState>(tpkt_.state);
    const DroneMode dmode = static_cast<DroneMode>(tpkt_.mode);

    // LED0: DroneState
    switch (dstate)
    {
        case DroneState::INIT:
            if (slow_blink) SetLedPixel(0, 255, 255, 255);
            break;

        case DroneState::IMU_BIAS_CALIBRATING:
            if (slow_blink) SetLedPixel(0, 255, 180, 0);
            break;

        case DroneState::DISARMED:
            SetLedPixel(0, 255, 180, 0);
            break;

        case DroneState::ARMING:
            if (slow_blink) SetLedPixel(0, 0, 255, 0);
            break;

        case DroneState::ARMED:
            SetLedPixel(0, 0, 255, 0);
            break;

        case DroneState::LANDING:
            if (slow_blink) SetLedPixel(0, 180, 0, 255);
            break;

        case DroneState::ERR:
            SetLedPixel(0, 255, 0, 0);
            break;

        default:
            SetLedPixel(0, 20, 20, 20);
            break;
    }

    // LED1: DroneMode
    switch (dmode)
    {
        case DroneMode::RATE_ACRO:
            SetLedPixel(1, 0, 0, 255);      // blue
            break;

        case DroneMode::ANGLE_SELF_LEVEL:
            SetLedPixel(1, 0, 180, 255);    // cyan
            break;

        default:
            SetLedPixel(1, 20, 20, 20);
            break;
    }

    led_strip_refresh(led_strip_);
}

//////////////////////////////////////////////////////////////////////////
// Error / Stop
//////////////////////////////////////////////////////////////////////////

esp_err_t KssController::Stop()
{
    return ESP_OK;
}

esp_err_t KssController::Error()
{
    vTaskDelay(pdMS_TO_TICKS(500));
    state_ = ControllerState::INIT;
    return ESP_OK;
}