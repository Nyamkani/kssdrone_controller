#pragma once

#include <atomic>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"

#include "ps2_pad.h"

class ControllerInput
{
public:
    explicit ControllerInput(Ps2Pad& pad);
    ~ControllerInput() = default;

    esp_err_t Initialize();

    esp_err_t StartTask();
    void StopTaskRequest();

    bool GetLatestPadState(Ps2Pad::PadState& out) const;

private:
    static void MainTask(void* param);
    void MainLoop();

private:
    static constexpr uint32_t TASK_PERIOD_MS = 10; // 100Hz

private:
    Ps2Pad& pad_;

    TaskHandle_t handle_ = nullptr;
    std::atomic<bool> task_stop_{false};

    std::atomic<int> read_idx_{0};
    std::atomic<int> write_idx_{1};

    Ps2Pad::PadState state_buf_[2] {};
};