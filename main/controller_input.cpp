#include "controller_input.h"

#include "esp_log.h"

static const char* TAG = "ControllerInput";

ControllerInput::ControllerInput(Ps2Pad& pad)
    : pad_(pad)
{
}

esp_err_t ControllerInput::Initialize()
{
    read_idx_.store(0);
    write_idx_.store(1);

    state_buf_[0] = Ps2Pad::PadState{};
    state_buf_[1] = Ps2Pad::PadState{};

    task_stop_.store(false);

    return this->StartTask();
}

esp_err_t ControllerInput::StartTask()
{
    esp_err_t ret = xTaskCreatePinnedToCore(
        MainTask,
        "controller_input_task",
        4096,
        this,
        5,
        &this->handle_,
        0
    );

    if (ret == 1)
    {
       ret = ESP_OK;
    }
    else 
    {
        ret = ESP_FAIL;
    }

    return ret;
}

void ControllerInput::StopTaskRequest()
{
    task_stop_.store(true);

    TaskHandle_t handle = this->handle_;
    if (handle != nullptr)
    {
        xTaskNotifyGive(handle);
    }
}

bool ControllerInput::GetLatestPadState(Ps2Pad::PadState& out) const
{
    const int idx = read_idx_.load();
    out = state_buf_[idx];
    return out.valid;
}

void ControllerInput::MainTask(void* param)
{
    auto* self = static_cast<ControllerInput*>(param);
    if (self == nullptr)
    {
        vTaskDelete(nullptr);
        return;
    }

    self->MainLoop();

    self->handle_ = nullptr;
    vTaskDelete(nullptr);
}

void ControllerInput::MainLoop()
{
    TickType_t last = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(this->TASK_PERIOD_MS); 

    while (true)
    {
        if (task_stop_.load())
            break;

        Ps2Pad::PadState st{};
        const esp_err_t ret = pad_.Read(st);

        if (ret == ESP_OK && st.valid)
        {
            const int w = write_idx_.load();
            state_buf_[w] = st;
            read_idx_.store(w);
            write_idx_.store(1 - w);
        }
        else
        {
            // ESP_LOGW(TAG, "Pad read failed");
        }

        vTaskDelayUntil(&last, period);
    }
}