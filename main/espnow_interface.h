#pragma once

#include <atomic>
#include <cstdint>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_now.h"
#include "esp_timer.h"

#include "packets.h"

struct ControlState
{
    float throttle = 0.0f;
    float roll_rad = 0.0f;
    float pitch_rad = 0.0f;
    float yaw_rate_rad = 0.0f;
    uint8_t armed = 0;
    uint8_t mode = 0;
};

class EspNowInterface
{
public:
    enum class ESPNOWState : uint8_t
    {
        INIT = 0,
        RUN = 1,
        // RUN_STABLE = 1,
        // RUN_UNSTABLE = 2,
        // RUN_DISCONNECTED = 3,
        STOP = 4,
        ERR = 10,
    };

public:
    EspNowInterface() = default;
    ~EspNowInterface() = default;

    esp_err_t Initialize();

    esp_err_t StartTask();
    void StopTaskRequest();

    bool SetControlCommand(const ControlPacket& cmd);
    bool GetLatestCommand(ControlPacket& out) const;
    bool GetLatestTelemetry(TelemetryPacket& out) const;

    bool IsPeerConfigured() const;
    bool GetLastSendOk() const;

    private:
    
        static void EspNowSendCb(const wifi_tx_info_t* tx_info, esp_now_send_status_t status);
        // static void OnSendDone(esp_now_send_status_t status);

        static void EspNowRecvCb(const esp_now_recv_info_t* recv_info, const uint8_t* data, int len);
        static void MainTask(void* param);

        void MainLoop();

        esp_err_t Init();
        esp_err_t Run();
        esp_err_t Error();

        bool IsMacValid(const uint8_t mac[6]) const;

    private:
        struct SendEvent
        {
            esp_now_send_status_t status = ESP_NOW_SEND_FAIL;
            uint8_t mac_addr[6] = {0,};
        };

    private:
        static constexpr uint32_t MAGIC = 0xA5A5A5A5;
        static constexpr uint32_t TX_QUEUE_SIZE = 1;
        static constexpr uint32_t RX_QUEUE_SIZE = 1;
        // static constexpr uint32_t SEND_PERIOD_MS = 1000;   // 1s
        static constexpr uint8_t  ESPNOW_CHANNEL = 1;

        uint32_t TASK_PERIOD_MS = 5;   // 5ms (200Hz)

        // 드론(수신기)의 MAC 주소로 수정
        static constexpr uint8_t DRONE_MAC[6] = {0x1C, 0xDB, 0xD4, 0x76, 0x1c, 0x14};


    private:
        static EspNowInterface* instance_;

        TaskHandle_t handle_ = nullptr;
        // QueueHandle_t tx_evt_queue_ = nullptr;
        QueueHandle_t rx_telemetry_queue_ = nullptr;

        std::atomic<bool> task_stop_{false};
        std::atomic<bool> last_send_ok_{false};

        std::atomic<int> read_idx_{0};
        std::atomic<int> write_idx_{1};

        ControlPacket cmd_buf_[2] {};

        ESPNOWState state_ = ESPNOWState::INIT;

        bool hw_initialized_ = false;

        // link status
        bool link_lost = false;
        uint8_t send_fail_count_ = 0;
        uint32_t last_retry_ms_ = 0;


        std::atomic<bool> tx_pending_{false};
        std::atomic<bool> send_done_{false};
        std::atomic<bool> send_status_ok_{false};

        uint16_t seq_ = 0;
        uint16_t tx_pending_seq_ = 0;

        uint32_t last_send_start_ms_ = 0;

        static constexpr uint32_t TX_PENDING_TIMEOUT_MS = 50;

};