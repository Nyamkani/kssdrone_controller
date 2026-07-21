#include "espnow_interface.h"

#include <cstring>

/* ESP-IDF */
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

EspNowInterface* EspNowInterface::instance_ = nullptr;

constexpr uint8_t EspNowInterface::DRONE_MAC[6];

//////////////////////////////////////////////////////////////////////////

// void EspNowInterface::EspNowSendCb(const wifi_tx_info_t* tx_info, esp_now_send_status_t status)
// {
//     if (instance_ == nullptr)
//         return;

//     if (instance_->tx_evt_queue_ != nullptr)
//     {
//         SendEvent evt{};
//         evt.status = status;

//         if (tx_info != nullptr)
//         {
//             std::memcpy(evt.mac_addr, tx_info->des_addr, 6);
//         }

//         xQueueSend(instance_->tx_evt_queue_, &evt, 0);
//     }
// }

// void EspNowInterface::OnSendDone(esp_now_send_status_t status)
// {
//     const bool ok = (status == ESP_NOW_SEND_SUCCESS);

//     last_send_ok_.store(ok);
//     tx_pending_.store(false);

//     if (ok)
//     {
//         send_fail_count_ = 0;
//         link_lost = false;
//     }
//     else
//     {
//         ++send_fail_count_;
//         if (send_fail_count_ >= 5)
//             link_lost = true;
//     }
// }

void EspNowInterface::EspNowSendCb(const wifi_tx_info_t* tx_info,
                                   esp_now_send_status_t status)
{
    if (instance_ == nullptr)
        return;

    instance_->send_status_ok_.store(status == ESP_NOW_SEND_SUCCESS);
    instance_->send_done_.store(true);
    instance_->tx_pending_.store(false);
}


void EspNowInterface::EspNowRecvCb(const esp_now_recv_info_t* recv_info,
                                   const uint8_t* data,
                                   int len)
{
    if (instance_ == nullptr || data == nullptr || len < sizeof(PacketType))
        return;

    PacketType type;
    std::memcpy(&type, data, sizeof(PacketType));

    switch (type)
    {
        case PacketType::CONTROL:
        {
            // if(len != sizeof(ControlPacket))
            //     return; // Invalid packet length for control packet, ignore it.
            break;
        }
        case PacketType::TELEMETRY:
        {
                if(len != sizeof(TelemetryPacket))
                    return; // Invalid packet length for control packet, ignore it.

                TelemetryPacket pkt{};
                std::memcpy(&pkt, data, sizeof(TelemetryPacket));
                
                // Handle control packet
                if (instance_->rx_telemetry_queue_ != nullptr)
                {
                    // callback has to be fast, so we just push the raw packet to the queue and let the task handle it.
                    xQueueOverwrite(instance_->rx_telemetry_queue_, &pkt);
                }


            break;
        }

        default:
            break;
    }
}

bool EspNowInterface::GetLatestTelemetry(TelemetryPacket& out) const
{
    return xQueueReceive(this->rx_telemetry_queue_, &out, 0) == pdTRUE;
}


//////////////////////////////////////////////////////////////////////////

bool EspNowInterface::IsMacValid(const uint8_t mac[6]) const
{
    for (int i = 0; i < 6; ++i)
    {
        if (mac[i] != 0x00)
            return true;
    }
    return false;
}

bool EspNowInterface::SetControlCommand(const ControlPacket& cmd)
{
    const int w = this->write_idx_.load();

    ControlPacket next = cmd;
    next.magic = MAGIC;

    this->cmd_buf_[w] = next;

    this->read_idx_.store(w);
    this->write_idx_.store(1 - w);

    return true;
}

bool EspNowInterface::GetLatestCommand(ControlPacket& out) const
{
    const int idx = this->read_idx_.load();
    out = this->cmd_buf_[idx];
    return true;
}

bool EspNowInterface::IsPeerConfigured() const
{
    return IsMacValid(DRONE_MAC);
}

bool EspNowInterface::GetLastSendOk() const
{
    return this->last_send_ok_.load();
}

//////////////////////////////////////////////////////////////////////////

void EspNowInterface::MainTask(void* param)
{
    auto* this_ = static_cast<EspNowInterface*>(param);

    if (this_ == nullptr)
    {
        vTaskDelete(nullptr);
        return;
    }

    this_->MainLoop();

    this_->handle_ = nullptr;

    vTaskDelete(nullptr);
}

void EspNowInterface::MainLoop()
{
    TickType_t last = xTaskGetTickCount();
    TickType_t period = pdMS_TO_TICKS(this->TASK_PERIOD_MS); 

    if (period == 0)
    {
        period = 1;
    }

    while (true)
    {
        if (this->task_stop_.load())
            break;

        switch (this->state_)
        {
            case ESPNOWState::INIT:
                if (this->Init() == ESP_OK)
                {
                    this->state_ = ESPNOWState::RUN;
                }
                else
                {
                    this->state_ = ESPNOWState::ERR;
                }
                break;

            case ESPNOWState::RUN:
                if (this->Run() != ESP_OK)
                {
                    this->state_ = ESPNOWState::ERR;
                }
                break;

            case ESPNOWState::STOP:
                vTaskDelay(pdMS_TO_TICKS(50));
                break;

            case ESPNOWState::ERR:
                this->Error();
                break;

            default:
                break;
        }
        vTaskDelayUntil(&last, period);
    }
}

//////////////////////////////////////////////////////////////////////////

esp_err_t EspNowInterface::Init()
{
    this->read_idx_.store(0);
    this->write_idx_.store(1);

    this->last_send_ok_.store(false);
    this->send_done_.store(false);
    this->send_status_ok_.store(false);
    this->tx_pending_.store(false);

    this->seq_ = 0;
    this->tx_pending_seq_ = 0;
    this->last_send_start_ms_ = 0;
    this->last_retry_ms_ = 0;

    ControlPacket init_pkt{};
    init_pkt.magic = MAGIC;
    init_pkt.seq = 0;
    init_pkt.throttle = 0.0f;
    init_pkt.roll_rad = 0.0f;
    init_pkt.pitch_rad = 0.0f;
    init_pkt.yaw_rate_rad_s = 0.0f;

    this->cmd_buf_[0] = init_pkt;
    this->cmd_buf_[1] = init_pkt;

    this->link_lost = false;
    this->send_fail_count_ = 0;

    return ESP_OK;
}

// esp_err_t EspNowInterface::Run()
// {
//     if (!this->hw_initialized_)
//         return ESP_OK;

//     uint32_t now = esp_timer_get_time() / 1000;

//     // dicide to send or not
//     bool should_send = false;

//     if (this->link_lost)
//     {
//         if (now - last_retry_ms_ >= 1000)
//         {
//             should_send = true;
//             last_retry_ms_ = now;
//         }
//     }
//     else
//     {
//         should_send = true;
//     }

//     if (!should_send)
//         return ESP_OK;

//     //  패킷 준비
//     ControlPacket pkt{};
//     GetLatestCommand(pkt);

//     //  send 요청
//     const esp_err_t ret = esp_now_send(
//         DRONE_MAC,
//         reinterpret_cast<const uint8_t*>(&pkt),
//         sizeof(ControlPacket));

//     if (ret != ESP_OK)
//     {
//         // 시스템 문제 → 링크 판단 X
//         // printf("send error: %s\n", esp_err_to_name(ret));
//         return ESP_OK;
//     }

//     //  callback 대기
//     SendEvent evt{};

//     if (xQueueReceive(this->tx_evt_queue_, &evt, pdMS_TO_TICKS(SEND_PERIOD_MS)) == pdTRUE)
//     {
//         this->last_send_ok_.store(evt.status == ESP_NOW_SEND_SUCCESS);
//         // printf("send status: %s\n", evt.status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
//     }
//     else
//     {
//         this->last_send_ok_.store(false);
//     }

//     //  링크 상태 업데이트
//     if (this->last_send_ok_.load())
//     {
//         this->send_fail_count_ = 0;
//         this->link_lost = false;
//     }
//     else //hystereis link lost 판단
//     {
//         this->send_fail_count_++;

//         if (this->send_fail_count_ >= 5)
//         {
//             this->link_lost = true;
//         }
//     }

//     return ESP_OK;
// }


// esp_err_t EspNowInterface::Run()
// {
//     if (!this->hw_initialized_)
//         return ESP_OK;

//     const uint32_t now = esp_timer_get_time() / 1000ULL;

//     // 1. 이전 send callback 결과 처리
//     if (this->send_done_.exchange(false))
//     {
//         if (this->send_status_ok_.load())
//         {
//             this->last_send_ok_.store(true);
//             this->send_fail_count_ = 0;
//             this->link_lost = false;
//         }
//         else
//         {
//             this->last_send_ok_.store(false);
//             ++this->send_fail_count_;

//             if (this->send_fail_count_ >= 5)
//                 this->link_lost = true;
//         }
//     }

//     // 2. pending timeout 처리
//     if (this->tx_pending_.load())
//     {
//         if (now - this->last_send_start_ms_ > TX_PENDING_TIMEOUT_MS)
//         {
//             this->tx_pending_.store(false);
//             this->last_send_ok_.store(false);
//             ++this->send_fail_count_;

//             if (this->send_fail_count_ >= 5)
//                 this->link_lost = true;
//         }
//         else
//         {
//             return ESP_OK;
//         }
//     }

//     // 3. link lost 상태에서는 1초마다 retry
//     bool should_send = false;

//     if (this->link_lost)
//     {
//         if (now - this->last_retry_ms_ >= 100)
//         {
//             should_send = true;
//             this->last_retry_ms_ = now;
//         }
//     }
//     else
//     {
//         should_send = true;
//     }

//     if (!should_send)
//         return ESP_OK;

//     // 4. packet 준비
//     ControlPacket pkt{};
//     GetLatestCommand(pkt);

//     pkt.magic = MAGIC;
//     pkt.seq = ++this->seq_;

//     // 5. 비동기 send
//     const esp_err_t ret = esp_now_send(
//         DRONE_MAC,
//         reinterpret_cast<const uint8_t*>(&pkt),
//         sizeof(ControlPacket));

//     if (ret != ESP_OK)
//     {
//         this->last_send_ok_.store(false);
//         ++this->send_fail_count_;

//         if (this->send_fail_count_ >= 5)
//             this->link_lost = true;

//         return ESP_OK;
//     }

//     this->tx_pending_seq_ = pkt.seq;
//     this->last_send_start_ms_ = now;
//     this->tx_pending_.store(true);

//     return ESP_OK;
// }

esp_err_t EspNowInterface::Run()
{
    if (!this->hw_initialized_)
        return ESP_OK;

    // 1. 이전 send callback 결과 처리
    if (this->send_done_.exchange(false))
    {
        const bool ok = this->send_status_ok_.load();

        this->last_send_ok_.store(ok);

        if (ok)
        {
            this->send_fail_count_ = 0;
            this->link_lost = false;
        }
        else
        {
            ++this->send_fail_count_;

            if (this->send_fail_count_ >= 5)
                this->link_lost = true;
        }
    }

    // 2. 최신 packet 준비
    ControlPacket pkt{};
    GetLatestCommand(pkt);

    pkt.magic = MAGIC;
    pkt.seq = ++this->seq_;

    // 3. 매 task 주기마다 비동기 send 시도
    const esp_err_t ret = esp_now_send(
        DRONE_MAC,
        reinterpret_cast<const uint8_t*>(&pkt),
        sizeof(ControlPacket));

    if (ret != ESP_OK)
    {
        this->last_send_ok_.store(false);
        ++this->send_fail_count_;

        // if (this->send_fail_count_ >= 5)
        //     this->link_lost = true;

        return ESP_OK;
    }

    return ESP_OK;
}

esp_err_t EspNowInterface::Error()
{
    vTaskDelay(pdMS_TO_TICKS(100));
    this->state_ = ESPNOWState::INIT;
    return ESP_OK;
}

//////////////////////////////////////////////////////////////////////////

esp_err_t EspNowInterface::StartTask()
{
    esp_err_t ret = xTaskCreatePinnedToCore(
        MainTask,
        "espnow_main_task",
        4096*3,
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

void EspNowInterface::StopTaskRequest()
{
    this->task_stop_.store(true);

    TaskHandle_t handle = this->handle_;
    if (handle != nullptr)
    {
        xTaskNotifyGive(handle);
    }
}

//////////////////////////////////////////////////////////////////////////

esp_err_t EspNowInterface::Initialize()
{
    this->instance_ = this;

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_t* sta_netif = esp_netif_create_default_wifi_sta();

    if (sta_netif == nullptr)
        return ESP_FAIL;


    vTaskDelay(pdMS_TO_TICKS(100)); 

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_start());

    vTaskDelay(pdMS_TO_TICKS(200));

    ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

    // tx_evt_queue_ = xQueueCreate(TX_QUEUE_SIZE, sizeof(SendEvent));
    // if (tx_evt_queue_ == nullptr)
    //     return ESP_ERR_NO_MEM;

    rx_telemetry_queue_ = xQueueCreate(RX_QUEUE_SIZE, sizeof(TelemetryPacket));
    if (rx_telemetry_queue_ == nullptr)
        return ESP_ERR_NO_MEM;


    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(EspNowSendCb));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(EspNowRecvCb));


    if (IsMacValid(DRONE_MAC))
    {
        esp_now_peer_info_t peer = {};
        std::memcpy(peer.peer_addr, DRONE_MAC, 6);
        peer.channel = ESPNOW_CHANNEL;
        peer.ifidx = WIFI_IF_STA;
        peer.encrypt = false;

        if (!esp_now_is_peer_exist(DRONE_MAC))
        {
            // ESP_ERROR_CHECK(esp_now_add_peer(&peer));

            esp_err_t add_ret = esp_now_add_peer(&peer);
            if (add_ret != ESP_OK)
            {
                return add_ret;
            }
        }
    }
    else    
    {
        return ESP_ERR_INVALID_ARG;
    }

    this->hw_initialized_ = true;

    return this->StartTask();
}