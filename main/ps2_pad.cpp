#include "ps2_pad.h"

#include <cstring>

#include "esp_log.h"
#include "esp_rom_sys.h"

static const char* TAG = "Ps2Pad";

Ps2Pad::Ps2Pad(gpio_num_t pin_att,
               gpio_num_t pin_cmd,
               gpio_num_t pin_dat,
               gpio_num_t pin_clk)
    : pin_att_(pin_att),
      pin_cmd_(pin_cmd),
      pin_dat_(pin_dat),
      pin_clk_(pin_clk)
{
}

void Ps2Pad::DelayUs(uint32_t us) const
{
    esp_rom_delay_us(us);
}

void Ps2Pad::Select() const
{
    gpio_set_level(pin_att_, 0);
    DelayUs(ATT_DELAY_MS);
}

void Ps2Pad::Deselect() const
{
    gpio_set_level(pin_att_, 1);
    DelayUs(ATT_DELAY_MS);
}

uint8_t Ps2Pad::TransferByte(uint8_t tx) const
{
    uint8_t rx = 0;

    // PS2류는 보통 LSB first
    // for (int i = 0; i < 8; ++i)
    // {
    //     // CLK idle high, falling/rising edges around sampling
    //     gpio_set_level(pin_clk_, 1);
    //     gpio_set_level(pin_cmd_, (tx >> i) & 0x01);
    //     DelayUs(BIT_DELAY_US);

    //     gpio_set_level(pin_clk_, 0);
    //     DelayUs(BIT_DELAY_US);

    //     const int bit = gpio_get_level(pin_dat_);
    //     rx |= (bit & 0x01) << i;

    //     gpio_set_level(pin_clk_, 1);
    //     DelayUs(BIT_DELAY_US);
    // }

    for (int i = 0; i < 8; ++i)
    {
        // 1. CMD 데이터를 먼저 세팅 (충분한 Setup Time 확보)
        gpio_set_level(pin_cmd_, (tx >> i) & 0x01);
        DelayUs(BIT_DELAY_US);

        // 2. Clock을 Low로 떨어뜨림 (이 순간 패드가 DAT를 준비함)
        gpio_set_level(pin_clk_, 0);
        
        // ★ 패드가 DAT 라인을 안정화할 시간을 줍니다. (매우 중요)
        DelayUs(BIT_DELAY_US); 

        // 3. 데이터 읽기
        if (gpio_get_level(pin_dat_)) {
            rx |= (1 << i);
        }

        // 4. Clock을 다시 High로 (다음 비트 준비)
        gpio_set_level(pin_clk_, 1);
        DelayUs(BIT_DELAY_US);
    }


    return rx;
}

esp_err_t Ps2Pad::SendCommand(const uint8_t* tx, uint8_t* rx, int len) const
{
    if (tx == nullptr || rx == nullptr || len <= 0)
        return ESP_ERR_INVALID_ARG;

    Select();
    DelayUs(20);

    for (int i = 0; i < len; ++i)
    {
        rx[i] = TransferByte(tx[i]);
    }

    Deselect();
    DelayUs(20);

    return ESP_OK;
}

esp_err_t Ps2Pad::ReadRawFrame(uint8_t* rx, int len)
{
    if (rx == nullptr || len < 9)
        return ESP_ERR_INVALID_ARG;

    // 기본 poll 커맨드
    // 0x01, 0x42, 0x00 이후 응답으로 상태가 옴
    uint8_t tx[9] = {0x01, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    return SendCommand(tx, rx, 9);
}

esp_err_t Ps2Pad::EnterConfigMode()
{
    uint8_t tx[9] = {0x01, 0x43, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t rx[9] = {};
    return SendCommand(tx, rx, 9);
}

esp_err_t Ps2Pad::ExitConfigMode()
{
    uint8_t tx[9] = {0x01, 0x43, 0x00, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A};
    uint8_t rx[9] = {};
    return SendCommand(tx, rx, 9);
}

esp_err_t Ps2Pad::SetAnalogMode()
{
    // 아날로그 모드/락 설정 시도
    uint8_t tx[9] = {0x01, 0x44, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00};
    uint8_t rx[9] = {};
    return SendCommand(tx, rx, 9);
}

esp_err_t Ps2Pad::Initialize()
{
    gpio_config_t out_cfg = {};
    out_cfg.pin_bit_mask =
        (1ULL << pin_att_) |
        (1ULL << pin_cmd_) |
        (1ULL << pin_clk_);
    out_cfg.mode = GPIO_MODE_OUTPUT;
    out_cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    out_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    out_cfg.intr_type = GPIO_INTR_DISABLE;

    ESP_ERROR_CHECK(gpio_config(&out_cfg));

    gpio_config_t in_cfg = {};
    in_cfg.pin_bit_mask = (1ULL << pin_dat_);
    in_cfg.mode = GPIO_MODE_INPUT;
    in_cfg.pull_up_en = GPIO_PULLUP_ENABLE;   // 데이터 라인 안정화 목적
    in_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    in_cfg.intr_type = GPIO_INTR_DISABLE;

    ESP_ERROR_CHECK(gpio_config(&in_cfg));

    // idle 상태
    gpio_set_level(pin_att_, 1);
    gpio_set_level(pin_clk_, 1);
    gpio_set_level(pin_cmd_, 1);

    DelayUs(1000);

    // 초기 poll 몇 번
    for (int i = 0; i < 5; ++i)
    {
        uint8_t rx[9] = {};
        ReadRawFrame(rx, 9);


        // printf("%02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
        //     rx[0], rx[1], rx[2], rx[3], rx[4],
        //     rx[5], rx[6], rx[7], rx[8]);

        DelayUs(1000);
    }

    // 아날로그 모드 진입 시도
    ESP_ERROR_CHECK(EnterConfigMode());
    DelayUs(1000);

    ESP_ERROR_CHECK(SetAnalogMode());
    DelayUs(1000);

    ESP_ERROR_CHECK(ExitConfigMode());
    DelayUs(1000);

    // 초기화 후 상태 확인
    uint8_t rx[9] = {};
    ESP_ERROR_CHECK(ReadRawFrame(rx, 9));

    // ESP_LOGI(TAG,
    //          "Init poll resp: %02X %02X %02X %02X %02X %02X %02X %02X %02X",
    //          rx[0], rx[1], rx[2], rx[3], rx[4], rx[5], rx[6], rx[7], rx[8]);

    return ESP_OK;
}

esp_err_t Ps2Pad::Read(PadState& out)
{
    uint8_t rx[9] = {};
    const esp_err_t ret = ReadRawFrame(rx, 9);
    if (ret != ESP_OK)
    {
        out.valid = false;
        return ret;
    }

    // rx[1]가 보통 모드 정보
    // 0x41: digital, 0x73: analog(red), clone은 다르게 올 수도 있음
    const uint8_t mode = rx[1];
    const uint8_t header = rx[2];

    if (header != 0x5A)
    {
        out.valid = false;
        return ESP_OK;
    }

    // 버튼 16비트
    // 일반적으로 rx[3]=low, rx[4]=high
    out.buttons = static_cast<uint16_t>(rx[3]) |
                  (static_cast<uint16_t>(rx[4]) << 8);

    // 아날로그 모드일 때 축 정보
    // 보통 rx[5]=RX, rx[6]=RY, rx[7]=LX, rx[8]=LY
    if (mode == 0x73 || mode == 0x79)
    {
        out.rx = rx[5];
        out.ry = rx[6];
        out.lx = rx[7];
        out.ly = rx[8];
        out.valid = true;
        return ESP_OK;
    }

    // 디지털 모드면 축은 기본값 유지
    out.lx = 128;
    out.ly = 128;
    out.rx = 128;
    out.ry = 128;
    out.valid = true;

    return ESP_OK;
}