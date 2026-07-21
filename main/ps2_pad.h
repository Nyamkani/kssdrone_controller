
#pragma once

#include <cstdint>

#include "driver/gpio.h"
#include "esp_err.h"

class Ps2Pad
{
public:
    struct PadState
    {
        uint16_t buttons = 0xFFFF; // pressed=0, released=1
        uint8_t lx = 128;
        uint8_t ly = 128;
        uint8_t rx = 128;
        uint8_t ry = 128;
        bool valid = false;
    };

public:
    Ps2Pad(gpio_num_t pin_att,
           gpio_num_t pin_cmd,
           gpio_num_t pin_dat,
           gpio_num_t pin_clk);

    esp_err_t Initialize();
    esp_err_t Read(PadState& out);

        esp_err_t ReadRawFrame(uint8_t* rx, int len);
private:
    void DelayUs(uint32_t us) const;
    uint8_t TransferByte(uint8_t tx) const;

    void Select() const;
    void Deselect() const;

    esp_err_t SendCommand(const uint8_t* tx, uint8_t* rx, int len) const;
    esp_err_t EnterConfigMode();
    esp_err_t ExitConfigMode();
    esp_err_t SetAnalogMode();
    // esp_err_t ReadRawFrame(uint8_t* rx, int len);

private:
    gpio_num_t pin_att_;
    gpio_num_t pin_cmd_;
    gpio_num_t pin_dat_;
    gpio_num_t pin_clk_;

    static constexpr uint32_t BIT_DELAY_US = 10;//8;
    static constexpr uint32_t ATT_DELAY_MS = 20;
};