#pragma once

#include <LovyanGFX.hpp>

namespace n203::firmware::display_s3 {

class N203LgfxDevice final : public lgfx::LGFX_Device {
public:
    N203LgfxDevice();

private:
    lgfx::Bus_SPI bus_;
    lgfx::Panel_ILI9488 panel_;
    lgfx::Light_PWM light_;
    lgfx::Touch_XPT2046 touch_;
};

}  // namespace n203::firmware::display_s3
