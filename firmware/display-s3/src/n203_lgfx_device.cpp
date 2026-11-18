#include "n203_lgfx_device.hpp"

#include "board_config.hpp"

namespace n203::firmware::display_s3 {

N203LgfxDevice::N203LgfxDevice() {
    {
        auto config = bus_.config();
        config.spi_host = SPI2_HOST;
        config.spi_mode = 0;
        config.freq_write = board::kLcdWriteFrequencyHz;
        config.freq_read = board::kLcdReadFrequencyHz;
        config.spi_3wire = false;
        config.use_lock = true;
        config.dma_channel = SPI_DMA_CH_AUTO;
        config.pin_sclk = board::kSpiSclk;
        config.pin_mosi = board::kSpiMosi;
        config.pin_miso = board::kSpiMiso;
        config.pin_dc = board::kLcdDc;
        bus_.config(config);
        panel_.setBus(&bus_);
    }

    {
        auto config = panel_.config();
        config.pin_cs = board::kLcdCs;
        config.pin_rst = board::kLcdReset;
        config.pin_busy = -1;
        config.readable = false;
        config.bus_shared = true;
        panel_.config(config);
    }

    {
        auto config = light_.config();
        config.pin_bl = board::kBacklightControl;
        config.invert = false;
        config.freq = board::kBacklightPwmFrequencyHz;
        config.pwm_channel = board::kBacklightPwmChannel;
        light_.config(config);
        panel_.setLight(&light_);
    }

    {
        auto config = touch_.config();
        config.pin_int = board::kTouchIrq;
        config.bus_shared = true;
        config.offset_rotation = 0;
        config.spi_host = SPI2_HOST;
        config.freq = board::kTouchFrequencyHz;
        config.pin_sclk = board::kSpiSclk;
        config.pin_mosi = board::kSpiMosi;
        config.pin_miso = board::kSpiMiso;
        config.pin_cs = board::kTouchCs;
        touch_.config(config);
        panel_.setTouch(&touch_);
    }

    setPanel(&panel_);
}

}  // namespace n203::firmware::display_s3
