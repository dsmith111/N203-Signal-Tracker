#pragma once

#include "n203/radar/radar_frame.hpp"

namespace n203::radar {

class RadarSource {
public:
    virtual ~RadarSource() = default;
    virtual bool poll(RadarFrame& frame) = 0;
};

}  // namespace n203::radar
