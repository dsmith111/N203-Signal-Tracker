#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace n203::radar::rd03d {

inline constexpr std::array<std::uint8_t, 4> kFrameHeader{{0xAA, 0xFF, 0x03, 0x00}};
inline constexpr std::array<std::uint8_t, 2> kFrameTrailer{{0x55, 0xCC}};
inline constexpr std::size_t kTargetSlotCount = 3U;
inline constexpr std::size_t kTargetSlotSize = 8U;
inline constexpr std::size_t kRawFrameSize =
    kFrameHeader.size() + kTargetSlotCount * kTargetSlotSize + kFrameTrailer.size();

using RawFrame = std::array<std::uint8_t, kRawFrameSize>;

enum class StreamEvent : std::uint8_t {
    kNone,
    kFrameReady,
    kFrameRejected,
};

struct StreamStats {
    std::uint32_t bytes = 0;
    std::uint32_t frames = 0;
    std::uint32_t rejected_frames = 0;
};

// A bounded framing-only collector for the documented fixed-size RD-03D UART
// report. It intentionally does not interpret target fields; captured bytes
// become evidence and native-test fixtures before normalization is enabled.
class RawFrameStream {
public:
    StreamEvent push(std::uint8_t byte, RawFrame& frame);

    const StreamStats& stats() const { return stats_; }

private:
    void searchForHeader(std::uint8_t byte);
    void resumeHeaderSearchFromCandidateSuffix();

    RawFrame candidate_{};
    std::size_t candidate_size_ = 0;
    std::size_t header_match_size_ = 0;
    StreamStats stats_{};
};

}  // namespace n203::radar::rd03d
