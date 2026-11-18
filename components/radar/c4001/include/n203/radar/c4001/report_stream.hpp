#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace n203::radar::c4001 {

inline constexpr std::size_t kMaximumReportSize = 96U;

struct Report {
    std::array<char, kMaximumReportSize + 1U> text{};
    std::size_t size = 0U;
};

enum class StreamEvent : std::uint8_t {
    kNone,
    kReportReady,
    kOverflowRejected,
};

struct StreamStats {
    std::uint32_t bytes = 0;
    std::uint32_t reports = 0;
    std::uint32_t overflow_rejections = 0;
    std::uint32_t interrupted_reports = 0;
};

// Bounded framing for C4001 ASCII active reports. A report starts with '$'
// and ends with '*'. Semantic parsing is intentionally deferred until bytes
// from the physical sensor have been captured as fixtures.
class ReportStream {
public:
    StreamEvent push(std::uint8_t byte, Report& report);
    void resetCandidate();

    const StreamStats& stats() const { return stats_; }

private:
    std::array<char, kMaximumReportSize + 1U> candidate_{};
    std::size_t candidate_size_ = 0U;
    StreamStats stats_{};
};

}  // namespace n203::radar::c4001
