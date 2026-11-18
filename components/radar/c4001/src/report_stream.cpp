#include "n203/radar/c4001/report_stream.hpp"

#include <algorithm>

namespace n203::radar::c4001 {

void ReportStream::resetCandidate() {
    candidate_.fill('\0');
    candidate_size_ = 0U;
}

StreamEvent ReportStream::push(std::uint8_t byte, Report& report) {
    ++stats_.bytes;

    if (byte == '$') {
        if (candidate_size_ != 0U) {
            ++stats_.interrupted_reports;
        }
        candidate_.fill('\0');
        candidate_[0] = '$';
        candidate_size_ = 1U;
        return StreamEvent::kNone;
    }

    if (candidate_size_ == 0U) {
        return StreamEvent::kNone;
    }

    if (candidate_size_ >= kMaximumReportSize) {
        candidate_size_ = 0U;
        ++stats_.overflow_rejections;
        return StreamEvent::kOverflowRejected;
    }

    candidate_[candidate_size_++] = static_cast<char>(byte);
    if (byte != '*') {
        return StreamEvent::kNone;
    }

    candidate_[candidate_size_] = '\0';
    report.text = candidate_;
    report.size = candidate_size_;
    candidate_size_ = 0U;
    ++stats_.reports;
    return StreamEvent::kReportReady;
}

}  // namespace n203::radar::c4001
