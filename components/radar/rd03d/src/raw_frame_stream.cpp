#include "n203/radar/rd03d/raw_frame_stream.hpp"

#include <algorithm>

namespace n203::radar::rd03d {

void RawFrameStream::searchForHeader(std::uint8_t byte) {
    if (byte == kFrameHeader[header_match_size_]) {
        ++header_match_size_;
    } else {
        // AA is both the only useful suffix and the first header byte.
        header_match_size_ = byte == kFrameHeader[0] ? 1U : 0U;
    }

    if (header_match_size_ == kFrameHeader.size()) {
        std::copy(kFrameHeader.begin(), kFrameHeader.end(), candidate_.begin());
        candidate_size_ = kFrameHeader.size();
        header_match_size_ = 0U;
    }
}

void RawFrameStream::resumeHeaderSearchFromCandidateSuffix() {
    const std::array<std::uint8_t, kFrameHeader.size() - 1U> suffix{{
        candidate_[kRawFrameSize - 3U],
        candidate_[kRawFrameSize - 2U],
        candidate_[kRawFrameSize - 1U],
    }};
    candidate_size_ = 0U;
    header_match_size_ = 0U;
    for (const std::uint8_t byte : suffix) {
        searchForHeader(byte);
    }
}

StreamEvent RawFrameStream::push(std::uint8_t byte, RawFrame& frame) {
    ++stats_.bytes;
    if (candidate_size_ == 0U) {
        searchForHeader(byte);
        return StreamEvent::kNone;
    }

    candidate_[candidate_size_++] = byte;
    if (candidate_size_ < candidate_.size()) {
        return StreamEvent::kNone;
    }

    frame = candidate_;
    const bool has_valid_trailer =
        candidate_[kRawFrameSize - 2U] == kFrameTrailer[0] &&
        candidate_[kRawFrameSize - 1U] == kFrameTrailer[1];
    if (has_valid_trailer) {
        ++stats_.frames;
        candidate_size_ = 0U;
        header_match_size_ = 0U;
        return StreamEvent::kFrameReady;
    }

    ++stats_.rejected_frames;
    resumeHeaderSearchFromCandidateSuffix();
    return StreamEvent::kFrameRejected;
}

}  // namespace n203::radar::rd03d
