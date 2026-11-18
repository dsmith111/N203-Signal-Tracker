#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>

#include "n203/radar/rd03d/raw_frame_stream.hpp"

namespace {

namespace rd03d = n203::radar::rd03d;

constexpr rd03d::RawFrame kDocumentedExample{{
    0xAA, 0xFF, 0x03, 0x00,
    0x0E, 0x03, 0xB1, 0x86, 0x10, 0x00, 0x4A, 0x01,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x55, 0xCC,
}};

rd03d::StreamEvent pushFrame(
    rd03d::RawFrameStream& stream,
    const rd03d::RawFrame& input,
    rd03d::RawFrame& output) {
    rd03d::StreamEvent event = rd03d::StreamEvent::kNone;
    for (const std::uint8_t byte : input) {
        const rd03d::StreamEvent next = stream.push(byte, output);
        if (next != rd03d::StreamEvent::kNone) {
            event = next;
        }
    }
    return event;
}

}  // namespace

void runRd03dRawFrameStreamTests() {
    {
        rd03d::RawFrameStream stream;
        rd03d::RawFrame output{};
        assert(pushFrame(stream, kDocumentedExample, output) ==
               rd03d::StreamEvent::kFrameReady);
        assert(output == kDocumentedExample);
        assert(stream.stats().bytes == rd03d::kRawFrameSize);
        assert(stream.stats().frames == 1U);
        assert(stream.stats().rejected_frames == 0U);
    }

    {
        rd03d::RawFrameStream stream;
        rd03d::RawFrame output{};
        for (const std::uint8_t noise : std::array<std::uint8_t, 7>{{
                 0x00, 0xAA, 0xAA, 0x12, 0xFF, 0x03, 0x00}}) {
            assert(stream.push(noise, output) == rd03d::StreamEvent::kNone);
        }
        assert(pushFrame(stream, kDocumentedExample, output) ==
               rd03d::StreamEvent::kFrameReady);
        assert(output == kDocumentedExample);
    }

    {
        rd03d::RawFrameStream stream;
        rd03d::RawFrame output{};
        rd03d::RawFrame corrupt = kDocumentedExample;
        corrupt.back() = 0x00;
        assert(pushFrame(stream, corrupt, output) ==
               rd03d::StreamEvent::kFrameRejected);
        assert(output == corrupt);
        assert(pushFrame(stream, kDocumentedExample, output) ==
               rd03d::StreamEvent::kFrameReady);
        assert(stream.stats().frames == 1U);
        assert(stream.stats().rejected_frames == 1U);
    }

    {
        // A missing payload byte causes the first byte of the next frame to be
        // consumed by the rejected candidate. Suffix recovery retains that AA
        // and synchronizes without sacrificing the following complete frame.
        rd03d::RawFrameStream stream;
        rd03d::RawFrame output{};
        for (std::size_t index = 0; index < kDocumentedExample.size(); ++index) {
            if (index != 12U) {
                stream.push(kDocumentedExample[index], output);
            }
        }
        assert(stream.push(kDocumentedExample[0], output) ==
               rd03d::StreamEvent::kFrameRejected);
        for (std::size_t index = 1; index < kDocumentedExample.size(); ++index) {
            stream.push(kDocumentedExample[index], output);
        }
        assert(output == kDocumentedExample);
        assert(stream.stats().frames == 1U);
        assert(stream.stats().rejected_frames == 1U);
    }
}
