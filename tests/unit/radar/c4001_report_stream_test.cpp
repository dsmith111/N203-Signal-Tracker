#include <cassert>
#include <cstddef>
#include <cstdint>
#include <string>

#include "n203/radar/c4001/report_stream.hpp"

namespace {

namespace c4001 = n203::radar::c4001;

c4001::StreamEvent pushText(
    c4001::ReportStream& stream,
    const char* text,
    c4001::Report& report) {
    c4001::StreamEvent event = c4001::StreamEvent::kNone;
    for (std::size_t index = 0; text[index] != '\0'; ++index) {
        const auto next = stream.push(static_cast<std::uint8_t>(text[index]), report);
        if (next != c4001::StreamEvent::kNone) {
            event = next;
        }
    }
    return event;
}

}  // namespace

void runC4001ReportStreamTests() {
    {
        c4001::ReportStream stream;
        c4001::Report report{};
        assert(pushText(stream, "$DFHPD,1, , , *", report) ==
               c4001::StreamEvent::kReportReady);
        assert(std::string(report.text.data(), report.size) == "$DFHPD,1, , , *");
        assert(stream.stats().reports == 1U);
    }

    {
        c4001::ReportStream stream;
        c4001::Report report{};
        assert(pushText(stream, "boot\r\n$DFDMD,1,1,1.817,0.129,15304, , *\r\n", report) ==
               c4001::StreamEvent::kReportReady);
        assert(std::string(report.text.data(), report.size) ==
               "$DFDMD,1,1,1.817,0.129,15304, , *");
    }

    {
        c4001::ReportStream stream;
        c4001::Report report{};
        pushText(stream, "$DFHPD,1,", report);
        assert(pushText(stream, "$DFHPD,0, , , *", report) ==
               c4001::StreamEvent::kReportReady);
        assert(std::string(report.text.data(), report.size) == "$DFHPD,0, , , *");
        assert(stream.stats().interrupted_reports == 1U);
    }

    {
        c4001::ReportStream stream;
        c4001::Report report{};
        assert(stream.push('$', report) == c4001::StreamEvent::kNone);
        for (std::size_t index = 0; index < c4001::kMaximumReportSize - 1U; ++index) {
            assert(stream.push('X', report) == c4001::StreamEvent::kNone);
        }
        assert(stream.push('X', report) == c4001::StreamEvent::kOverflowRejected);
        assert(pushText(stream, "$DFHPD,1, , , *", report) ==
               c4001::StreamEvent::kReportReady);
        assert(stream.stats().overflow_rejections == 1U);
    }

    {
        c4001::ReportStream stream;
        c4001::Report report{};
        pushText(stream, "$DFHPD,1,", report);
        stream.resetCandidate();
        assert(pushText(stream, "ignored*$DFHPD,0, , , *", report) ==
               c4001::StreamEvent::kReportReady);
        assert(std::string(report.text.data(), report.size) == "$DFHPD,0, , , *");
    }
}
