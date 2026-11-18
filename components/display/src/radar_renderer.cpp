#include "n203/display/radar_renderer.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

#include "n203/display/layout.hpp"
#include "n203/display/radar_geometry.hpp"
#include "n203/display/theme.hpp"

namespace n203::display {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr int kHalfFieldOfViewDegrees = 40;

const char* scannerLabel(ScannerPresentation scanner) {
    switch (scanner) {
        case ScannerPresentation::kStarting: return "WARMUP";
        case ScannerPresentation::kScanning: return "MOTION";
        case ScannerPresentation::kRangeOnly: return "RANGE";
        case ScannerPresentation::kPresenceOnly: return "PRESENCE";
        case ScannerPresentation::kStale: return "DATA HOLD";
        case ScannerPresentation::kOffline: return "NO SIGNAL";
        case ScannerPresentation::kInvalidFrame: return "FRAME ERR";
    }
    return "UNKNOWN";
}

Color scannerColor(ScannerPresentation scanner) {
    switch (scanner) {
        case ScannerPresentation::kStale:
        case ScannerPresentation::kInvalidFrame:
            return theme::kWarning;
        case ScannerPresentation::kOffline:
            return theme::kAlert;
        default:
            return theme::kText;
    }
}

const char* statusLabel(const DisplayState& state) {
    switch (state.scanner) {
        case ScannerPresentation::kStarting:
        case ScannerPresentation::kStale:
        case ScannerPresentation::kOffline:
        case ScannerPresentation::kInvalidFrame:
            return scannerLabel(state.scanner);
        default:
            break;
    }

    if (state.frame.motion == radar::MotionState::kDopplerWarning) {
        return "DOPPLER WARN";
    }
    if (state.frame.motion == radar::MotionState::kStabilizing) {
        return "STABILIZING";
    }
    if (state.scanner == ScannerPresentation::kPresenceOnly) {
        switch (state.frame.presence) {
            case radar::PresenceState::kClear: return "AREA CLEAR";
            case radar::PresenceState::kDetected: return "PRES DETECT";
            case radar::PresenceState::kUnknown: return "PRESENCE ?";
        }
    }
    return scannerLabel(state.scanner);
}

Color statusColor(const DisplayState& state) {
    if (state.scanner == ScannerPresentation::kStarting ||
        state.scanner == ScannerPresentation::kStale ||
        state.scanner == ScannerPresentation::kOffline ||
        state.scanner == ScannerPresentation::kInvalidFrame) {
        return scannerColor(state.scanner);
    }
    if (state.frame.motion == radar::MotionState::kDopplerWarning ||
        state.frame.motion == radar::MotionState::kStabilizing) {
        return theme::kWarning;
    }
    if (state.scanner == ScannerPresentation::kPresenceOnly) {
        switch (state.frame.presence) {
            case radar::PresenceState::kClear: return theme::kMutedText;
            case radar::PresenceState::kDetected: return theme::kPrimary;
            case radar::PresenceState::kUnknown: return theme::kWarning;
        }
    }
    return scannerColor(state.scanner);
}

Point sectorPoint(int angle_degrees, int radius) {
    const double angle = static_cast<double>(angle_degrees) * kPi / 180.0;
    const Point origin = DisplayLayout::radarCenter();
    return {
        origin.x + static_cast<int>(std::lround(std::sin(angle) * radius)),
        origin.y - static_cast<int>(std::lround(std::cos(angle) * radius)),
    };
}

void drawSectorArc(DisplayCanvas& graphics, int radius, Color color) {
    Point previous = sectorPoint(-kHalfFieldOfViewDegrees, radius);
    for (int angle = -kHalfFieldOfViewDegrees + 2; angle <= kHalfFieldOfViewDegrees; angle += 2) {
        const Point next = sectorPoint(angle, radius);
        graphics.drawLine(previous.x, previous.y, next.x, next.y, color);
        previous = next;
    }
}

void drawSectorGrid(DisplayCanvas& graphics, std::uint32_t now_ms) {
    const Point origin = DisplayLayout::radarCenter();
    const int outer_radius = DisplayLayout::radarRadius();
    const bool brightness_flutter = ((now_ms / 90U) % 37U) == 0U;
    const Color inner_grid = brightness_flutter ? theme::kGrid : theme::kGridDim;

    for (int band = 1; band < 4; ++band) {
        drawSectorArc(graphics, outer_radius * band / 4, inner_grid);
    }
    drawSectorArc(graphics, outer_radius, theme::kGrid);

    const Point left_edge = sectorPoint(-kHalfFieldOfViewDegrees, outer_radius);
    const Point right_edge = sectorPoint(kHalfFieldOfViewDegrees, outer_radius);
    graphics.drawLine(origin.x, origin.y, left_edge.x, left_edge.y, theme::kGrid);
    graphics.drawLine(origin.x, origin.y, right_edge.x, right_edge.y, theme::kGrid);

    for (int angle = -kHalfFieldOfViewDegrees; angle <= kHalfFieldOfViewDegrees; angle += 10) {
        const Point outer = sectorPoint(angle, outer_radius);
        const Point inner = sectorPoint(angle, outer_radius - 7);
        graphics.drawLine(inner.x, inner.y, outer.x, outer.y, theme::kPrimary);
    }

    const int pulse_radius = static_cast<int>((now_ms % 2400U) * static_cast<std::uint32_t>(outer_radius) / 2400U);
    if (pulse_radius > 18) {
        drawSectorArc(graphics, pulse_radius, theme::kPulse);
    }
    graphics.fillCircle(origin.x, origin.y, 3, theme::kPrimary);
}

std::uint32_t contactRangeMm(const radar::Contact& contact) {
    if (contact.has(radar::kContactRange)) {
        return contact.range_mm;
    }
    if (contact.has(radar::kContactPosition)) {
        const double squared_range = static_cast<double>(contact.x_mm) * contact.x_mm +
                                     static_cast<double>(contact.y_mm) * contact.y_mm;
        return static_cast<std::uint32_t>(std::lround(std::sqrt(squared_range)));
    }
    return std::numeric_limits<std::uint32_t>::max();
}

void drawTrackingBrackets(DisplayCanvas& graphics, Point point, int radius, Color color) {
    constexpr int kCornerLength = 5;
    graphics.drawLine(point.x - radius, point.y - radius, point.x - radius + kCornerLength, point.y - radius, color);
    graphics.drawLine(point.x - radius, point.y - radius, point.x - radius, point.y - radius + kCornerLength, color);
    graphics.drawLine(point.x + radius, point.y - radius, point.x + radius - kCornerLength, point.y - radius, color);
    graphics.drawLine(point.x + radius, point.y - radius, point.x + radius, point.y - radius + kCornerLength, color);
    graphics.drawLine(point.x - radius, point.y + radius, point.x - radius + kCornerLength, point.y + radius, color);
    graphics.drawLine(point.x - radius, point.y + radius, point.x - radius, point.y + radius - kCornerLength, color);
    graphics.drawLine(point.x + radius, point.y + radius, point.x + radius - kCornerLength, point.y + radius, color);
    graphics.drawLine(point.x + radius, point.y + radius, point.x + radius, point.y + radius - kCornerLength, color);
}

bool drawContact(
    DisplayCanvas& graphics,
    const radar::Contact& contact,
    const DisplayState& state,
    std::uint32_t now_ms,
    std::uint32_t& nearest_range_mm) {
    Point point{};
    if (!radarPointForContact(contact, state.display_range_mm, point)) {
        return false;
    }

    nearest_range_mm = std::min(nearest_range_mm, contactRangeMm(contact));
    std::uint32_t jitter_seed = static_cast<std::uint32_t>(contact.id) * 747796405U;
    jitter_seed ^= (now_ms / 110U) * 2891336453U;
    jitter_seed ^= jitter_seed >> 16U;
    point.x += static_cast<int>(jitter_seed % 3U) - 1;
    point.y += static_cast<int>((jitter_seed >> 4U) % 3U) - 1;

    const Point origin = DisplayLayout::radarCenter();
    const double delta_x = point.x - origin.x;
    const double delta_y = point.y - origin.y;
    const double pixel_range = std::max(1.0, std::sqrt(delta_x * delta_x + delta_y * delta_y));
    if (contact.has(radar::kContactRadialVelocity) && contact.radial_velocity_mm_s != 0) {
        const int history_direction = contact.radial_velocity_mm_s > 0 ? -1 : 1;
        for (int ghost = 2; ghost >= 1; --ghost) {
            const int offset = ghost * 5 * history_direction;
            const Point ghost_point{
                point.x + static_cast<int>(std::lround(delta_x / pixel_range * offset)),
                point.y + static_cast<int>(std::lround(delta_y / pixel_range * offset)),
            };
            graphics.fillCircle(ghost_point.x, ghost_point.y, ghost + 1, theme::kTargetGhost);
        }
    }

    const bool selected = state.has_selected_contact && state.selected_contact_id == contact.id;
    const bool locked = selected && state.target_locked;
    const Color contact_body = locked ? theme::kLocked : theme::kTargetBody;
    const Color tracking_color = locked ? theme::kLocked : theme::kSelection;
    const int halo_radius = 7 + static_cast<int>(contact.confidence) / 28;
    graphics.fillCircle(point.x, point.y, halo_radius, theme::kTargetHalo);
    graphics.fillCircle(point.x, point.y, 5, contact_body);
    graphics.fillCircle(point.x, point.y, 2, theme::kTargetCore);
    graphics.fillRect(point.x + halo_radius - 1, point.y + static_cast<int>((jitter_seed >> 8U) % 3U) - 1,
                      2, 2, contact_body);

    if (selected) {
        drawTrackingBrackets(graphics, point, halo_radius + 5, tracking_color);
        const std::string tracking_label = (state.target_locked ? "LOCK " : "TRK ") + std::to_string(contact.id);
        graphics.drawText(point.x + halo_radius + 8, point.y - 3, tracking_label, tracking_color);
    }
    return true;
}

std::string formatDistance(std::uint32_t range_mm) {
    if (range_mm == std::numeric_limits<std::uint32_t>::max()) {
        return "--.-M";
    }
    return std::to_string(range_mm / 1000U) + "." + std::to_string((range_mm % 1000U) / 100U) + "M";
}

void drawEdgeIndicators(DisplayCanvas& graphics, std::uint8_t contact_count) {
    for (int indicator = 0; indicator < 9; ++indicator) {
        const int y = 48 + indicator * 22;
        const Color color = indicator < static_cast<int>(contact_count) + 2 ? theme::kText : theme::kGridDim;
        graphics.fillRect(3, y, indicator % 3 == 0 ? 7 : 4, 8, color);
        graphics.fillRect(470 + (indicator % 3 == 0 ? 0 : 3), y, indicator % 3 == 0 ? 7 : 4, 8, color);
    }
}

void drawCrtOverlay(DisplayCanvas& graphics) {
    for (int y = 3; y < DisplayLayout::kLogicalHeight; y += 4) {
        graphics.drawLine(1, y, DisplayLayout::kLogicalWidth - 2, y, theme::kScanline);
    }

    // Keep the sparse phosphor grain fixed. Animating all 42 specks dirties
    // dozens of otherwise static SPI tiles on every frame without adding
    // useful motion information.
    std::uint32_t noise = 0x91E10DA5U;
    for (int speck = 0; speck < 42; ++speck) {
        noise = noise * 1664525U + 1013904223U;
        const int x = static_cast<int>(noise % DisplayLayout::kLogicalWidth);
        noise = noise * 1664525U + 1013904223U;
        const int y = static_cast<int>(noise % DisplayLayout::kLogicalHeight);
        graphics.fillRect(x, y, 1, 1, theme::kNoise);
    }

    graphics.strokeRect(0, 0, DisplayLayout::kLogicalWidth, DisplayLayout::kLogicalHeight, theme::kBezelEdge);
    graphics.strokeRect(1, 1, DisplayLayout::kLogicalWidth - 2, DisplayLayout::kLogicalHeight - 2,
                        theme::kBezelEdge);
}

}  // namespace

void RadarRenderer::render(DisplayCanvas& graphics, const DisplayState& state, std::uint32_t now_ms) const {
    graphics.clear(theme::kBackground);
    drawSectorGrid(graphics, now_ms);

    std::uint32_t nearest_range_mm = std::numeric_limits<std::uint32_t>::max();
    std::uint8_t visible_contact_count = 0;
    for (std::size_t index = 0; index < state.frame.contact_count; ++index) {
        if (drawContact(graphics, state.frame.contacts[index], state, now_ms, nearest_range_mm)) {
            ++visible_contact_count;
        }
    }
    drawEdgeIndicators(graphics, visible_contact_count);

    graphics.drawText(12, 10, "N203/MTRK", theme::kMutedText);
    const char* status_label = statusLabel(state);
    const Color status_color = statusColor(state);
    const int status_width = static_cast<int>(std::char_traits<char>::length(status_label)) * 6;
    graphics.drawText((DisplayLayout::kLogicalWidth - status_width) / 2, 10,
                      status_label, status_color);
    graphics.drawText(419, 10, "MOT " + std::to_string(visible_contact_count), theme::kText);
    if (state.has_selected_contact) {
        graphics.drawText(12, 26,
                          (state.target_locked ? "LOCK " : "TRACK ") + std::to_string(state.selected_contact_id),
                          state.target_locked ? theme::kLocked : theme::kSelection);
    } else {
        graphics.drawText(12, 26, "TRACK AUTO", theme::kMutedText);
    }

    const bool show_status_banner =
        state.scanner == ScannerPresentation::kStale ||
        state.scanner == ScannerPresentation::kOffline ||
        state.scanner == ScannerPresentation::kInvalidFrame ||
        state.frame.motion == radar::MotionState::kDopplerWarning ||
        state.frame.motion == radar::MotionState::kStabilizing;
    if (show_status_banner) {
        graphics.fillRect(174, 38, 132, 22, theme::kPanel);
        graphics.strokeRect(174, 38, 132, 22, status_color);
        graphics.drawText(240 - status_width / 2, 46, status_label, status_color);
    }

    graphics.fillRect(0, 282, DisplayLayout::kLogicalWidth, 38, theme::kPanel);
    graphics.drawLine(0, 282, DisplayLayout::kLogicalWidth - 1, 282, theme::kGridDim);
    const Rect decrease = DisplayLayout::rangeDecreaseBounds();
    const Rect increase = DisplayLayout::rangeIncreaseBounds();
    graphics.strokeRect(decrease.x, decrease.y, decrease.width, decrease.height, theme::kGridDim);
    graphics.strokeRect(increase.x, increase.y, increase.width, increase.height, theme::kGridDim);
    graphics.drawText(decrease.x + 20, decrease.y + 8, "-", theme::kPrimary, 2);
    graphics.drawText(increase.x + 20, increase.y + 8, "+", theme::kPrimary, 2);
    graphics.drawText(116, 299, "RNG " + std::to_string(state.display_range_mm / 1000U) + "M", theme::kText);

    const std::string distance = formatDistance(nearest_range_mm);
    const int distance_width = static_cast<int>(distance.size()) * 12;
    graphics.drawText((DisplayLayout::kLogicalWidth - distance_width) / 2, 291, distance, theme::kTargetCore, 3);
    graphics.drawText(422, 299, "MOT " + std::to_string(visible_contact_count), theme::kText);

    drawCrtOverlay(graphics);
}

}  // namespace n203::display
