#include "n203/display/display_app.hpp"

#include "n203/display/layout.hpp"
#include "n203/display/radar_geometry.hpp"
#include "n203/display/widgets/range_control.hpp"

namespace n203::display {
namespace {
constexpr std::uint32_t kStaleAfterMs = 1200;
constexpr int kContactTouchRadiusSquared = 14 * 14;
}

void DisplayApp::acceptRadarFrame(const radar::RadarFrame& frame, std::uint32_t received_ms) {
    if (radar::validateFrame(frame) != radar::FrameValidationError::kNone) {
        invalid_frame_received_ = true;
        return;
    }

    invalid_frame_received_ = false;
    state_.frame = frame;
    last_frame_received_ms_ = received_ms;
    received_frame_ = true;

    if (state_.has_selected_contact) {
        bool selected_contact_exists = false;
        for (std::size_t index = 0; index < frame.contact_count; ++index) {
            if (frame.contacts[index].id == state_.selected_contact_id) {
                selected_contact_exists = true;
                break;
            }
        }
        if (!selected_contact_exists) {
            state_.has_selected_contact = false;
            state_.target_locked = false;
        }
    }
}

void DisplayApp::acceptInput(const DisplayInput& input) {
    if (input.type != InputType::kTouchReleased) {
        return;
    }

    if (DisplayLayout::rangeDecreaseTouchBounds().contains(input.point)) {
        state_.display_range_mm = RangeControl::decrease(state_.display_range_mm);
        return;
    }
    if (DisplayLayout::rangeIncreaseTouchBounds().contains(input.point)) {
        state_.display_range_mm = RangeControl::increase(state_.display_range_mm);
        return;
    }
    selectContactAt(input.point);
}

void DisplayApp::update(std::uint32_t now_ms) {
    updateScannerPresentation(now_ms);
}

void DisplayApp::selectContactAt(Point point) {
    const Rect radar_bounds = DisplayLayout::radarBounds();
    const Point radar_center = DisplayLayout::radarCenter();
    if (!radar_bounds.contains(point) || squaredDistance(point, radar_center) > DisplayLayout::radarRadius() * DisplayLayout::radarRadius()) {
        return;
    }

    for (std::size_t index = 0; index < state_.frame.contact_count; ++index) {
        const radar::Contact& contact = state_.frame.contacts[index];
        Point contact_point{};
        if (!radarPointForContact(contact, state_.display_range_mm, contact_point) ||
            squaredDistance(point, contact_point) > kContactTouchRadiusSquared) {
            continue;
        }

        if (state_.has_selected_contact && state_.selected_contact_id == contact.id) {
            state_.target_locked = !state_.target_locked;
        } else {
            state_.selected_contact_id = contact.id;
            state_.has_selected_contact = true;
            state_.target_locked = false;
        }
        return;
    }

    state_.has_selected_contact = false;
    state_.target_locked = false;
}

void DisplayApp::updateScannerPresentation(std::uint32_t now_ms) {
    if (invalid_frame_received_) {
        state_.scanner = ScannerPresentation::kInvalidFrame;
        return;
    }
    if (!received_frame_) {
        state_.scanner = ScannerPresentation::kStarting;
        return;
    }
    if (state_.frame.health == radar::RadarHealth::kOffline ||
        state_.frame.measurement_state == radar::MeasurementState::kOffline) {
        state_.scanner = ScannerPresentation::kOffline;
        return;
    }
    if (now_ms - last_frame_received_ms_ > kStaleAfterMs ||
        state_.frame.measurement_state == radar::MeasurementState::kStale) {
        state_.scanner = ScannerPresentation::kStale;
        return;
    }

    switch (state_.frame.measurement_state) {
        case radar::MeasurementState::kInitializing:
            state_.scanner = ScannerPresentation::kStarting;
            break;
        case radar::MeasurementState::kRangeOnly:
            state_.scanner = ScannerPresentation::kRangeOnly;
            break;
        case radar::MeasurementState::kPresenceOnly:
            state_.scanner = ScannerPresentation::kPresenceOnly;
            break;
        case radar::MeasurementState::kScanning:
            state_.scanner = ScannerPresentation::kScanning;
            break;
        case radar::MeasurementState::kStale:
            state_.scanner = ScannerPresentation::kStale;
            break;
        case radar::MeasurementState::kOffline:
            state_.scanner = ScannerPresentation::kOffline;
            break;
    }
}

}  // namespace n203::display
