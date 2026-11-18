#include <cassert>

#include "n203/display/display_app.hpp"
#include "n203/display/radar_geometry.hpp"

void runDisplayAppTests() {
    n203::display::DisplayApp app;
    n203::radar::RadarFrame frame{};
    frame.sensor_timestamp_ms = 900000;
    frame.health = n203::radar::RadarHealth::kHealthy;
    frame.measurement_state = n203::radar::MeasurementState::kScanning;
    frame.capabilities.flags = n203::radar::kPosition | n203::radar::kRange;
    frame.motion = n203::radar::MotionState::kTracking;
    frame.motion_quality = 100;
    frame.contact_count = 1;
    frame.contacts[0].id = 7;
    frame.contacts[0].valid_fields = n203::radar::kContactPosition | n203::radar::kContactRange;
    frame.contacts[0].y_mm = 5000;

    app.acceptRadarFrame(frame, 100);
    app.update(100);
    assert(app.state().scanner == n203::display::ScannerPresentation::kScanning);
    assert(app.state().frame.motion == n203::radar::MotionState::kTracking);

    app.acceptInput(n203::display::DisplayInput::touchReleased({240, 146}));
    assert(app.state().has_selected_contact);
    assert(app.state().selected_contact_id == 7);
    assert(!app.state().target_locked);

    app.acceptInput(n203::display::DisplayInput::touchReleased({240, 146}));
    assert(app.state().target_locked);

    app.acceptInput(n203::display::DisplayInput::touchReleased({80, 296}));
    assert(app.state().display_range_mm == 15000);

    // The visible controls are inset, but their physical touch targets extend
    // through the bottom edge of the resistive panel.
    app.acceptInput(n203::display::DisplayInput::touchReleased({28, 318}));
    assert(app.state().display_range_mm == 10000);
    app.acceptInput(n203::display::DisplayInput::touchReleased({80, 319}));
    assert(app.state().display_range_mm == 15000);
    app.acceptInput(n203::display::DisplayInput::touchReleased({80, 319}));
    assert(app.state().display_range_mm == 20000);
    app.acceptInput(n203::display::DisplayInput::touchReleased({80, 319}));
    assert(app.state().display_range_mm == 25000);
    app.acceptInput(n203::display::DisplayInput::touchReleased({80, 319}));
    assert(app.state().display_range_mm == 25000);
    app.acceptInput(n203::display::DisplayInput::touchReleased({80, 320}));
    assert(app.state().display_range_mm == 25000);

    app.update(1301);
    assert(app.state().scanner == n203::display::ScannerPresentation::kStale);

    n203::display::Point projected{};
    assert(n203::display::radarPointForContact(frame.contacts[0], 10000, projected));
    n203::radar::Contact outside_field_of_view = frame.contacts[0];
    outside_field_of_view.x_mm = 5000;
    outside_field_of_view.y_mm = 1000;
    assert(!n203::display::radarPointForContact(outside_field_of_view, 10000, projected));

    n203::radar::Contact range_only{};
    range_only.valid_fields = n203::radar::kContactRange;
    range_only.range_mm = 1817U;
    assert(n203::display::radarPointForContact(range_only, 10000, projected));
    assert(projected.x == n203::display::DisplayLayout::radarCenter().x);
}
