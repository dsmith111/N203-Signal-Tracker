#include <cassert>

#include "n203/display/display_app.hpp"
#include "n203/display/radar_geometry.hpp"

void runDisplayAppTests() {
    n203::display::DisplayApp app;
    n203::radar::RadarFrame frame{};
    frame.sensor_timestamp_ms = 0;
    frame.health = n203::radar::RadarHealth::kHealthy;
    frame.measurement_state = n203::radar::MeasurementState::kScanning;
    frame.capabilities.flags = n203::radar::kPosition | n203::radar::kRange;
    frame.contact_count = 1;
    frame.contacts[0].id = 7;
    frame.contacts[0].valid_fields = n203::radar::kContactPosition | n203::radar::kContactRange;
    frame.contacts[0].y_mm = 5000;

    app.acceptRadarFrame(frame);
    app.update(0);
    assert(app.state().scanner == n203::display::ScannerPresentation::kScanning);

    app.acceptInput(n203::display::DisplayInput::touchReleased({240, 146}));
    assert(app.state().has_selected_contact);
    assert(app.state().selected_contact_id == 7);
    assert(!app.state().target_locked);

    app.acceptInput(n203::display::DisplayInput::touchReleased({240, 146}));
    assert(app.state().target_locked);

    app.acceptInput(n203::display::DisplayInput::touchReleased({80, 296}));
    assert(app.state().display_range_mm == 15000);

    app.update(1201);
    assert(app.state().scanner == n203::display::ScannerPresentation::kStale);

    n203::display::Point projected{};
    assert(n203::display::radarPointForContact(frame.contacts[0], 10000, projected));
    n203::radar::Contact outside_field_of_view = frame.contacts[0];
    outside_field_of_view.x_mm = 5000;
    outside_field_of_view.y_mm = 1000;
    assert(!n203::display::radarPointForContact(outside_field_of_view, 10000, projected));
}
