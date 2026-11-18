cmake_minimum_required(VERSION 3.20)

project(n203_signal_tracker LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(n203_radar_core
    components/radar/core/src/frame_validation.cpp)
target_include_directories(n203_radar_core PUBLIC
    components/radar/core/include)

add_library(n203_motion
    components/motion/src/motion_estimator.cpp)
target_include_directories(n203_motion PUBLIC
    components/motion/include)

add_library(n203_radar_protocol
    components/radar/protocol/src/radar_link.cpp)
target_include_directories(n203_radar_protocol PUBLIC
    components/radar/protocol/include)
target_link_libraries(n203_radar_protocol PUBLIC n203_radar_core)

add_library(n203_radar_simulated
    components/radar/simulated/src/simulated_radar_source.cpp)
target_include_directories(n203_radar_simulated PUBLIC
    components/radar/simulated/include)
target_link_libraries(n203_radar_simulated PUBLIC n203_radar_core)

add_library(n203_radar_rd03d
    components/radar/rd03d/src/position_tracker.cpp
    components/radar/rd03d/src/range_pipeline.cpp
    components/radar/rd03d/src/raw_frame_stream.cpp)
target_include_directories(n203_radar_rd03d PUBLIC
    components/radar/rd03d/include)
target_link_libraries(n203_radar_rd03d PUBLIC n203_radar_core)

add_library(n203_radar_c4001
    components/radar/c4001/src/presence_pipeline.cpp
    components/radar/c4001/src/range_pipeline.cpp
    components/radar/c4001/src/report_stream.cpp)
target_include_directories(n203_radar_c4001 PUBLIC
    components/radar/c4001/include)
target_link_libraries(n203_radar_c4001 PUBLIC n203_radar_core)

add_library(n203_display
    components/display/src/display_app.cpp
    components/display/src/layout.cpp
    components/display/src/radar_geometry.cpp
    components/display/src/radar_renderer.cpp
    components/display/src/widgets/range_control.cpp)
target_include_directories(n203_display PUBLIC
    components/display/include)
target_link_libraries(n203_display PUBLIC n203_radar_core)

add_executable(n203-display-emulator
    emulations/display/src/main.cpp
    emulations/display/src/mouse_touch.cpp
    emulations/display/src/raster_display_host.cpp
    emulations/display/src/scenario_source.cpp)
target_include_directories(n203-display-emulator PRIVATE
    emulations/display/include)
target_link_libraries(n203-display-emulator PRIVATE n203_display n203_radar_core)

if(WIN32)
    target_sources(n203-display-emulator PRIVATE
        emulations/display/src/win32_live_window.cpp)
    target_compile_definitions(n203-display-emulator PRIVATE
        N203_HAS_WIN32_LIVE=1)
    target_link_libraries(n203-display-emulator PRIVATE user32 gdi32)
endif()

enable_testing()
add_executable(n203-unit-tests
    tests/unit/radar/frame_validation_test.cpp
    tests/unit/radar/radar_link_test.cpp
    tests/unit/radar/c4001_presence_pipeline_test.cpp
    tests/unit/radar/c4001_range_pipeline_test.cpp
    tests/unit/radar/c4001_report_stream_test.cpp
    tests/unit/radar/rd03d_position_tracker_test.cpp
    tests/unit/radar/rd03d_range_pipeline_test.cpp
    tests/unit/radar/rd03d_raw_frame_stream_test.cpp
    tests/unit/radar/simulated_radar_source_test.cpp
    tests/unit/motion/motion_estimator_test.cpp
    tests/unit/display/display_app_test.cpp)
target_link_libraries(n203-unit-tests PRIVATE
    n203_display n203_motion n203_radar_c4001 n203_radar_core n203_radar_protocol
    n203_radar_rd03d n203_radar_simulated)
add_test(NAME n203-unit-tests COMMAND n203-unit-tests)

add_executable(n203-rd03d-soak-tests
    tests/integration/rd03d_pipeline_soak_test.cpp)
target_link_libraries(n203-rd03d-soak-tests PRIVATE
    n203_display n203_radar_core n203_radar_protocol n203_radar_rd03d)
add_test(NAME n203-rd03d-soak-tests COMMAND n203-rd03d-soak-tests)
set_tests_properties(n203-rd03d-soak-tests PROPERTIES
    LABELS "integration;soak"
    TIMEOUT 30)
