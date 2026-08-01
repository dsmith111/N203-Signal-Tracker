cmake_minimum_required(VERSION 3.20)

project(n203_signal_tracker LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(n203_radar_core
    components/radar/core/src/frame_validation.cpp)
target_include_directories(n203_radar_core PUBLIC
    components/radar/core/include)

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
    tests/unit/display/display_app_test.cpp)
target_link_libraries(n203-unit-tests PRIVATE n203_display n203_radar_core)
add_test(NAME n203-unit-tests COMMAND n203-unit-tests)
