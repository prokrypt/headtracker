cmake_minimum_required(VERSION 3.16)
project(HeadTrackerBLE LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Path to the unzipped X-Plane SDK (the folder that contains CHeaders/).
set(XPLANE_SDK "${CMAKE_SOURCE_DIR}/SDK" CACHE PATH "X-Plane SDK directory")
if(NOT EXISTS "${XPLANE_SDK}/CHeaders/XPLM/XPLMPlugin.h")
  message(FATAL_ERROR "X-Plane SDK not found. Download it from developer.x-plane.com/sdk "
                      "and unzip it to ${XPLANE_SDK}, or pass -DXPLANE_SDK=/path/to/SDK")
endif()

# SimpleBLE (Bluetooth LE via BlueZ/D-Bus), built static into the plugin.
include(FetchContent)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
# Otherwise `cmake --install` also copies SimpleBLE's headers and static lib
# into Resources/plugins/{include,lib64}.
set(SIMPLEBLE_INSTALL OFF CACHE BOOL "" FORCE)
FetchContent_Declare(simpleble
  GIT_REPOSITORY https://github.com/simpleble/simpleble.git
  GIT_TAG v1.4.0
  GIT_SHALLOW TRUE
  SOURCE_SUBDIR simpleble)
FetchContent_MakeAvailable(simpleble)

add_library(HeadTrackerBLE SHARED src/plugin.cpp src/ble_link.cpp src/settings_ui.cpp)
target_include_directories(HeadTrackerBLE PRIVATE "${XPLANE_SDK}/CHeaders/XPLM" "${XPLANE_SDK}/CHeaders/Widgets")
target_compile_definitions(HeadTrackerBLE PRIVATE LIN=1 XPLM200=1 XPLM210=1 XPLM300=1 XPLM301=1 XPLM400=1)
target_compile_options(HeadTrackerBLE PRIVATE -fvisibility=hidden -Wall)
target_link_libraries(HeadTrackerBLE PRIVATE simpleble::simpleble)
# On Linux the XPLM symbols are resolved by X-Plane at load time (no link library needed).

set_target_properties(HeadTrackerBLE PROPERTIES PREFIX "" SUFFIX ".xpl" OUTPUT_NAME "HeadTrackerBLE")

# `cmake --install build` lays out the plugin folder X-Plane expects.
set(XPLANE_DIR "" CACHE PATH "X-Plane 12 install folder (for cmake --install)")
if(XPLANE_DIR)
  set(CMAKE_INSTALL_PREFIX "${XPLANE_DIR}/Resources/plugins" CACHE PATH "" FORCE)
endif()
install(TARGETS HeadTrackerBLE LIBRARY DESTINATION HeadTrackerBLE/lin_x64)
# Only install the example ini the first time, so a rebuild never overwrites
# the settings saved with "Save default" in the sim.
install(CODE "
  set(_ini \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/HeadTrackerBLE/head_tracker.ini\")
  if(EXISTS \"\${_ini}\")
    message(STATUS \"Keeping existing \${_ini}\")
  else()
    file(INSTALL \"${CMAKE_SOURCE_DIR}/head_tracker.ini\" DESTINATION \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/HeadTrackerBLE\")
  endif()
")
