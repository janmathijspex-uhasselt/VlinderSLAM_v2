cmake_minimum_required(VERSION 3.5)

project(VlinderSLAM_v2 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include folders
include_directories(include)

# Source files
set(SOURCES
    main.cpp
    UDPSender.cpp
    ESKF.cpp
)

# Header files
set(HEADERS
    UDPMessage.h
    UDPSender.h
    PoseIO.h
    imudatareceiver.h
    ESKF.h
    unrolledFPFt.h
)

# Create executable
add_executable(VlinderSLAM_v2 ${SOURCES} ${HEADERS})

# Set opencv directory to opencv v4
set(OpenCV_DIR /usr/local/lib/cmake/opencv4)

# Required packages
find_package(stella_vslam REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(realsense2 REQUIRED)
find_package(pangolin_viewer REQUIRED)

# Include directories
target_include_directories(VlinderSLAM_v2 PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    /usr/local/include/DeviceFactory
)

# Find DeviceFactory library
find_library(DEVICE_FACTORY_LIB DeviceFactory PATHS /usr/local/lib)

# Link libraries
target_link_libraries(VlinderSLAM_v2 PRIVATE
    stella_vslam::stella_vslam
    ${OpenCV_LIBS}
    Eigen3::Eigen
    yaml-cpp
    ${DEVICE_FACTORY_LIB}
    realsense2::realsense2
    pangolin_viewer::pangolin_viewer
)

include(GNUInstallDirs)
install(TARGETS VlinderSLAM_v2
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# Copy assets folder to build directory
file(COPY ${CMAKE_SOURCE_DIR}/assets DESTINATION ${CMAKE_BINARY_DIR})
message(STATUS "Assets copied to ${CMAKE_BINARY_DIR}/assets")
