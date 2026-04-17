cmake_minimum_required(VERSION 3.10)

project(project)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

if(EXISTS "/usr/lib/aarch64-linux-gnu/cmake/opencv4/OpenCVConfig.cmake")
    set(OpenCV_DIR "/usr/lib/aarch64-linux-gnu/cmake/opencv4")
endif()
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc videoio)

execute_process(
    COMMAND ${Python3_EXECUTABLE} -c
            "import pathlib, onnxruntime; base = pathlib.Path(onnxruntime.__file__).resolve().parent / 'capi'; libs = sorted(base.glob('libonnxruntime.so*')); print(libs[0] if libs else '')"
    OUTPUT_VARIABLE ONNXRUNTIME_LIB
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

if(NOT ONNXRUNTIME_LIB)
    message(FATAL_ERROR "Failed to locate libonnxruntime.so from Python onnxruntime package")
endif()

get_filename_component(ONNXRUNTIME_LIB_DIR ${ONNXRUNTIME_LIB} DIRECTORY)
set(ONNXRUNTIME_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/third_party/onnxruntime/include)
set(PROJECT_THIRD_PARTY_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/third_party)

set(PROJECT_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/function/include)

set(PROJECT_CORE_SOURCES
    function/src/common/logger.cpp
    function/src/communication/uart.cpp
    function/src/control/Judgment.cpp
    function/src/control/auto_avoid.cpp
    function/src/control/to_stm.cpp
    function/src/web/log_dashboard_server.cpp
)

set(PROJECT_SENSOR_COMPONENT_SOURCES
    function/src/sensors/gps/gps_node.cpp
    function/src/sensors/imu/imu_node.cpp
    function/src/sensors/lidar/lidar_sensor.cpp
    function/src/sensors/camera/camera_node.cpp
)

add_library(project_core STATIC ${PROJECT_CORE_SOURCES})
target_include_directories(project_core
    PUBLIC
        ${PROJECT_INCLUDE_DIR}
        ${PROJECT_THIRD_PARTY_INCLUDE_DIR}
)
ament_target_dependencies(project_core
    ament_index_cpp
    rclcpp
    sensor_msgs
    std_msgs
)
target_link_libraries(project_core pthread)

add_library(project_sensor_components STATIC ${PROJECT_SENSOR_COMPONENT_SOURCES})
target_include_directories(project_sensor_components
    PUBLIC
        ${PROJECT_INCLUDE_DIR}
)
ament_target_dependencies(project_sensor_components
    geometry_msgs
    rclcpp
    sensor_msgs
    std_msgs
)
target_link_libraries(project_sensor_components pthread)

add_executable(project
    obj/main.cpp
)
target_include_directories(project
    PRIVATE
        ${PROJECT_INCLUDE_DIR}
)
ament_target_dependencies(project
    rclcpp
)
target_link_libraries(project
    project_core
)

add_executable(lsn10_serial_scan_node
    function/src/tools/lsn10_serial_scan_node.cpp
)
ament_target_dependencies(lsn10_serial_scan_node
    rclcpp
    sensor_msgs
)
target_link_libraries(lsn10_serial_scan_node pthread)

add_executable(imu_corrector_node
    function/src/tools/imu_corrector_node.cpp
)
target_include_directories(imu_corrector_node
    PRIVATE
        ${PROJECT_INCLUDE_DIR}
)
ament_target_dependencies(imu_corrector_node
    geometry_msgs
    rclcpp
    sensor_msgs
    std_msgs
)
target_link_libraries(imu_corrector_node
    project_sensor_components
    pthread
)

add_executable(rgb_yolo_detector
    function/src/tools/rgb_yolo_detector.cpp
)
target_include_directories(rgb_yolo_detector
    PRIVATE
        ${OpenCV_INCLUDE_DIRS}
        ${ONNXRUNTIME_INCLUDE_DIR}
)
ament_target_dependencies(rgb_yolo_detector
    rclcpp
    std_msgs
)
target_link_libraries(rgb_yolo_detector
    pthread
    ${OpenCV_LIBS}
    ${ONNXRUNTIME_LIB}
)

set_target_properties(project PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin
)
set_target_properties(lsn10_serial_scan_node PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin
)
set_target_properties(imu_corrector_node PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin
)
set_target_properties(rgb_yolo_detector PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin
    BUILD_RPATH ${ONNXRUNTIME_LIB_DIR}
    INSTALL_RPATH ${ONNXRUNTIME_LIB_DIR}
)

install(TARGETS
    project
    lsn10_serial_scan_node
    imu_corrector_node
    rgb_yolo_detector
    DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
    source/scripts/configure_h30_imu.py
    source/scripts/configure_g60_gps.py
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
    source/launch
    source/config
    source/web
    source/pt
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
