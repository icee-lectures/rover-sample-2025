cmake_minimum_required(VERSION 3.8)
project(camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(aruco_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# ---- GStreamer を pkg-config で探す ----
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0)
pkg_check_modules(GSTAPP REQUIRED gstreamer-app-1.0)
pkg_check_modules(GSTBASE REQUIRED gstreamer-base-1.0)

include_directories(
  ${GST_INCLUDE_DIRS}
  ${GSTAPP_INCLUDE_DIRS}
  ${GSTBASE_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(camera src/camera.cpp)
add_executable(aruco_detector src/aruco_detector.cpp)

ament_target_dependencies(camera
  rclcpp
  sensor_msgs
)

ament_target_dependencies(aruco_detector
  rclcpp
  sensor_msgs
  aruco_msgs
)

target_link_libraries(camera
  ${GST_LIBRARIES}
  ${GSTAPP_LIBRARIES}
  ${GSTBASE_LIBRARIES}
)

target_link_libraries(aruco_detector
  ${OpenCV_LIBRARIES}
)

install(TARGETS camera
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS aruco_detector
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/)

ament_package()
