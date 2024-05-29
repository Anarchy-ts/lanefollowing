cmake_minimum_required(VERSION 3.8)
project(lanefollowing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)

ament_python_install_package(${PROJECT_NAME})

#Build
# add_executable(lanefollowing src/lanefollowing.cpp)
# target_include_directories(lanefollowing PUBLIC ${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME})
# ament_target_dependencies(lanefollowing rclcpp cv_bridge sensor_msgs geometry_msgs OpenCV)

add_executable(initpoint src/lanefollower/initpoint.cpp)
ament_target_dependencies(initpoint rclcpp sensor_msgs cv_bridge OpenCV)

add_executable(midpoint src/lanefollower/midpts.cpp)
ament_target_dependencies(midpoint rclcpp sensor_msgs cv_bridge OpenCV)

add_executable(alignpt src/lanefollower/test.cpp)
ament_target_dependencies(alignpt rclcpp sensor_msgs cv_bridge OpenCV)

add_executable(final src/lanefollower/final.cpp)
ament_target_dependencies(final rclcpp sensor_msgs geometry_msgs cv_bridge OpenCV)

add_executable(avoid src/obstacles/avoid.cpp)
ament_target_dependencies(avoid rclcpp sensor_msgs geometry_msgs nav2_msgs nav2_util cv_bridge OpenCV)



install(
  DIRECTORY 
    src/config/
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS 
    src/obstacles/robot_navigator.py
    src/obstacles/nav2pose.py
    src/obstacles/ahead.py
    src/obstacles/switch.py
    src/obstacles/stop.py
    src/obstacles/initial.py
    src/obstacles/aligner.py
    src/lanefollower/pixelcoord.py
    src/lanefollower/detector.py
    src/lanefollower/mid_ipm.py
    src/lanefollower/lanegoal.py
    src/lanefollower/lane_align.py
    src/lanefollower/follow.py
    src/lanefollower/vect.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  midpoint
  initpoint
  final
  avoid
  alignpt
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
