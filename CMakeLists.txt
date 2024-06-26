cmake_minimum_required(VERSION 3.8)
project(n10_edu_drive_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(${PROJECT_NAME}_node src/edu_drive_node.cpp src/EduDrive.cpp src/MotorController.cpp src/RPiAdapterBoard.cpp src/PowerManagementBoard.cpp src/can/canprotocol.cpp src/can/SocketCAN.cpp src/can/SocketCANObserver.cpp)

ament_target_dependencies(n10_edu_drive_ros2_node sensor_msgs std_srvs rclcpp)

install(
  DIRECTORY
    launch
    parameter
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  n10_edu_drive_ros2_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
