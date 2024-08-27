cmake_minimum_required(VERSION 3.8)
project(mas_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(mas REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# find_package(rpi_gpio REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Motor.action"
)

# install(PROGRAMS
#   scripts/server.py
#   scripts/client.py
#   DESTINATION lib/${PROJECT_NAME}
# )



ament_package()
