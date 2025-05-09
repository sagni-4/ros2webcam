cmake_minimum_required(VERSION 3.8)
project(blue_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

include_directories(include ${OpenCV_INCLUDE_DIRS})

add_executable(color_filter src/color_filter.cpp)
ament_target_dependencies(color_filter rclcpp sensor_msgs cv_bridge)
target_link_libraries(color_filter ${OpenCV_LIBS})

install(TARGETS
  color_filter
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()


