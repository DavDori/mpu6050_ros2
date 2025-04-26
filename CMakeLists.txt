cmake_minimum_required(VERSION 3.5)
project(mpu6050)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_library(I2C_LIBRARY i2c)

install(DIRECTORY 
  launch 
  config 
  DESTINATION share/${PROJECT_NAME}
)

add_library(mpu6050sensor include/${PROJECT_NAME}/mpu6050sensor.cpp)
target_include_directories(mpu6050sensor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(mpu6050sensor
  ${I2C_LIBRARY}
)

add_executable(mpu6050node src/mpu6050node)

target_link_libraries(mpu6050node 
  i2c 
  mpu6050sensor
)
target_include_directories(mpu6050node PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(mpu6050node 
  rclcpp 
  sensor_msgs
)

install(TARGETS 
  mpu6050node 
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
