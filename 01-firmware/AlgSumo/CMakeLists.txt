cmake_minimum_required(VERSION 3.8)
project(algsumo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Añadimos geometry_msgs aquí
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED) 

# 2. Asegúrate de que el nombre del archivo sea src/main.cpp
add_executable(algsumo src/main.cpp)

# 3. Añadimos geometry_msgs a las dependencias del ejecutable
ament_target_dependencies(algsumo 
  rclcpp 
  std_msgs 
  geometry_msgs
)

install(TARGETS
  algsumo
  DESTINATION lib/${PROJECT_NAME})

ament_package()

