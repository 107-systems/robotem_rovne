#######################################################################################
cmake_minimum_required(VERSION 3.8)
#######################################################################################
project(robotem_rovne)
set(ROBOTEM_ROVNE_TARGET ${PROJECT_NAME}_node)
#######################################################################################
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Werror -Wpedantic)
endif()
#######################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
#######################################################################################
add_subdirectory(external/mp-units/src)
#######################################################################################
add_executable(${ROBOTEM_ROVNE_TARGET}
  src/Node.cpp
  src/main.cpp
)
#######################################################################################
target_include_directories(${ROBOTEM_ROVNE_TARGET} PRIVATE
        include
)
#######################################################################################
target_compile_features(${ROBOTEM_ROVNE_TARGET} PRIVATE cxx_std_20)
#######################################################################################
target_link_libraries(${ROBOTEM_ROVNE_TARGET}
        ${rclcpp_LIBRARIES}
        mp-units::mp-units
)
#######################################################################################
ament_target_dependencies(${ROBOTEM_ROVNE_TARGET} rclcpp std_msgs sensor_msgs geometry_msgs tf2 tf2_geometry_msgs)
#######################################################################################
ament_export_dependencies(rosidl_default_runtime)
#######################################################################################
install(TARGETS ${ROBOTEM_ROVNE_TARGET} DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
#######################################################################################
ament_package()
#######################################################################################
