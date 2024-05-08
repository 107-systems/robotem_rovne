#######################################################################################
cmake_minimum_required(VERSION 3.8)
#######################################################################################
project(robotem_rovne)
set(ROBOTEM_ROVNE_TARGET ${PROJECT_NAME}_node)
set(ROBOTEM_ROVNE_SRV_TARGET ${PROJECT_NAME})
#######################################################################################
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Werror -Wpedantic)
endif()
#######################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(mp-units REQUIRED)
find_package(rosidl_default_generators REQUIRED)
#######################################################################################
rosidl_generate_interfaces(${ROBOTEM_ROVNE_SRV_TARGET}
        "srv/AngularTarget.srv"
)
#######################################################################################
rosidl_get_typesupport_target(cpp_typesupport_target ${ROBOTEM_ROVNE_SRV_TARGET} "rosidl_typesupport_cpp")
#######################################################################################
add_executable(${ROBOTEM_ROVNE_TARGET}
  src/Node.cpp
  src/main.cpp
)
#######################################################################################
target_include_directories(${ROBOTEM_ROVNE_TARGET} PRIVATE
        include
        ${mp_units_INCLUDE_DIRS}
)
#######################################################################################
target_compile_features(${ROBOTEM_ROVNE_TARGET} PRIVATE cxx_std_20)
#######################################################################################
target_link_libraries(${ROBOTEM_ROVNE_TARGET}
        ${rclcpp_LIBRARIES}
        ${mp-units_LIBRARIES}
        ${cpp_typesupport_target}
)
#######################################################################################
ament_target_dependencies(${ROBOTEM_ROVNE_TARGET} rclcpp std_msgs std_srvs)
#######################################################################################
ament_export_dependencies(rosidl_default_runtime)
#######################################################################################
install(TARGETS ${ROBOTEM_ROVNE_TARGET} DESTINATION lib/${PROJECT_NAME})
#######################################################################################
ament_package()
#######################################################################################
