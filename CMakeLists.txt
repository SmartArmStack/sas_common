cmake_minimum_required(VERSION 3.8)
project(sas_common)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(pybind11 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

##### CPP LIBRARY #####

add_library(${PROJECT_NAME} INTERFACE
    )

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp)

install(
    DIRECTORY include/
    DESTINATION include
    )

install(
    TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    LIBRARY DESTINATION lib
    #ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
    )

##END## CPP LIBRARY #####

########## PYBIND11 Module ###############
ament_python_install_package(${PROJECT_NAME})

pybind11_add_module(_${PROJECT_NAME} SHARED
    src/sas_common_py.cpp)
ament_target_dependencies(_${PROJECT_NAME} PUBLIC
    rclcpp)

target_link_libraries(_${PROJECT_NAME} PRIVATE ${PROJECT_NAME})

target_include_directories(_${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

install(TARGETS _${PROJECT_NAME}
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)
########## PYBIND11 Module ###############

##### CPP BINARY #####

add_executable(sas_common_ros2_parameter_test_node
   src/examples/sas_common_ros2_parameter_test_node.cpp
)

ament_target_dependencies(sas_common_ros2_parameter_test_node PUBLIC
    rclcpp)

target_link_libraries(sas_common_ros2_parameter_test_node PUBLIC
    ${PROJECT_NAME}
    )

target_include_directories(sas_common_ros2_parameter_test_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

install(TARGETS
  sas_common_ros2_parameter_test_node
  DESTINATION lib/${PROJECT_NAME})

##END## CPP BINARY #####

##### LAUNCH FILES #####

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

##END## LAUNCH FILES #####

ament_package()
