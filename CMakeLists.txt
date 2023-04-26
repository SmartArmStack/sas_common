cmake_minimum_required(VERSION 3.8)
project(sas_common)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(pybind11 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/Float64.msg"
    "msg/Bool.msg"
    )

ament_export_dependencies(rosidl_default_runtime)

add_library(${PROJECT_NAME}_lib SHARED
    src/sas_common.cpp
    )

ament_target_dependencies(${PROJECT_NAME}_lib rclcpp)

target_include_directories(${PROJECT_NAME}_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_export_targets(export_${PROJECT_NAME}_lib HAS_LIBRARY_TARGET)

########## PYBIND11 Module ###############
pybind11_add_module(_${PROJECT_NAME} SHARED src/sas_common_py.cpp)

target_link_libraries(_${PROJECT_NAME} PRIVATE ${PROJECT_NAME}_lib)

install(TARGETS _${PROJECT_NAME}
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)
########## PYBIND11 Module ###############

install(
    DIRECTORY include/
    DESTINATION include
    )

install(
    TARGETS ${PROJECT_NAME}_lib
    EXPORT export_${PROJECT_NAME}_lib
    LIBRARY DESTINATION lib
    #ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
    )


ament_package()
