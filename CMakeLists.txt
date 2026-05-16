cmake_minimum_required(VERSION 3.20)

project(sas_common)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()

#################################
# pybind11 import block [BEGIN] #
# vvvvvvvvvvvvvvvvvvvvvvvvvvvvv #
set(PYBIND11_FINDPYTHON ON) # Fix CMP0148 Warning. https://github.com/pybind/pybind11/issues/4785
add_subdirectory(pybind11)
# ^^^^^^^^^^^^^^^^^^^^^^^^^^^ #
# pybind11 import block [END] #
###############################

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sas_core REQUIRED)
find_package(sas_msgs REQUIRED)
find_package(sas_conversions REQUIRED)

####################################
# CPP Shared Library Block [BEGIN] #
# vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv #
# https://ros2-tutorial.readthedocs.io/en/latest/
# The most common use case is to merge everything you need to export
# into the same shared library called ${PROJECT_NAME}.


add_library(${PROJECT_NAME}
        src/sas_object_client.cpp
    )

#ament_target_dependencies(${PROJECT_NAME} rclcpp geometry_msgs sas_conversions)

target_include_directories(${PROJECT_NAME}
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
#ament_export_dependencies(rclcpp geometry_msgs sas_conversions)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
    -ldqrobotics
    rclcpp::rclcpp
    geometry_msgs::geometry_msgs
    sas_core::sas_core
    sas_conversions::sas_conversions
    )

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
# ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ #
# CPP Shared Library Block [END] #
##################################

########## PYBIND11 Module ###############
ament_python_install_package(${PROJECT_NAME})

pybind11_add_module(_${PROJECT_NAME} SHARED
    src/sas_common_py.cpp)

target_link_libraries(_${PROJECT_NAME} PRIVATE ${PROJECT_NAME})

target_include_directories(_${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

install(TARGETS _${PROJECT_NAME}
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)
########## PYBIND11 Module ###############

############################
# CPP Binary Block [BEGIN] #
# vvvvvvvvvvvvvvvvvvvvvvvv #
# https://ros2-tutorial.readthedocs.io/en/latest/
# While we cant use blocks https://cmake.org/cmake/help/latest/command/block.html#command:block
# we use set--unset
set(RCLCPP_LOCAL_BINARY_NAME sas_common_ros2_parameter_test_node)

add_executable(${RCLCPP_LOCAL_BINARY_NAME}
   src/examples/sas_common_ros2_parameter_test_node.cpp
)

target_link_libraries(${RCLCPP_LOCAL_BINARY_NAME} PRIVATE
    ${PROJECT_NAME}
    rclcpp::rclcpp
    )

target_include_directories(${RCLCPP_LOCAL_BINARY_NAME} PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

install(TARGETS ${RCLCPP_LOCAL_BINARY_NAME}
  DESTINATION lib/${PROJECT_NAME})

unset(RCLCPP_LOCAL_BINARY_NAME)
# ^^^^^^^^^^^^^^^^^^^^^^ #
# CPP Binary Block [END] #
##########################

############################
# CPP Binary Block [BEGIN] #
# vvvvvvvvvvvvvvvvvvvvvvvv #
# https://ros2-tutorial.readthedocs.io/en/latest/
# While we cant use blocks https://cmake.org/cmake/help/latest/command/block.html#command:block
# we use set--unset
set(RCLCPP_LOCAL_BINARY_NAME sas_object_test_node)

add_executable(${RCLCPP_LOCAL_BINARY_NAME}
        src/examples/sas_object_test_node.cpp
)

target_link_libraries(${RCLCPP_LOCAL_BINARY_NAME} PUBLIC
        ${PROJECT_NAME}
        rclcpp::rclcpp
)

target_include_directories(${RCLCPP_LOCAL_BINARY_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

install(TARGETS ${RCLCPP_LOCAL_BINARY_NAME}
        DESTINATION lib/${PROJECT_NAME})

unset(RCLCPP_LOCAL_BINARY_NAME)
# ^^^^^^^^^^^^^^^^^^^^^^ #
# CPP Binary Block [END] #
##########################

########################
# Launch Block [BEGIN] #
# vvvvvvvvvvvvvvvvvvvv #
# According to https://github.com/SmartArmStack/sas_robot_driver/blob/ros2/CMakeLists.txt
install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}/
)
# ^^^^^^^^^^^^^^^^^^ #
# Launch Block [END] #
######################

#########################
# Scripts Block [BEGIN] #
# vvvvvvvvvvvvvvvvvvvvv #
# https://ros2-tutorial.readthedocs.io/en/latest/
install(DIRECTORY
        scripts/
        FILE_PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
        DESTINATION lib/${PROJECT_NAME}
)
# ^^^^^^^^^^^^^^^^^^^ #
# Scripts Block [END] #
#######################

ament_package()
