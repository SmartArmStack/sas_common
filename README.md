# sas_common

> [!TIP]
> Repository for this module: https://github.com/SmartArmStack/sas_common \
> More information about SmartArmStack is available in https://smartarmstack.github.io/.

## ROS 2 Nodes

The ROS 2 nodes are defined in `CMakeLists.txt`, with example implementations located in the `src/examples/` directory. The package also includes a Python wrapper test script in the `scripts/` directory.

### sas_common_ros2_parameter_test_node

This example and test node demonstrates the behaviour of `sas::get_ros_parameter`. It is intended to validate ROS 2 parameter parsing, including the package-specific handling of empty lists encoded as `["EMPTY_LIST"]`.

Source file: `src/examples/sas_common_ros2_parameter_test_node.cpp`

Run the node with:

```bash
ros2 run sas_common sas_common_ros2_parameter_test_node
```

### sas_object_test_node

This test node validates the `sas::ObjectClient` and `sas::ObjectServer` interfaces. It can be used to check that object client-server communication is working as expected within the `sas_common` package.

Source file: `src/examples/sas_object_test_node.cpp`

Run the node with:

```bash
ros2 run sas_common sas_object_test_node
```

### sas_simulator_test_node

This test node validates the `sas::SimulatorClient` and `sas::SimulatorServer` interfaces. It is useful for confirming that simulator client-server communication is configured and operating correctly.

Source file: `src/examples/sas_simulator_test_node.cpp`

Run the node with:

```bash
ros2 run sas_common sas_simulator_test_node
```

### test_python_wrapper.py

This Python script tests the `pybind11` Python bindings provided by the package. It is located in the `scripts/` directory and can be executed through ROS 2 as a package script.

Script location: `scripts/test_python_wrapper.py`

Run the script with:

```bash
ros2 run sas_common test_python_wrapper.py
```

## Launch Files

The launch file(s) are located in the `launch/` directory. 

### sas_common_ros2_parameter_test_launch.py

This launch file starts `sas_common_ros2_parameter_test_node` with a predefined set of parameters. It is designed to test ROS 2 parameter parsing, including the handling of empty lists encoded as `["EMPTY_LIST"]`.

Launch file location: `launch/sas_common_ros2_parameter_test_launch.py`

Run the launch file with:

```bash
ros2 launch sas_common sas_common_ros2_parameter_test_launch.py
```
