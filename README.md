# sas_common

> [!TIP]
> Repository for this module: https://github.com/SmartArmStack/sas_common. <br/>
> More information about SmartArmStack is available in https://smartarmstack.github.io/.

## ROS 2 Nodes & Parameters

Example implementations are located in the `src/examples/` directory. The package also includes a Python wrapper test script in the `scripts/` directory.

The `sas_common_ros2_parameter_test_node` is launched through `sas_common_ros2_parameter_test_launch.py`, which loads parameters from `config/config.yaml` (pass a different file with `config_file:=/path/to/config.yaml`).

```bash
ros2 run sas_common sas_object_test_node
```

### Node: `sas_common_ros2_parameter_test_node`

| Property | Value |
|---|---|
| **Executable** | `sas_common_ros2_parameter_test_node` |
| **ROS node name** | `sas_common_ros2_parameter_test` (set by the `name` launch argument of `sas_common_ros2_parameter_test_launch.py`) |
| **Description** | Example/test node demonstrating the behaviour of `sas::get_ros_parameter`. It validates ROS 2 parameter parsing, including the package-specific handling of empty lists encoded as `["EMPTY_LIST"]`. |

#### Parameters

All eight parameters are optional — an absent vector parameter defaults to an empty list.

| Parameter | Type | Mandatory / Optional | Default | Purpose |
|---|---|---|---|---|
| `empty_string_vector` | string array | Optional | `[]` (empty) | Exercises the `["EMPTY_LIST"]` marker for an empty string vector |
| `empty_integer_vector` | integer array | Optional | `[]` (empty) | Exercises the `["EMPTY_LIST"]` marker for an empty integer vector |
| `empty_double_vector` | double array | Optional | `[]` (empty) | Exercises the `["EMPTY_LIST"]` marker for an empty double vector |
| `empty_bool_vector` | boolean array | Optional | `[]` (empty) | Exercises the `["EMPTY_LIST"]` marker for an empty boolean vector |
| `string_vector` | string array | Optional | `[]` (empty) | A non-empty string vector to test parsing |
| `integer_vector` | integer array | Optional | `[]` (empty) | A non-empty integer vector to test parsing |
| `double_vector` | double array | Optional | `[]` (empty) | A non-empty double vector to test parsing |
| `bool_vector` | boolean array | Optional | `[]` (empty) | A non-empty boolean vector to test parsing |

#### Sample launch

```bash
ros2 launch sas_common sas_common_ros2_parameter_test_launch.py
```

### sas_object_test_node

This test node validates the `sas::ObjectClient` and `sas::ObjectServer` interfaces. It can be used to check that object client-server communication is working as expected within the `sas_common` package.

```bash
ros2 run sas_common sas_object_test_node
```

### sas_simulator_test_node

This test node validates the `sas::SimulatorClient` and `sas::SimulatorServer` interfaces. It is useful for confirming that simulator client-server communication is configured and operating correctly.

```bash
ros2 run sas_common sas_simulator_test_node
```

### test_python_wrapper.py

This Python script tests the `pybind11` Python bindings provided by the package. It is located in the `scripts/` directory and can be executed through ROS 2 as a package script.

```bash
ros2 run sas_common test_python_wrapper.py
```

## Launch Files

The launch file(s) are located in the `launch/` directory. 

```bash
docker run --rm murilomarinho/sas:jazzy bash -c "ros2 launch sas_common sas_common_ros2_parameter_test_launch.py"
```

### sas_common_ros2_parameter_test_launch.py

This launch file starts `sas_common_ros2_parameter_test_node`, loading its parameters from `config/config.yaml` (or a file of your choice via `config_file:=/path/to/config.yaml`). It is designed to test ROS 2 parameter parsing, including the handling of empty lists encoded as `["EMPTY_LIST"]`.

```bash
ros2 launch sas_common sas_common_ros2_parameter_test_launch.py
```
