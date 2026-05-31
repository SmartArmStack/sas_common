# sas_common

## ROS2 nodes (examples / test nodes)

Nodes are defined in `CMakeLists.txt` and in the
`scripts/` folder.

- `sas_common_ros2_parameter_test_node`
  - Source: `src/examples/sas_common_ros2_parameter_test_node.cpp`
  - Purpose: example/test node that demonstrates and validates `sas::get_ros_parameter` behavior
    (including the special handling of empty lists encoded as `["EMPTY_LIST"]`).

Run:
```bash
ros2 run sas_common sas_common_ros2_parameter_test_node
```

- `sas_object_test_node`
  - Source: `src/examples/sas_object_test_node.cpp`
  - Purpose: test `sas::ObjectClient` and `sas::ObjectServer`.

Run:
```bash
ros2 run sas_common sas_object_test_node
```

- `sas_simulator_test_node`
  - Source: `src/examples/sas_simulator_test_node.cpp`
  - Purpose: test `sas::SimulatorClient` and `sas::SimulatorServer`.

Run:
```bash
ros2 run sas_common sas_simulator_test_node
```

- Python wrapper script: `scripts/test_python_wrapper.py`
  - Location: `scripts/test_python_wrapper.py`
  - Purpose: test the pybind11 Python bindings.

Run:
```bash
ros2 run sas_common test_python_wrapper.py
```

## Launch files

Launch files are in the `launch/` folder.

- `launch/sas_common_ros2_parameter_test_launch.py`
  - Location: `launch/sas_common_ros2_parameter_test_launch.py`
  - Purpose: launches the `sas_common_ros2_parameter_test_node` with a set of
    parameters used to exercise parameter parsing (including empty-list handling)

Run:
```bash
ros2 launch sas_common sas_common_ros2_parameter_test_launch.py
```

