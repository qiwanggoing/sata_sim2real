# Deploy-an-RL-policy-on-the-Unitree-Go2-robot
This repository provides a framework for low-level control of a legged robot (Unitree Go2), using ROS 2 as the communication middleware. The MuJoCo simulator is used to validate the control policy in simulation. Once the policy performs well in MuJoCo, you can deploy it on the real robot by simply setting the ROS parameter is_simulation to false. Also a base velocity estimator using extended Karman Filter is provided to estimate the velocity of base. 

## Environment
- **Ubuntu**: 20.04
- **ROS 2**: Foxy
- **MuJoCo**: 3.2.3
- **Python**: 3.8.20
- **Pinocchio**: 3.4.0

## Access Robot Sensor Data via ROS 2
Refer to the official Unitree ROS 2 repository for setup and examples:
[unitree ros2](https://github.com/unitreerobotics/unitree_ros2)

### Potential Build Issue
When building the ROS 2 packages, you may encounter the following error:
```bash
ModuleNotFoundError: No module named 'unitree_go.unitree_go_s__rosidl_typesupport_c’
```
or 
``` bash
rosidl_generator_py.import_type_support_impl.UnsupportedTypeSupport: Could not import 'rosidl_typesupport_c' for package 'unitree_go’
```
This issue occurs because **ROS 2 Foxy supports only Python 3.8** for generating C-based type support modules. If a different Python version is used, the build process may still succeed, but the runtime will **fail to locate the generated files**, resulting in import errors when executing ROS 2 commands.

### Solution
Manually set your Python version to **3.8** when building the workspace. For example:
```bash
export PYTHON_EXECUTABLE=/usr/bin/python3.8
colcon build --symlink-install
```
Make sure Python 3.8 is installed and available at the specified path.

## Simulation

