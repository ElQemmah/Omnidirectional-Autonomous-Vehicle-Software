# Chelon ASV Core (Real-World)

ROS/Python implementation of the **Chelon** omnidirectional Autonomous Surface Vehicle (ASV) software stack for **real vehicles**.  
This repository contains the runtime code for onboard operation (Guidance–Navigation–Control), including GPS/IMU handling, controller/allocator, planners, and stereocamera-based obstacle monitoring.

> **Note:** This repository is intended for **real-vehicle deployment**.  

---

## Features
- **Navigation**: GPS/IMU ingestion and state publication (leveraging IMU onboard fusion).  
- **Guidance**: global waypoint planner (YAML missions) and a minimal local obstacle-aware planner.  
- **Control**: LTI controller (as presented in the paper, Sec. 3.5) and Σ⁺ thrust allocation for the four-thruster holonomic layout.  
- **Vision**: stereocamera subscriber (e.g., ZED 2i) to compute minimum obstacle distance in the field of view.  
- **Hardware**: thruster driver node for serial communication with ESCs.  
- **Bringup**: launch files and parameter configs under `asv/launch`.  
- **Reproducibility**: parameters identified in the paper (linear model and controller gains) provided in `config/`.  
- **Documentation**: additional notes in `docs/`.  

---

## Quickstart
### Dependencies
- ROS (Noetic recommended).  
- Python 3.  
- `rospy`, `sensor_msgs`, `geometry_msgs`, `std_msgs`.  
- ZED 2i ROS wrapper (if using the stereocamera).
- Battery Management System software is not included. Please visit repository: ([https://github.com/YOUR-ORG/chelon_asv_sim](https://github.com/RobotnikAutomation/daly_bms)).

### Running the stack
```bash
roslaunch asv asv_core_2.launch
```

## Repository Structure

```
CMakeLists.txt
README.md
action/
    Controller.action
docs/
    architecture.md
    hardware_setup.md
    overview.md
    quickstart.md
img/
    block_diagram.png
launch/
    asv_core_2.launch
    sim_asv_gazebo.launch
    sim_planner.launch
lib/
    WGS84.py
    __init__.py
    __pycache__/
        WGS84.cpython-36.pyc
        WGS84.cpython-38.pyc
    planner/
        __pycache__/
            border_planner.cpython-36.pyc
            border_planner.cpython-38.pyc
            line_planner.cpython-36.pyc
            line_planner.cpython-38.pyc
            planner.cpython-36.pyc
            planner.cpython-38.pyc
        border_planner.py
        line_planner.py
        planner.py
    test_planner.py
    ublib/
        F9P-C99 Fixed Base config.txt
        F9P-C99 Moving Base config.txt
        F9P-C99 Rover.txt
        WGS84.py
        __pycache__/
            ublox_interface.cpython-36.pyc
            ublox_interface.cpython-38.pyc
        list_usb_dev.bash
        pynmeagps/
            __init__.py
            __pycache__/
                __init__.cpython-38.pyc
                _version.cpython-38.pyc
                exceptions.cpython-38.pyc
                nmeahelpers.cpython-38.pyc
                nmeamessage.cpython-38.pyc
                nmeareader.cpython-38.pyc
                nmeatypes_core.cpython-38.pyc
                nmeatypes_get.cpython-38.pyc
                nmeatypes_poll.cpython-38.pyc
                nmeatypes_set.cpython-38.pyc
                socket_stream.cpython-38.pyc
            _version.py
            exceptions.py
            nmeahelpers.py
            nmeamessage.py
            nmeareader.py
            nmeatypes_core.py
            nmeatypes_get.py
            nmeatypes_poll.py
            nmeatypes_set.py
            socket_stream.py
        pyrtcm/
            __init__.py
            __pycache__/
                __init__.cpython-38.pyc
                _version.cpython-38.pyc
                exceptions.cpython-38.pyc
                rtcmhelpers.cpython-38.pyc
                rtcmmessage.cpython-38.pyc
                rtcmreader.cpython-38.pyc
                rtcmtypes_core.cpython-38.pyc
                rtcmtypes_get.cpython-38.pyc
                socket_stream.cpython-38.pyc
            _version.py
            exceptions.py
            rtcmhelpers.py
            rtcmmessage.py
            rtcmreader.py
            rtcmtypes_core.py
            rtcmtypes_get.py
            socket_stream.py
        pyubx2/
            __init__.py
            __pycache__/
                __init__.cpython-38.pyc
                _version.cpython-38.pyc
                exceptions.cpython-38.pyc
                socket_stream.cpython-38.pyc
                ubxhelpers.cpython-38.pyc
                ubxmessage.cpython-38.pyc
                ubxreader.cpython-38.pyc
                ubxtypes_configdb.cpython-38.pyc
                ubxtypes_core.cpython-38.pyc
                ubxtypes_get.cpython-38.pyc
                ubxtypes_poll.cpython-38.pyc
                ubxtypes_set.cpython-38.pyc
            _version.py
            exceptions.py
            socket_stream.py
            ubxhelpers.py
            ubxmessage.py
            ubxreader.py
            ubxtypes_configdb.py
            ubxtypes_core.py
            ubxtypes_get.py
            ubxtypes_poll.py
            ubxtypes_set.py
        ublox_interface.py
    vectlib/
        __pycache__/
            vectornav_interface.cpython-36.pyc
            vectornav_interface.cpython-38.pyc
            vectornav_packet.cpython-36.pyc
            vectornav_packet.cpython-38.pyc
        vectornav_interface.py
        vectornav_packet.py
license.txt
meshes/
    body.stl
    body_collision.stl
    propeller.stl
msg/
    External_data.msg
    Matrix.msg
    Vector.msg
package.xml
param/
    param.yaml
scripts/
    __pycache__/
        communication_bridge.cpython-38.pyc
        ublox_corrections.cpython-38.pyc
    all_data_collector.py
    daly_bms_node.py
    gps_imu_odometry.py
    thrusters_manager2.py
    ublox_gps.py
    vectornav_imu.py
    vehicle_controller.py
src/
    joystick_teleop.py
    planner_test.py
    planner_test_client.py
    plot.py
    plot_pool.py
    pool.py
    pool_1vehi.py
    pool_client.py
    test_client.py
urdf/
    asv.gazebo
    asv.urdf.xacro
    gps_imu.urdf.xacro
    neigh.urdf.xacro
    thruster.urdf.xacro
```

---

### Configuration
- Parameters can be changed in `ros/asv/config/params.yaml`.  
- Model parameters, control gains, and allocation matrix are in `config/`.  
- Missions can be defined in `config/waypoints.yaml`.  

---

## Documentation
See the `docs/` folder for:  
- `overview.md`: Project overview.  
- `architecture.md`: Node graph and description.  
- `quickstart.md`: Dependencies and usage.  
- `hardware_setup.md`: Hardware configuration (GPS, IMU, thrusters, stereo camera).  

---

## Citation
If you use this software, please cite the associated paper:

```
@article{Chelon2025,
  title   = {Linear System Identification and Control of a Low-Cost High-Performance Omnidirectional Marine Surface Vehicle for Swarming Applications},
  author  = {El Qemmah, Ayman and Cario, Gianni and Casavola, Alessandro and Lupia, Marco and Tedesco, Francesco},
  journal = {Journal of Field Robotics},
  year    = {2025},
  doi     = {DOI TBA}
}
```

---

## License
This software is licensed under the custom LaSa, DIMES license (see headers in source files).  

---
