# ASV ROS package
### Install
Create the `ares_ros_ws` folder as workspace, in the  home directory (need to satisfy some path requirements). Copy in the `src` folder the `asv` package. 

#### Dependences
##### UUV Simulator
https://github.com/uuvsimulator/uuv_simulator.git 
> For Noetic compliance use:
> https://github.com/uuvsimulator/uuv_simulator/tree/66474bd3648f9fa2d0cf1d42e153ee08f3d3da7e 
#### ASV Wave Simulator
https://github.com/srmainwaring/asv_wave_sim.git
> For Noetic compliance use:
> https://github.com/srmainwaring/asv_wave_sim/tree/develop
#### Interface and solver for CG Optimization Problems
https://github.com/FrancoTor95/solver_test_py.git

### Run
Execute the simulation environment:
```bash
roslaunch asv sim_asv_gazebo.launch
```
Execute the thruster controller:
```bash
rosrun asv thrusters_manager.py
```
Send thrust command to the ASV (`Tx`=90N, `Ty`=`Tth`=0):
```bash
rostopic pub teleop/tau asv/Vector '{data: [90,0,0]}'
```

### Configuration
In `~/ares_ros_ws/src/asv_wave_sim/asv_wave_sim_gazebo/world_models/ocean_waves` folder, the `model.sdf` file contains the waves parameter to configure the main parameters of the waves field:
```xml
<wave>
    <number>3</number>
    <steepness>0.01</steepness>
    <scale>0.01</scale>
    <angle>0.0</angle>
    <amplitude>0.0</amplitude>
    <period>0.01</period>
    <direction>0 0</direction>
</wave>
```
and the main dynamic friction parameters of the water:
```xml
<!-- Linear and Angular Damping -->
<cDampL1>1.0E-6</cDampL1>
<cDampL2>1.0E-6</cDampL2>
<cDampR1>1.0E-6</cDampR1>
<cDampR2>1.0E-6</cDampR2>

<!-- 'Pressure' Drag -->
<cPDrag1>1.0E+2</cPDrag1>
<cPDrag2>1.0E+2</cPDrag2>
<fPDrag>0.4</fPDrag>
<cSDrag1>1.0E+2</cSDrag1>
<cSDrag2>1.0E+2</cSDrag2>
<fSDrag>0.4</fSDrag>
<vRDrag>1.0</vRDrag>
```
