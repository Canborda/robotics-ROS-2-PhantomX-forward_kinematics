## robotics_lab2
# KEYOP PHANTOMX ROBOT WITH ROS


This repository shows how to connect and operate via keyboard a Phantom X robot with ROS.

> ## Authors
> 
> - Camilo Andrés Borda Gil
> - Edwin Alfredo Higuera Bustos

<br>

---

<br>

## How to Use the Package

The first thing to do is to clone this repository (inside your catkin workspace) and build the package for ROS:

```
git clone https://github.com/Canborda/robotics_lab2.git
catkin build robotics_lab2
source devel/setup.bash
```

Make sure you have those python libraries installed (we use them for the `keyop_node`):
- [Py console](https://pypi.org/project/py-console/)
- [Pynput](https://pypi.org/project/pynput/)

To launch the package you have two modes: you can test it with a real [PhantomX Robot Arm](https://www.trossenrobotics.com/p/PhantomX-Pincher-Robot-Arm.aspx) in the __controller mode__ or just visualize the robot in Rviz in the __simulation mode__.

<br>

### __Simulation Mode__

<br>

```
roslaunch robotics_lab2 px_rviz_keyop.launch
```
For this mode you won't need anything additional to this package! It will start the Rviz interface with a model of the _PhantomX robot_ and a cli to control the different joints position.

> Demonstration

![simulation mode package demonstration](./assets/simulation_mode.gif)

<br>

### __Controller Mode__

<br>

```
roslaunch robotics_lab2 px_controllers.launch
```
This command will start several nodes:
- The `dynamixel_workbench_controllers` node allows the communication between ROS and the robot.
- The `px_keyop` node starts a pretty cli so you can select a joint to move or change between five predefined positions.
- The `joint_states_translator` node just receives all the joints position in radians from the keyop node, and maps them into the bit-level units for the dynamixel package.
- The `rviz` node for __real time visualization__ of your robot configuration.

> Demonstration

![ADD DEMONSTRATION HERE!]()

<br>

---

<br>