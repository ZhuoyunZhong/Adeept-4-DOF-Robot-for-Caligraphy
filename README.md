# Adeept-4-DOF-Robot-for-Caligraphy

**Author**:  WPI RBE 501 2020 Spring Team: [Jessica  Herman](https://github.com/KatConroy57), [Xinxiao Li](https://github.com/thejose5), [Anqi Shen](https://github.com/joeyzhong90595), [Zhuoyun Zhong](https://github.com/joeyzhong90595).

![robot_graph](demo/robot.png)

## Install & Build

Environment: Ubuntu 18.04 + ROS Melodic

To install gazebo controllers:

`sudo apt-get install ros-melodic-joint-state-controller`
`sudo apt-get install ros-melodic-effort-controllers`
`sudo apt-get install ros-melodic-position-controllers`

Build Instruction

1. Clone this repository under **catkin_ws/src/**
2. Compile under **catkin_ws**: `catkin_make`
3. Go to path: **catkin_ws/src/Adeept-4-DOF-Robot-for-Caligraphy/adeept_command/src/**

## Launch the Adeept Robot

To see the robot in rviz:

`roslaunch adeept_description adeept_rviz.launch`

To generate the robot in gazebo:

`roslaunch adeept_gazebo adeept_world.launch`

To launch all the command nodes:

`roslaunch adeept_command command.launch`


## Project Detail

@TODO

The code Implemented three nodes including a forward kinematic, an inverse kinematic and a connector. After opening all the nodes, there will be some services. The first two nodes provide inverse kinematic and forward kinematic calculation.

`rosservice call inv_kin x, y, z, phi, theta, psi` Integration

`rosservice call for_kin q1, q2, d3 `

The connector builds bridges for connecting gazebo robot and kinematic nodes. The service it provides does not need any input but requires that the robot in gazebo is working. It takes the pose/joint variables of the robot in gazebo and calls `compute_ik`/`compute_fk` to compute the result. It will give both the computed joint variables/pose result and gazebo joint variables/pose data for comparison. If the error is less than 0.01, one could confirm that the nodes are working correctly.

`rosservice call check_ik` 

`rosservice call check_fk` 

---

The code also implemented and tuned a PD position controller. For better performance, we tuned the parameters in a new Adeept robot which has only one joint working and all the others fixed.

Generate Adeept 2 in gazebo:

`roslaunch adeept_gazebo adeept2_world.launch`

This time only joint 3 can be controlled and the available range for joint 3 is from 0 to 0.3 meters.

To give a reference position to the PD controller of joint 3:

`rosservice call set_joint_pos_ref '{joint_name: joint3, ref: 0.2}'`  

One should be able to see the joint 3 moves to the desired position.

To tuned the value and see the result in real time, suggest using `rqt`. More detail of tunning process can refer to [Gazebo Control Tutorial](http://gazebosim.org/tutorials?tut=ros_control).

Repeat this process for all the three joints. After getting reasonable controller parameters, change the PD values in the file `adeept_control/config/adeept_control.yaml`

---

The final part first implemented a node for forward and inverse velocity kinematic. The node provides inverse velocity kinematic and forward velocity kinematic calculation.

`rosservice call vel_inv_kin q1, q2, q3, x, y, z, Vx, Vy, Vz, Wx, Wy, Wz` 

`rosservice call vel_for_kin q1, q2, q3, q1_dot, q2_dot, q3_dot `

Then is to implement and tune a velocity controller for the robot.

Similar to project 2 mentioned above, use a robot with only one joint movable to tune the controller.

`roslaunch adeept_gazebo adeept2_world.launch`

The default controller of the robot is the position controller. In order to use a velocity controller, we need to use `controller manager`of ROS to switch from position controller to velocity controller.

`rosservice call /adeept/controller_manager/switch_controller '{start_controllers: [joint3_velocity_controller], stop_controllers: [joint3_position_controller], strictness: 2}`

To give a reference speed to the PD controller:

`rosservice call set_joint_vel_ref '{joint_name: joint3, ref: 1}'`  

One should be able to see the joint 3 moves at desired speed and ends up at the position of 0.3.

To tuned the value and see the result in real time, suggest using `rqt`. More detail of tunning process can refer to [Gazebo Control Tutorial](http://gazebosim.org/tutorials?tut=ros_control).

Repeat this process for all the three joints. After getting reasonable controller parameters, change the PD values in the file `adeept_control/config/adeept_control.yaml`

#### Integration

---

The final goal of the this project is to combine inverse position and inverse velocity kinematics with position and velocity controllers. In this way, when one gives the robot a Cartesian coordinate or a desired speed of the end of effector, the robot could figure out how to react using inverse kinematics, which is usually what people would like the robots to do.

First, as mentioned before, position controllers and velocity controllers are two different controllers, one can only use one of them for controlling one joint. To switch controller of all the three joints between position controllers and velocity controllers, one can use the server provided by `switch_control.py`. The parameter "p2v" represents changing from position controller to velocity controller and "v2p" plays a similar role.

`rosservice call switch_control 'p2v'` 

`rosservice call switch_control 'v2p'` 

As mentioned above, the robot could move following given joint velocity or position using two different controllers:

`rosservice call set_joint_pos_ref 'joint_name' pos_ref` 

`rosservice call set_joint_vel_ref 'joint_name' vel_ref` 

Now combined with inverse kinematics, it could also find its way given a desired world coordinates or desired Cartesian velocity.

`rosservice call set_cartesian_pos_ref x y z` 

`rosservice call set_cartesian_vel_ref Vx Vy Vz Wx Wy Wz` 

Noted that if the robot could not go to an invalid coordinates (x, y, z), it would not move and a "false" will be given. For the velocity one, the joint speed limitation is set to about 0.4 due to safety setting by velocity controller. When the joint speed given to joints exceeds that, it will stay in 0.4, ending up an undesired trajectory.

A valid example would be:

`rosservice call set_cartesian_pos_ref "{x: 0.4, y: 0, z: 0.4}"` `

`rosservice call switch_control 'p2v'`

`rosservice call set_cartesian_vel_ref "{Vx: 0, Vy: 0.1, Vz: 0, Wx: 0, Wy: 0, Wz: 0}"`

One should be able to see that the robot moves to (0.4, 0, 0.4) in the world coordinate. Then it moves along the +y direction for 3 seconds at a speed of 0.1.

## Node graph:

![node_graph](demo/node_graph.png)
