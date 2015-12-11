## Idmind - ROS

###### Example
To run (complete system):
```
roslaunch no_build idmind_navigation.launch
```
or (only localization):
```
roslaunch no_build idmind_amcl.launch
```

###### Dependencies
* [serial](http://www.ros.org/browse/details.php?distro=indigo&name=serial) - serial library

###### Necessary packages
* [joy](http://wiki.ros.org/joy) - joystick/gamepad drivers
* [hokuyo_node](http://wiki.ros.org/hokuyo_node) - hokuyo laser drivers

**Important note**: udev rules must be set up correctly for the system to work properly. They can be found inside the *no_build* package.

---

### idmind_ancillary
This metapackage contains ancillary packages for the Idmind ROS software. Besides helper nodes, there are some tools and utilities for debugging and calibrating the robots.

#### idmind_obstacle_detection ####
This package contains nodes for detecting obstacles.

**1. laser_obstacle_detector**  
This node uses laser data to detect obstacles given an angle (used for defining a centered arc) and a threshold. When a reading below the threshold is detected inside the "pizza slice" a flag is rised.

  1. *Subscribed Topics*
    * scan (sensor_msgs/LaserScan)
  
  2. *Published Topics*
    * status (std_msgs/UInt8)  
      1 when an obstacle is detected, 0 when clear.
  
  3. *Parameters*
    * ~arc_angle (double, default: 80.0 degrees)
    * ~distance_threshold (double, default: 0.4 meters)

#### idmind_utilities ####
This package contains some tools and utilities for debugging and calibrating the robots.

**1. pose_initializer**  
Provides a service to publish a PoseWithCovarianceStamped to the topic *initialpose*.

  1. *Published Topics*
    * initialpose (geometry_msgs/PoseWithCovarianceStamped)

  2. *Services*
    * pose_init (idmind_utilities/PoseInit)
      (x, y, th) Positions in meters, orientation in radians.

**2. wheel_speed_calibration**  
Tool to calibrate the relationship between commanded and real wheel velocities.

  1. *Subscribed Topics*
    * idmind_motors/encoders (std_msgs/Int32MultiArray)

  2. *Published Topics*
    * idmind_motors/twist (geometry_msgs/Twist)

---

### idmind_robot
This metapackage integrates Idmind's robotic platforms with ROS. Currently it supports the 2wd and omni platforms. Further robots will be added in the future. The software is component based for various reasons, being the main ones reusability, manageable size and easier maintenance.

#### idmind_arms ####
This package controls the arms in those robots equipped with them. It sets them in different positions and torque modes. It is also in charge of bringing up the arms after they have been disconnected because of a power overload. Lastly, it publishes their power state.

1. *Subscribed Topics*
  * ~position (std_msgs/UInt16MultiArray)  
    (position_left, time_left, position_right, time_right) Positions 0-150 degrees. Times 0-2500 milliseconds.

2. *Published Topics*
  * ~status (std_msgs/UInt8MultiArray)  
    (power_left, power_right) 1 when there is torque control, 0 when the arm has been switched off.

3. *Services*
  * torque (idmind_arms/Torque)  
    (mode_left, mode_right) Mode can be 0x00 (OFF), 0x60 (ON), 0x40 (brake).

#### idmind_behaviours ####
This is an example package for controlling the different expressions of the social robots from Idmind. It needs the packages *idmind_arms* and *idmind_interaction*. It uses some configuration files found into the *no_build* package, under config/behaviours/.

1. *Published Topics*
  * idmind_interaction/head (std_msgs/UInt8MultiArray)
  * mouth_encoder/mouth_shape (std_msgs/UInt8)
  * idmind_interaction/lights (std_msgs/UInt8MultiArray)
  * idmind_arms/position (std_msgs/UInt16MultiArray)

#### idmind_imu ####
This package reads the inertial data from the IMU on the robots. It stablishes a serial link with the inertial sensor and translates the data to a topic. It can work in *stream* or *sample* mode. The streaming frequency is hard set to 100Hz. In *sample* mode the maximum frequency is ~50Hz.

1. *Published Topics*
  * ~inertial (std_msgs/Float64MultiArray)  
    (yaw, pitch, roll) In radians.

2. *Parameters*
  * ~mode (string, default: "sample")  
    Possible values are "sample" or "stream".
  * ~update_frequency (double, default: 20.0 Hz)  
    Only when in *sample* mode.

#### idmind_interaction ####
This package controls the mouth, lights, and head of the social robots from Idmind. It has an initialization option to test all the components, and shuts down everything when completed, making sure everything is at its default/OFF state.

**1. idmind_interaction**
This node establishes a connection with the interaction board. It controls the display of all LEDs, including the mouth, and the head's position and velocity. It publishes the position and mode of the head as well as the state of the capacitive sensors on the shell. It provides a service to switch the projector ON and OFF.

  1. *Subscribed Topics*
    * ~mouth (std_msgs/UInt8MultiArray)  
      32 binary states for every LED in the mouth. 1 is ON, 0 is OFF.
    * ~lights (std_msgs/UInt8MultiArray)  
      (Device number 0-5, red, green, blue, shift_time) Device 0-5, RGB 0-100 and time 0-255 centiseconds.
    * ~head (std_msgs/UInt8MultiArray)  
      (position, velocity) Position 0-180 side-to-side, velocity 0-100%.

  2. *Published Topics*
    * ~head_angle_mode (std_msgs/UInt8MultiArray)  
      (head_position, head_mode) Head mode becomes 1 when externally forced.
    * ~capacitive (std_msgs/UInt8MultiArray)  
      Five sensors. 1 when activated, 0 otherwise.

  3. *Services*
    * projector (idmind_interaction/Projector)  
      (switch) 1 for switching ON, 0 for switching OFF.

  4. *Parameters*
    * ~capacitive_true_positives (int, default: 5)  
      Number of consecutive positives to publish a hit for the respective capacitive sensor.
    * ~head_recovery_time (int, default: 10 seconds)  
      Time for the robot to wait before restoring normal operation of the head after having been externally manipulated. Maximum 30 seconds.
    * ~initialize (bool, default: false)  
      Wether or not to execute the initialization tests. These tests check the correct functioning of all the social expressions.

**2. mouth_encoder**
This node provides an easy way to control the mouth through pre-defined shapes. It reads a file with mouth designs located in the *no_build* package, under config/behaviours/, and stores them in memory. It then listens to the *mouth_shape* topic and sends the corresponding mouth to the *idmind_interaction* node.

  1. *Subscribed Topics*
    * ~mouth_shape (std_msgs/UInt8)  
      Number corresponding to a mouth design.

  2. *Published Topics*
    * idmind_interaction/mouth (std_msgs/UInt8MultiArray)

  3. *Parameters*
    * ~number_of_mouths (int, default: 5)  
      This parameter must correspond to the number of mouths in the text file.

#### idmind_motors ####
This package is the core of the locomotion drivers for the robotic platforms from Idmind. It publishes the encoder readings and passes on the velocity commands for every wheel. It also calculates the inverse kinematics for both differential and omnidirectional structures. It also includes an acceleration and velocity limitator for smoothing the movements. It provides two services, one for hard-stopping the robot and another for getting battery state information. Additionally, it has a "watchdog" built in which stops the robot when the twist messages stop coming after a specific time.

1. *Subscribed Topics*
  * ~twist (geometry_msgs/Twist)

2. *Published Topics*
  * ~encoders (std_msgs/Int32MultiArray)  
    In ticks. Size depending on the number of wheels.

3. *Services*
  * hardstop (idmind_motors/Hardstop)  
    (activate, hardstop_time, back_up, stay_bloqued) Activate can be 1 or 0 and hardstop_time is the number of seconds for the robot to be blocked. Back_up is an option for the robot to move backwards momentaneously after the hardstop and can be 1 or 0. Stay_bloqued leaves the robot still until the hardstop is deactivated and takes either 1 or 0.
  * voltages_status (idmind_motors/VoltagesStatus)  
    This service takes no arguments and displays a series of voltages and statuses from the electronic boards.

4. *Parameters*
  * idmind_robot/robot (string, default: "generic")  
    Can also be "2wd" or "omni", which are robots with special configurations.
  * idmind_robot/kinematics (string)  
    The type of kinematics of the robotic platform. Can be "differential" or "omnidirectional".
  * idmind_robot/wheels (int)  
    The number of wheels in the robot.
  * idmind_robot/wheel_radius (double)  
    The radius of the robot wheels in meters.
  * idmind_robot/base_width (double)  
    The base width for differential robots or equivalent base width (sum of half of each side) for omnidirectional robots. In meters.
  * idmind_robot/max_acc_v (double, default: 0.5)  
    Maximum translational acceleration in meters per squared second.
  * idmind_robot/max_acc_w (double, default: 2.0)  
    Maximum rotational acceleration in radians per squared second.
  * idmind_robot/acc_dec_factor (double, default: 5.0)  
    Relationship between accelerations and decelerations, used to calculate the latter.
  * idmind_robot/max_v (double, default: 1.0)  
    Maximum translational velocity in meters per second.
  * idmind_robot/max_w (double, default: 2.0)  
    Maximum rotational velocity in meters per second.
  * idmind_robot/velocity_offset (int, default: 0)  
    Offset in the velocity command.
  * idmind_robot/watchdog_time (double, default: 0.06)  
    Maximum time allowed without any velocity twist before stopping the robot. In seconds.

#### idmind_odometry ####
This package calculates the odometry based on the wheel encoders and the IMU. It works with differential and omnidirectional platforms. It publishes both, topic and transform, from "odom" to "base_link". There is the possibility of solely using the encoders when there is no IMU available.

1. *Subscribed Topics*
  * idmind_motors/encoders (std_msgs/Int32MultiArray)
  * idmind_imu/inertial (std_msgs/Float64MultiArray)

2. *Published Topics*
  * odom (nav_msgs/Odometry)

3. *Parameters*
  * ~use_imu (bool, default: true)  
    Wether or not to use the IMU for calculating the odometry. If false, the encoders' data will be used for calculating the rotations as well.

  Note: the following four parameters are explained in the *idmind_motors* package.
  * idmind_robot/kinematics (string)
  * idmind_robot/wheels (int)
  * idmind_robot/wheel_radius (double)
  * idmind_robot/base_width (double)

  * idmind_robot/ticks (int)  
    The number of wheel encoder ticks per revolution.

#### idmind_sensors ####
This package is the main interface with the sensors board in the robots. It provides a way of knowing the battery voltages and charging statuses as well as performing docking/undocking operations. It makes available the state of the power button which is critical for many functions. It also publishes the ground sensors data and, if configured, can send a "hardstop" when detecting a value below the minimum thresholds which can be set using parameters.

1. *Published Topics*
  * ~ground_sensors (std_msgs/Int32MultiArray)  
    (left, front_left, front_right, right)
  * idmind_motors/twist (geometry_msgs/Twist)
  * ~power_button (std_msgs/UInt8)  
    1 is ON, 0 is OFF.

2. *Services*
  * batteries (idmind_sensors/Batteries)  
    This service takes no arguments and displays a series of voltages and statuses from the electronics board.
  * dock_undock (idmind_sensors/DockUndock)  
    (control) 2 for undocking and exiting, 3 for entering and docking. There is a third option, 1, in which only the docking is executed, without the approach to the docking station. Use only in cases in which the undocking wasn't successful, the robot is still at the docking station, and it is necessary to dock again. Make sure the robot is sufficiently close to the docking station before starting a docking manouevre.

3. *Services Called*
  * hardstop (idmind_motors/Hardstop)

4. *Parameters*
  * ~fall_hardstop (bool, default: false)  
    Wether or not to send a hardstop when a ground sensor detects a value below the minimum threshold.

  Minimum threshold values for the ground sensors.
  * ~ground_min_left (int)
  * ~ground_min_front_left (int)
  * ~ground_min_front_right (int)
  * ~ground_min_right (int)

  Note: if "fall_hardstop" is true and these parameters are not set the node won't start.

#### idmind_serial ####
This package provides a wrapper for the *serial* ROS package. It offers different methods for reading and writing through serial connections. This package is used by all the other packages which have a direct connection to the electronic boards through serial (by USB). It makes sure the bus is clean and checks for the messages' checksums.

#### idmind_teleoperation ####
This package is an interface between the raw message provided by the *joy* package and the Idmind ROS software. It can be used to produce a twist from the joystick axes and, therefore, teleoperate the robots. It takes into account the state of the robot so as to only send commands when the robot is in manual mode. Furthermore, it facilitates sending a "hardstop" signal to the robot with only one joystick button, as well as commanding different state transitions through the "chatter" topic. Lastly, in case the joystick stopped, it zeroes the twist for stopping the robot.

1. *Subscribed Topics*
  * joy (sensor_msgs/Joy)
  * state (std_msgs/UInt8MultiArray)

2. *Published Topics*
  * idmind_motors/twist (geometry_msgs/Twist)
  * ~chatter (std_msgs/String)
    * b4 (button 4) + b5 + b1 = "move"
    * b2 = "stop"
    * b3 = "selfie"
    * b4 + b5 + b9 = "wake"
    * b4 + b5 + b8 = "sleep"
    * b0 = hardstop
    * a3 (axis 3) = twist_linear_x
    * a0 = twist_linear_y
    * a2 = twist_angular_z

3. *Services Called*
  * hardstop (idmind_motors/Hardstop)

---

### no_build ####
  This package homes different folders which do not need to be built. Everything which is not code and can be used by different nodes and/or robots.

##### config
* *behaviours*  
  Contains text files used for configuring different interactions.
* *costmap*  
  Configuration files for the navigation costmaps.
* *params*  
  Files with robot (hardware) configuration parameters.
* *udev_rules*  
  Udev rules for the different electronics and interfaces.
    
##### launch
  Necessary launch files to deploy the whole Idmind system. They are chained following the hierarchies below:

```
idmind_navigation:  
  idmind_amcl:  
    idmind_map  
    idmind_base:  
      lasers
```
```
idmind_amcl:  
  idmind_map  
  idmind_base:  
    lasers
```

Note: *idmind_odometry* can be used instead *idmind_amcl* for odometry only localization.

##### maps
  Different maps used for localization and navigation.

##### scripts
  Scripts used for executing system, usually hardware related, actions.

