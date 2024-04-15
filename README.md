# SLAM and Autonomous Navigation using ROS and Mecanum Drive

![Robot in Action](https://github.com/rajeev-gupta-bashrc/MECANUM_DRIVE/blob/Upload-images-for-user-guide/images/mecanum1.png)

Welcome to the **SLAM and Autonomous Navigation using ROS and Mecanum Drive** project repository! This project demonstrates the integration of Simultaneous Localization and Mapping (SLAM) techniques with the powerful ROS `move_base` package, enabling autonomous navigation for a robot equipped with a 4-wheeled Mecanum Drive system. The project also incorporates Mecanum kinematics to achieve precise and omnidirectional movement, allowing the robot to traverse complex environments effortlessly.

## Introduction
In this project, we showcase the fusion of advanced robotics concepts. We leverage the capabilities of the Robot Operating System (ROS) to integrate SLAM techniques with the `move_base` navigation stack, empowering our robot to autonomously navigate while simultaneously building a map of its surroundings. The unique Mecanum Drive system enhances the robot's mobility, enabling it to move freely in any direction.

## Features
- **ROS-Powered Navigation:** We harnessed ROS and the `move_base` package for robust and adaptable autonomous navigation.
- **SLAM Integration:** The project incorporates the ROS SLAM package to concurrently map the environment and accurately localize the robot.
- **4-Wheeled Mecanum Drive:** We've seamlessly integrated a Mecanum Drive system, enabling omnidirectional movement. Mecanum kinematics are utilized to control the individual wheel velocities and achieve precise motion.
- **Obstacle Avoidance:** The project implements obstacle detection and avoidance strategies to ensure safe and efficient navigation.

## Getting Started
To experience this project on your local machine, follow these steps:
1. Install ROS Noetic by following the step-by-step tutorial provided here: [ROS Noetic Installation](https://wiki.ros.org/noetic/Installation/Ubuntu).
2. Create a ROS workspace:
    ```
    mkdir -p your_ws/src
    cd your_ws
    catkin_make
    ```
3. Install the required dependencies:
      ```
      sudo apt-get install ros-noetic-joy ros-noetic-teleop-twist-joy \
      ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
      ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
      ros-noetic-rosserial-python ros-noetic-rosserial-client \
      ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
      ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
      ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz \
      ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers 
      ```
4. Clone the project repository into your workspace:
    ```
    cd ~/your_ws/src
    git clone https://github.com/Rajeev-Gupta555/MECANUM_DRIVE.git
    cd ..
    catkin_make
    ```

## Usage
Follow these steps to use the project:
1. Launch the mecanum robot simulation in Gazebo:
    ``` 
    roslaunch mecanum main.launch
    ```
2. Open a new terminal and launch the mecanum controller:
    ```
    roslaunch mecanum mecanum_ctrl.launch
    ```
    You should observe X, Y, and yaw output in the terminal.
3. For autonomous navigation within a pre-saved map:
    Open another terminal
    ```
    roslaunch mecanum navigation_mecanum.launch
    ```
    An RViz window will open, displaying the pre-saved map and the mecanum robot. You can publish a navigation goal using RViz for the robot to autonomously navigate to.

### Launching SLAM and Navigation
To run SLAM or create your own map file, modify the world file in the `gazebo.launch` file with your custom world file. Launch the `main.launch` and then, in a new terminal:
- Run: `roslaunch mecanum mapping.launch`
An RViz window will open, showing the robot and the laser data. To map the environment, you need to move the robot. Open a new terminal and run the teleop node.
    ```
    roslaunch mecanum send_cmd_vel.launch
    ```
This node is slightly different from usual teleop nodes; here, we'll be sending angles to the robot with respect to its x-axis in the anticlockwise direction. The robot will move holonomically along the direction specified by the angle. Here are the commands:
    - Press * and then enter an angle input in degrees, like *45 'enter'.
    - R for anticlockwise rotation and r for clockwise.
    - W to increase linear speed and X to decrease speed.
    - Press any other key to stop.

The robot will map the Gazebo world, and once mapping is complete, save your map by running:
    ``` 
    rosrun map_server map_saver -f ~/maps
    ```
Your map's image and `.yaml` file should be saved in the `maps` folder. If you want to launch the `navigation_mecanum.launch` with your saved map, then modify the `.yaml` file in the launch file with your own.

## Mecanum Kinematics
Mecanum wheels, also known as Ilon wheels or Swedish wheels, were invented in 1973 by Bengt Ilon, a Swedish engineer. They are a common type of omnidirectional wheel design. In this design, a series of free-moving rollers are attached to the hub at a 45° angle around the circumference.
![Robot in Action](https://github.com/rajeev-gupta-bashrc/MECANUM_DRIVE/blob/Upload-images-for-user-guide/images/mecanum_kinematics.png)

### Four Mecanum Omnidirectional Solution
![Robot in Action](https://github.com/rajeev-gupta-bashrc/MECANUM_DRIVE/blob/Upload-images-for-user-guide/images/mecanum_matrix.png)

## Contributing
---
Feel free to explore, learn, and contribute to the convergence of robotics, SLAM, and autonomous navigation using ROS and Mecanum Drive technology. Happy exploring! 🤖🌟
We encourage any contributions to expand the range of functionalities of this project.
