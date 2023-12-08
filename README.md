# PbD-With-Support
This repository contains the source code for an end-user robot programming system that can be used for programming a robot manipulator by kinesthetic demonstration; the system includes a customizable help menu and chat assistant to support end-users during programming. The system consists of a front-end web interface and back-end software that runs on Ubuntu 20.04 using [ROS](https://www.ros.org/) Noetic. 

The programming system is designed to be customized for use with different robots or different support content. We include instructions on how to customize this system in this README.

## Contents
- [frontend](https://github.com/intuitivecomputing/PbD-With-Support/tree/main/frontend): This directory contains the code for the front-end web interface. 
  - [index.html](https://github.com/intuitivecomputing/PbD-With-Support/blob/main/frontend/index.html) is the entry point for the front-end system.
- [backend](https://github.com/intuitivecomputing/PbD-With-Support/tree/main/backend): This directory contains the back-end code of the robot programming system.
  - The subdirectory [prog_support_backend](https://github.com/intuitivecomputing/PbD-With-Support/tree/main/backend/prog_support_backend) is the ROS package folder containing code that handles user program management and robot motion.
  - The subdirectory [prog_support_marker](https://github.com/intuitivecomputing/PbD-With-Support/tree/main/backend/prog_support_marker) is the ROS package folder containing code that handles user program visualization. 
- - - -

## Software and Hardware Requirements for Running the Programming System

### Front-End
The  front-end code has been deployed on a system running Ubuntu 20.04 (64-bit). The software was hosted using an [Apache](https://httpd.apache.org/download.cgi) server (version 2.4.41) and was run on [Mozilla Firefox for Ubuntu](https://www.mozilla.org/en-US/firefox/linux/) Version 117.0 (64-bit).

### Back-End
The back-end code has been deployed on a system running Ubuntu 20.04 (64-bit). The software uses [ROS Noetic](http://wiki.ros.org/noetic), which can be installed by following the instructions [here](http://wiki.ros.org/noetic/Installation).

### Hardware
This programming system requires the use of a robot with kinesthetic teaching capabilities and a gripper; you must have access to code that can activate/deactivate the robot's kinesthetic teaching mode and move the robot and its gripper. Instructions for how to set up the system for different robots that meet the requirements above are available below. 

### Required ROS Packages

[ros3djs](https://github.com/RobotWebTools/ros3djs)

[roslibjs](https://github.com/RobotWebTools/roslibjs)

- - - -

## Usage
To use the system, you will need to create a catkin workspace that contains the two back-end subdirectories (prog_support_backend and prog_support_marker), as well as the required ROS packages specified above and any packages required for the robot you are using (i.e., robot driver). You will first need to modify the backend to work with the robot you will be using and prepare JSON files specifying the content you would like the help menu and chat assistant to display.

### Modifying the Back-End
The file [template.cpp](https://github.com/intuitivecomputing/PbD-With-Support/blob/main/backend/prog_support_backend/src/template.cpp) includes starter code for the backend that handles program management and communication with the front-end interface. You will need to add code to the template for interfacing with the robot you are using (see the comments in the file for instructions). For an example of an instantiation of the template for the Kinova Gen3 7-DOF arm, please see [kinova_example.cpp](https://github.com/intuitivecomputing/PbD-With-Support/blob/main/backend/prog_support_backend/src/kinova_example.cpp). 

Once you are finished modifying the template, add the following lines to [CMakeLists.txt](https://github.com/intuitivecomputing/PbD-With-Support/blob/main/backend/prog_support_backend/CMakeLists.txt) inside the prog_support_backend package (remember to replace the placeholder below with your file name):
`add_executable(backend src/*replace with name of your file*.cpp)`

`target_link_libraries(backend ${catkin_LIBRARIES})`

`add_dependencies(
   backend
   ${${PROJECT_NAME}_EXPORTED_TARGETS}
 )`

You will also need to modify the file [marker_publisher.launch](https://github.com/intuitivecomputing/PbD-With-Support/blob/main/backend/prog_support_marker/launch/marker_publisher.launch) to specify the tf frames for your robot's base and end effector in the following lines:
`<!-- Change the base_frame name based on your robot type-->`

`<arg name="base_frame" default="base_link"/>`

`<!-- Change the tool_frame name based on your robot type-->`

`<arg name="tool_frame" default="tool_frame"/>`


Then, recompile your catkin workspace. 



To run the system, open a terminal on Ubuntu and run the following scripts:
- Run the driver for your robot.
- On a new tab or terminal, run the following script: `roslaunch prog_support_backend web_communication.launch`
- On a new tab or terminal, run the following script: `rosrun prog_support_backend backend`
- On a new tab or terminal, run the following script: `rosrun marker_package marker_publisher.py`

Then, open up a web browser and pull up the index.html file (or appropriate domain if you have hosted the frontend interface on the web).

- [icl_kortex](https://github.com/intuitivecomputing/icl_kortex)
