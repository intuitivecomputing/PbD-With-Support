# PbD-With-Support
This repository contains the source code for an end-user robot programming system that can be used for programming a robot manipulator by kinesthetic demonstration; the system includes a customizable help menu and chat assistant to support end-users during programming. The system consists of a front-end web interface and back-end software that runs on Ubuntu 20.04 using [ROS](https://www.ros.org/) Noetic. 

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
The programming system may be used with any robot manipulator with kinesthetic teaching capabilities; instructions for how to set up the system for different robots are available below. 

### Required ROS Packages

