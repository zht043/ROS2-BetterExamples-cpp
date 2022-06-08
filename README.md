# ROS2-BetterExamples-cpp
A collection of ROS2 C++ code examples and tutorial docs

This repository contains C++ code examples for the ROS2(Robot Operating System 2) framework. The goal is to help beginners quickly learn ROS2 framework and how it can be applied on actual robots, therefore these examples are slightly more sophisticated, in a meaningful way with more usage variants/details, than the [official ROS2 rclcpp examples](https://github.com/ros2/examples/tree/master/rclcpp).
    

The examples are organized in separate packages such that it would be easy to copy-paste into your own project. Docs explaining ROS2 concepts/commands and step-by-step guides are also provided. 


[ROS2 official tutorial](https://docs.ros.org/en/foxy/Tutorials.html) (just for reference)
## Get Started 
This tutorial assumes you are already familiar with C++ and Linux. 
Though ROS2 is technically cross-platform, the instructions here are meant to run on Ubuntu 20.04.

Some dependencies to install in order to use full features of this repo:
```bash
sudo apt install build-essential cmake clang libboost-all-dev python3 python3-pip libprotobuf-dev protobuf-compiler
```

In addition, I use the zsh shell and VSCode IDE, their related configs are listed in the README files of the respective packages (packages means the subdirectories of 00.GetStarted, 01.Core...... directories in this repo, they are the main functioning blocks of a ROS software) 

### ROS2 concepts:
* (developing ......)  


### Install 
Please follow the instructions in [Install](./Docs/Install.md)

Clone this repository and use it as the **Workspace** for the packages it contains.
```zsh
git clone https://github.com/zht043/ROS2-BetterExamples-cpp.git
```

You may follow the instructions in [create your first package](./Docs/CreatYourFIrstROS2Package.md) to create a hello-world package from scratch and help you get familiar with the build & run workflow and project structure of ROS2. 

## Packages of ROS2 Examples



(developing, checkout the dev branch for the latest progress)


