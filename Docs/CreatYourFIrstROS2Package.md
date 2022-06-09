# Create Your First ROS2 Package

The ROS2 framework is designed to break down a robotics software system into concurrent multiprocess-programs called nodes, which interacts with one another by the 3 major core functionalities: Topic, Service, and Action. 

Therefore, a hello world code for ROS2 should do a little more than just a C++ hello world program in a package. This section will introduce how to create a ROS2 package containing two nodes doing the same thing: printing hello world to stdouts on two seperate terminals, and also how to compile and run these codes.

A ROS2 **package** may contain multiple **Nodes**, each **Node** runs on a separate process in the OS. Each **Node** can have multiple threads like any C++ program as well.  

## Typical Package File Structure
```zsh
- <package_name>
		- CMakeLists.txt  # CMake is the most popular C++ compilation/package managing tool,there are some specific CMake commands for ROS2
		- package.xml     # Similar to the maven tool for Java, manages the meta info for this package
		- include  # contains C++ header files
		- src      # contains C++ source files 
		- msg      # contains ROS2 message type definition files for pub-sub communication
		- srv      # contains ROS2 service def files
		- action   # contains ROS2 action def files 
		- launch   # contains launch scripts, can be used to launch multiple Nodes in multiple windows via a single command

		# Extra advanced stuff
		- test
		- scripts
		- docs
		- rviz
		- maps
		- config
		- models  # SDF models
		- urdf
		- worlds  # Gazebo
		
		- <other user-defined stuff>
```
* The colcon tool will compile the codes in a package based on the info in **package.xml** and **CMakeList.txt**
* **Packages** are usually kept under a **Workspace** directory, the compilation process by colcon usually generates the resulting products including executables under the workspace directory instead of inside the packages themselves  

## Creat Package

* First create a Workspace directory if you haven't done so already, in this example **the workspace will be the "00.GetStarted" directory** (in the official tutorial it's the **"dev_ws"** directory), then create a "src" directory under the workspace root.

* The source_ros2_foxy is a custom command defined in my .zshrc (or .bashrc for bash users), it sources ros2 underlay and other useful stuff, check out the [Install](./Install.md) guide.

```zsh
source_ros2_foxy

# Assuming you are already in the workspace directory
#   and the workspace contains a directory named "src"

# using the pkg tool to generate a starter package named hello_world
mkdir -p src
cd src
ros2 pkg create --build-type ament_cmake hello_world

# create other common directories in a ROS2 package 
cd hello_world
mkdir -p msg srv action launch

# then open the package directory in VSCode (or another editor/IDE) 
code . 
```

## Configure package.xml
* Open package.xml in your editor/IDE
* Modify info elements like version, description, email, license, etc., as you see fit
* For this example, simply add the following line below "<buildtool_depend>ament_cmake</buildtool_depend>":
    ```xml
    <depend>rclcpp</depend>
    ```
* In more complex applications, there will be more additional elements 

## Configure CMakeList.txt
Modify CMakeList.txt to be:
```CMake
cmake_minimum_required(VERSION 3.5)
project(hello_world)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(node1 src/node1.cpp)
ament_target_dependencies(node1 rclcpp)

add_executable(node2 src/node2.cpp)
ament_target_dependencies(node2 rclcpp)

install(
  TARGETS node1 node2
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

```

## VSCode ROS Tips
(tips for noobs, experts plz ignore)
* Assuming you have already installed the ROS plugins in VSCode listed in the [Install](./Install.md) guide
* In your .vscode/c_cpp_properties.json (in the package directory, not workspace or repo root!), add this path to the includePath field:
	* "\<complete path to your workspace\>/install/\<package name\>/include/"
		* the "install" directory is auto-generated after compilation using colcon, it doesn't exist when you have a fresh start with a new workspace yet
		* this path leads to where ROS2 stores the auto-generated C++ sources from the .msg, .srv, .action definition files  
		* telling VSCode this particular include path helps the not-so-smart(:P) VSCode C++ intellisense functionalities.
		* As an example, in my computer this path would be:
        	* "/home/zht/Project/ROS2-BetterExamples-cpp/00.GetStarted/install/hello_world/include/"
			 
* Having VSCode opening your particular package directory allows the CMake and C++ intellisense plugins in VSCode to work properly with features such as autocomplete, etc. 
	* in this particular hello world example, your VSCode should be opening the "hello_world" directory
* Useful command if unreasonable error squiggles appears: ctrl+shift+P, then type: CMake:Configure 
* Technically this hello world example doesn't have any .msg, .srv, or .action definition, but you will eventually need them anyways

## Write C++
Since the two nodes do the exact same thing, I will only post the code for node1.cpp, just copy paste it and rename it to node2.cpp for the second node.
```C++
#include <memory>
#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using rclcpp::TimerBase;

class Node1 : public rclcpp::Node {
public:
    Node1() : Node("Node1") {
        /* create a timer that calls the timer_callback func for every 0.5 seconds */
        timer = create_wall_timer(
            0.5s, 
            std::bind(&Node1::timer_callback, this)
        );
    }
private:
    void timer_callback();
    std::shared_ptr<TimerBase> timer;
};


void Node1::timer_callback() {
    RCLCPP_INFO(get_logger(), "Hello World!");
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(
        std::make_shared<Node1>()
    );
    
    rclcpp::shutdown();
    return 0;
}
``` 
For node2.cpp, change Node("Node1") to Node("Node2")


## Compile
* colcon tool will be used for compiling these ROS C++ codes 
* The normal C++ project uses cmake & make commands to compile, but in ROS2 framework, it involves compiling multiple CMake projects (corresponding to the multiple ROS2 Nodes)
* colcon will compile/build everything in a workspace or package with a single command
```zsh
cd ../.. #this will navigate back to the workspace directory
source_ros2_foxy
colcon build
```
* colcon build will compile all packages in the workspace
* To compile just the hello_world package, use this instead:
	```zsh
	colcon build --packages-up-to hello_world
	```
* colcon command is meant to be called under the workspace directory, instead of the package directory!

## Run
* Spawn a new terminal, don't use the terminal you called colcon for running your ROS2 code, or bugs might arise according to ROS2 documentations

In your new terminal:
```zsh
cd <path to your workspace> # workspace directory, not package directory
source install/setup.zsh
ros2 run hello_world node1
```

then in another new terminal:
```zsh
cd <path to your workspace> # workspace directory, not package directory
source install/setup.zsh
ros2 run hello_world node2
```