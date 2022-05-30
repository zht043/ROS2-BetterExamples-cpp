# Install ROS2

## ROS2 Foxy LTS
[official ROS2 Foxy install guide](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)

summary:

```zsh
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings


sudo apt update && sudo apt install curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt upgrade
sudo apt install ros-foxy-desktop
sudo apt install ros-foxy-ros-base
```
(might take 5+ min)

## colcon
colcon is a build tool for ROS2 framework, [colcon tutorial](https://docs.ros.org/en/foxy/Tutorials/Colcon-Tutorial.html )
```zsh
sudo apt install python3-colcon-common-extensions
```

## rosdep
[rosdep wiki install page](https://wiki.ros.org/rosdep#Installing_rosdep)
```zsh
sudo apt-get install python3-rosdep
sudo rosdep init
rosdep update
```


## Recommended Configs
install argcomplete
```zsh
sudo apt-get install python3-argcomplete
```

### Adding convenient shell function for sourcing ROS2 underlay & tab-autocomplete dependencies
* if you use bash shell (default for Ubuntu), open **~/.bashrc** with an editor (vim, gedit, vscode, etc.), append the following scripts to this file
    ```bash
    source_ros2_foxy() {
        source /opt/ros/foxy/setup.bash  
        source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
        source /usr/share/colcon_cd/function/colcon_cd.sh
    }
    ```
* if you use zsh shell (I recommend zsh), open **~/.zshrc** with an editor, append the following scripts to this file
    ```zsh
    source_ros2_foxy() {
        source /opt/ros/foxy/setup.zsh  
        source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.zsh
        source /usr/share/colcon_cd/function/colcon_cd.sh

        # argcomplete for ros2 & colcon
        eval "$(register-python-argcomplete3 ros2)"
        eval "$(register-python-argcomplete3 colcon)"
    }
    ```
* Every time you want to source a ROS shell script file, choose the file ending with .zsh if you use zsh shell, or ending with .bash if you use bash shell. ROS built script files have both versions.

### VSCode Plugins
I prefer using Microsoft Visual Studio Code Editor to write cpp code, here are some plugins I recommend for ROS2 dev:

* ROS plugins:
    * **ROS** by microsoft
    * **ROS2** by nonanono
    * **URDF** by smilerobotics
* C++ plugins:
    * C/C++   
    * C/C++ Extension Pack
    * Better C++ Syntax
    * C++ Helper
    * Header source switch
    * Include Autocomplete
    * C/C++ Include Guard
    * Makefile Tools
        * note: CMake tools are included within C/C++ Extension Pack
