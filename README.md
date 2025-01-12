# Gazebo_Tutorial
Gazebo Ignition-Fortress tutorial

Followed by [Gazebo official Tutorial](https://gazebosim.org/docs/fortress/tutorials/)

- [ROS2 Humble Installation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- [Gazebo Binary Ubuntu Install](https://gazebosim.org/docs/fortress/install_ubuntu/)
- [Gazebo Source Installation](https://gazebosim.org/docs/fortress/install_ubuntu_src/)

## 1. Launch simulation
``` shell
ign gazebo building_robot.sdf
```
> [!note]
> Note: You can name your file any name and save it anywhere on your computer.

## 2. Topics and Messages
After you launched the robot world, send a message to to out robot.
``` shell
ign topic -t "/cmd_vel" -m ignition.msgs.Twist -p "linear: {x: 0.5}, angular: {z: 0.05}"
```

Now you should have your robot moving in the simulation.

> [!note]
> Note: Don’t forget to press the play button in the simulation.

- **The command specifies the topic to publish** to after the `-t` option.
- After the `-m` **we specify the message type**. Our robot expects messages of type Twist which consists of two components, linear and angular.
- After the `-p` option **we specify the content (value) of the message**: `linear speed x: 0.5` and `angular speed z: 0.05`.

> [!tip]
> You can know what every topic option does using this command: ign topic -h


``` shell
sudo apt install ros-foxy-gazebo-ros-pkgs
```