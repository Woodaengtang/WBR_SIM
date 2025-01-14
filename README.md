# WBR Gazebo Simulation
Simulation conducted in ROS2 Foxy and Gazebo Classic
- [ROS2 Foxy installation](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)
- Gazebo classic installation
``` shell
sudo apt install ros-foxy-gazebo-ros-pkgs
```
# Introduction
Recent advancements in the field of robotics have seen significant progress in legged robots. Among them, a mobile manipulation robot, is being utilized in industrial environments. However, conventional legged robots generally require a relatively high Cost of Transport (CoT) during operation, making them challenging to deploy efficiently in energy-constrained environments. As an alternative, Wheeled Legged Robots (WLRs) have been proposed, combining legged structures with wheels to enhance energy efficiency and provide stable locomotion across various terrains. Research in this area has been actively pursued.

Wheeled Legged Robots integrate the advantages of both wheel mechanisms and leg mechanisms. Wheeled robots excel in fast and efficient movement on flat surfaces but face limitations in handling complex terrains. Conversely, legged robots exhibit excellent obstacle-overcoming and irregular terrain adaptation capabilities but demand relatively complex motion planning and extended travel times. A Wheeled Legged Robot seeks to combine these strengths. When designing a WLR, selecting an appropriate degree of freedom and gait sequence can reduce the CoT.

This enables mobile robots to find potential applications in various fields, including exploration of rugged terrains, rescue missions, and logistics. For example, Boston Dynamics' Handle combines the mobility and adaptability of legs to achieve stable and flexible movements across diverse terrains. Similarly, ETH Zurich's Ascento has been the subject of various studies, including implementing fall recovery mechanisms, jump mechanisms, and phase optimization to reduce weight.