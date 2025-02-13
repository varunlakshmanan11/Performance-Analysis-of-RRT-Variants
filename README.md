# Performance Analysis of RRT Path Planning Algorithm

## Project Description

This Project involves comparison of performance of different variants of RRT Path Planning algorithms in 2D and implementing them to a TurtleBot3 in gazebo simulation environment by using a pid controller to check their performance on 3D simulation.

## Instructions to run the package:

## Softwares required:

Ubuntu 22.04\
ROS2 Humble\
Gazebo

Steps to be done before opening the terminal
- Extract the files from 'final_project_group_24' zip folder. 

- Create a workspace with a src folder.

- Copy the folders '24_package' and 'turtlebot3_project3' inside the '24_package' folder in the extracted folder and place them in your 'src' folder of your workspace.

Run the following commands in the terminal:

```
cd <your_workspace>

colcon build
source install/setup.bash
```

To run the gazebo environment use this following command in the terminal:

```
ros2 launch turtlebot3_project3 competition_world.launch.py
```
Run the following commands for trying different algorithms on a seperate terminal:

For RRT-star:
```
ros2 run 24_package closed_loop_RRT_star
```
For informed-RRT-star:
```
ros2 run 24_package closed_loop_informed_RRT_star
```
For Q-RRT-star:
```
ros2 run 24_package closed_loop_Q_RRT_star
```
For informed-Q-RRT-star:
```
ros2 run 24_package closed_loop_informed_Q_RRT_star
```
For RRT:
```
ros2 run 24_package closed_loop_RRT
```
Note:  Reset or launch the gazebo world each time you run different algorithm.
       The 2d vizualization folder contains python codes for 2d vizualization, which are not meant for gazebo simulation.

The codes in 2d vizualization ask for the following inputs:
- Start node
- goal node
- search radius (for all algorithms except RRT)
- step size (for all algorithms except RRT)
- depth (for Quick-RRT-star, Informed-RRT-star)
- Iterations (for all algorithms except RRT)

Video demos of the observations made

2D- visulization
https://drive.google.com/file/d/1oeMSC2dic9kGpPTEV-Chv5lB1lhhDZjj/view?usp=sharing

Gazebo Visualization
https://drive.google.com/file/d/1Y2LveUvn7KraKCprEd8V5AkdQiZWvRAb/view

## Contibutors

Member 1:\
Varun Lakshmanan\
120169595\
varunl11@umd.edu\
https://github.com/varunlakshmanan11

Member 2:\
Nitish Ravisankar Raveendran\
120385506\
rrnitish@umd.edu\
https://github.com/Nitish05

Member 3:\
Sai Jagadeesh Muralikrishnan\
120172243\
jagkrish@umd.edu\
https://github.com/cravotics
