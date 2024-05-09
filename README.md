# Ros2_moving_car

This project simulates an object with three wheels using ROS2. 
The object can be visualized in both Gazebo and RViz. 
It includes functionalities to move the object to random positions and control it via keyboard input.

## Demo
In Rviz
<img width="585" alt="스크린샷 2024-05-09 오후 3 54 42" src="https://github.com/mininiii/Ros2_moving_car/assets/96100666/94b040f3-aea2-4d96-b763-dfda96db6bfc">

In Gazebo
https://github.com/mininiii/Ros2_moving_car/assets/96100666/bccab431-d5e3-4870-8259-5288649802d3

## Setup
ros2, gazebo, rviz2 should be already installed.

1. Create a workspace directory:
   ```
   mkdir example_ws
   ```

2. Create a `src` directory in the workspace:
   ```
   mkdir example_ws/src
   ```

3. Clone this repository into the `src` directory:
   ```
   cd example_ws/src
   git clone https://github.com/mininiii/Ros2_moving_car
   ```

## Launch
1. Build and source the workspace:
   ```
   cd example_ws
   colcon build
   source install/setup.bash
   ```

2. Launch the simulation:
   ```
   ros2 launch moving_car sim.launch.py
   ```

   This will launch RViz and Gazebo.

## Keyboard control
After launching the simulation, you can move the object using your keyboard input in another terminal:
  ```
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
  ```
