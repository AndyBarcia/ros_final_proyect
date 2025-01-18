# Intelligent Robotics 1 - Final Project (ros_final_project)

This repository contains a ROS (Robot Operating System) package for implementing a basic navigation system for a TurtleBot robot using a behavior tree. The robot uses lidar data for perception and employs a subsumption architecture to handle navigation and obstacle avoidance.

## Project Overview

The core of this project is to demonstrate a basic autonomous navigation system using a behavior tree that approximates a subsumption architecture. The robot's primary behavior is to move forward while avoiding obstacles. When encountering an obstacle, the robot attempts a backoff maneuver followed by a random but biased safe rotation to find a clear path.

## Components

The project consists of the following main components:

*   **`main.py`:** The main execution script. It initializes the ROS environment, sets up the behavior tree, and controls the robot.
*   **`utils/lidar.py`:** A class that handles processing of lidar data, including calculating safe distances and detecting possible door locations.
*   **`behaviours/`:** A directory containing the different behavior tree nodes that control the robot's actions:
    *   `obstacle_detection.py`: Detects if an obstacle is too close, considering the speed of the robot.
    *   `fixed_movement.py`: Moves the robot at a fixed linear and angular speed.
    *   `random_rotation.py`: Rotates the robot to a random but biased safe direction.
    *   `decide_movement.py`: *(Unused)* A behavior to decide the robot movement based on safe distances.
    *   `find_doors.py`: *(Unused)* A behavior that checks for the existence of doors.
    *   `move_to_door.py`: *(Unused)* A behavior that makes the robot move towards a detected door.
    *   `stuck_detection.py`: *(Unused)* A behavior that tries to detect if the robot is stuck.

## Subsumption Architecture

The robot's behavior is organized in a two-level subsumption architecture using a behavior tree:

1.  **Level 1 (Default):** The robot moves forward, relying on `ObstacleDetection` to check if there is an obstacle in front. If an obstacle is detected, it activates level 2.
2.  **Level 2 (Recovery):** The robot tries to recover from an obstacle detection by backing off and rotating randomly to a safe direction, exploring the environment.

## Compiling

To compile the project, follow these steps:

1.  Clone the repository into your ROS workspace `src` folder:

    ```bash
    cd /root/catkin_ws/src
    git clone https://github.com/AndyBarcia/ros_final_proyect.git
    ```

2.  Compile the ROS package:

    ```bash
    cd /root/catkin_ws && catkin_make
    ```

    Make sure you have all the necessary dependencies installed (e.g. `rospy`, `geometry_msgs`, `sensor_msgs`, `py_trees`).

## Running

To run the project:

1.  Source the ROS environment:

    ```bash
    source /root/catkin_ws/devel/setup.bash
    ```

2.  Run the main node:
   
   ```bash
   rosrun ros_final_proyect main.py
   ```

    **Note:** This project is intended to run within a simulated environment with a TurtleBot robot. You'll need to ensure the simulation environment is running before launching the navigation node. If the simulation is not running the `/odom` and `/scan` topics will not be available, and the system will not work as expected.

## Important Considerations

*   The unused behaviors (`decide_movement.py`, `find_doors.py`, `move_to_door.py`, `stuck_detection.py`) are not integrated into the main behavior tree, but can be used to further improve the navigation system and its logic.
* The `random_rotation.py` uses a *softmax* approach to make it more biased towards open spaces when trying to rotate, and can be adjusted using a *temperature* parameter.
* The `obstacle_detection.py` now considers the robot's speed when trying to detect obstacles.
* The robot's behavior can be further customized by adjusting the parameters in the code, such as the robot's speed, sensor thresholds, and time limits.

## Further Development

This project provides a base for a more complex navigation system. Future improvements could include:

*   Integrating the unused behaviors into the main behavior tree.
*   Implementing more sophisticated obstacle avoidance techniques, such as path planning or more intelligent recovery maneuvers.
*   Adding more advanced behaviors, such as door recognition and traversal.
*   Adding error handling and debugging mechanisms.
