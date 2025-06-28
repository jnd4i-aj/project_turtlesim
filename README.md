# project_turtlesim

A ROS1-based simulation project for controlling and programming turtles in a virtual environment using Python. The project is structured as a ROS (Robot Operating System) catkin package and demonstrates various robot control concepts including goal navigation, pursuit behaviors, noise modeling, and more.

The idea behind the project was to build and implement a dynamic pursuit scenario in ROS Turtlesim, inspired by the classic childhood police-thief game, where the Police Turtle actively pursues the Thief Turtle while adhering to defined dynamic constraints.

---

## Features

- **Turtle Control**: Move the turtle to user-defined goals with PID controllers.
- **Chase Behaviors**: Various scripts to chase another robot (the "Robber Turtle") using real-time pose tracking and control.
- **Gaussian Noise Modeling**: Add noise to the pose for testing robustness of algorithms.
- **Spawning Multiple Turtles**: Scripted spawning of turtles at random locations and after delays.
- **Motion Planning**: Predictive planning to intercept a moving target based on its path.
- **ROS Integration**: Uses standard ROS packages (`rospy`, `roscpp`, `std_msgs`) and topics for inter-node communication.

---

## Installation

### Prerequisites

- ROS1 (Noetic recommended)
- Catkin workspace set up (`catkin_ws`)
- Python 3 with numpy

### Steps

1. Clone the repository into your catkin workspace's `src` folder:

    ```bash
    cd ~/catkin_ws/src
    git clone https://github.com/jnd4i-aj/project_turtlesim.git
    ```

2. Install dependencies:

    ```bash
    cd ~/catkin_ws
    rosdep install --from-paths src --ignore-src -r -y
    ```

3. Build the workspace:

    ```bash
    catkin build
    source devel/setup.bash
    ```

---

## Usage

### Run the Turtlesim Node

1. Start ROS core:

    ```bash
    roscore
    ```

2. In a new terminal, launch the turtlesim node:

    ```bash
    rosrun turtlesim turtlesim_node
    ```

### Main Functionalities

- **Manual Turtle Goal Control**  
  Run the script to move the turtle to a user-provided goal position:

  ```bash
  rosrun project_turtlesim goal_1_control_turtle.py
  ```

  The script will prompt for X and Y coordinates for the goal.

- **Spawn a Second Turtle (PT) After Delay**  
  Run:

  ```bash
  rosrun project_turtlesim spawn_second_turtle.py
  ```

  This script waits 10 seconds after launch and spawns a new turtle at a random location.

- **Chase the Robber Turtle**  
  Run one of the chasing scripts, e.g.:

  ```bash
  rosrun project_turtlesim goal_4_chase_turtle_fast.py
  ```

  or with prediction:

  ```bash
  rosrun project_turtlesim goal_5_chasing_with_plan.py
  ```

- **Add Gaussian Noise to Turtle Pose**  
  The utility functions in `src/common_utils.py` can be used to publish noisy pose data for robustness testing.

---

## Example: Chasing the Robber Turtle

1. Launch turtlesim and spawn the "Robber Turtle".
2. Spawn the "Police Turtle" (PT) using `spawn_second_turtle.py`.
3. Start the chase script (`goal_4_chase_turtle_fast.py` or `goal_5_chasing_with_plan.py`).
4. The PT will subscribe to the RT's pose and try to catch it, printing updates to the console.

---

## Project Structure

- `src/goal_1_control_turtle.py`: Move turtle to a goal using user input.
- `src/spawn_second_turtle.py`: Spawns a new turtle after a delay.
- `src/goal_4_chase_turtle_fast.py`: Fast chase implementation.
- `src/goal_5_chasing_with_plan.py`: Predictive chase with path planning.
- `src/common_utils.py`: Helper functions (e.g., add Gaussian noise).
- `CMakeLists.txt` & `package.xml`: Standard ROS package files.

---


## License

This project currently does not specify a license. If you plan to use it beyond personal or academic purposes, please clarify with the repository owner.

---

## Author

GitHub: [jnd4i-aj](https://github.com/jnd4i-aj)
