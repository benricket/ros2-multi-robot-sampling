## Multi-robot sampling with ROS2 in C++

This project implements a simple approach to sampling a scalar field via multiple robots, using a Gaussian process to maintain a model of the environment being sampled over time.

### Setup

To run this code, ensure ROS2 Humble is installed. From the workspace directory in which you intend to use this project, clone the repository in `src/`, and build the project using `colcon build --packages-select adaptive_sample` from the workspace root. 

### Single-robot sampling

Simulation of a single robot is largely contained in the `single_robot_send_waypoint_launch.py` launch file. To run this:

- Build the project with `colcon build --packages-select adaptive_sample`
- In one terminal, source the workspace install directory with `source install/setup.bash`. Then, run the launch file with `ros2 launch adaptive_sample single_robot_send_waypoint_launch.py`.
- In a second terminal, source the workspace install directory again, and then run the controller script with `ros2 run adaptive_sample sim_controller.py`. Pressing ENTER in this terminal will start or resume the simulation, while pressing SPACE will pause the simulation. 
- If you want to adjust any of the simulation parameters on the fly, the three weights of the reward function (mean, variance, and distance coefficients) can be adjusted via `ros2 param set` on the command line. The speed of the simulated robot and the standard deviation of the random noise applied to the waypoint picking script can also be adjusted as ROS2 params.