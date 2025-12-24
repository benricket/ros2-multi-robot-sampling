## Multi-robot sampling with ROS2 in C++

This project implements a simple approach to sampling a scalar field via multiple robots, using a Gaussian process to maintain a model of the environment being sampled over time.

### Overview

The system contained in this project is coordinated rather than distributed, where a central processing node stores the model of the environment being sampled and is responsible for updating the model in response to samples and providing waypoints for the robots to navigate to. This functionality is contained in `src/orchestrator.cpp` as a ROS node. The primary functions of this node are as follows:
- Randomly generate the scalar field to sample at the start of the simulation. This takes the form of a class which inherits from the abstract `Environment` class from `src/ground_truths.cpp`, and is currently set to randomly generate an environment by summing multiple Gaussian distributions in space. If repeatability is desirable, the seed for the random number generator should be fixed.
- Contain a Gaussian Process (GP) object (from [this library](https://github.com/andreacasalino/GaussianProcesses)) to use as a model of the environment. Data is added to this model whenever communicated by the robots, and the model is sampled when generating waypoints.
- `upload_data_callback()`, which handles a robot reaching a waypoint and uploading data. As the orchestrator actually holds the environment model, it is still responsible for querying the environment for the sampled value using the position indicated by the robot. We add this value to the GP, draw values from the GP with a given resolution to cover the map, and generate a new waypoint to pass to the robot given our reward function.
- `weight_model_distance()`, which returns a waypoint to sample given the position of a robot and the reward function coefficients. The reward function is a linear combination of the following --- mean of the environment according to the model, variance of the environment according to the model, distance from the robot (penalty), and square root of distance from the robot (reward). The mean and variance incentivize the sampling of higher-valued regions while also encouraging exploration, the distance penalty ensures the robot prioritizes waypoints generally nearby, and the distance reward discourages sampling too close to the current location.
- `retrain_hyperparams()`, which is called every time a set number of data points are added to the model to retrain the model hyperparameters using gradient descent. 

The other functionality is contained in `simple_robot.cpp`, which contains a node representing a single robot in the simulation. The robot has a position, and maintains a queue of waypoints to navigate to. When the robot reaches a waypoint, it broadcasts this to the orchestrator node before continuing to the next waypoint. The function of the robot is not sophisticated --- it moves at a fixed speed, directly in the direction of the waypoint, and has no checks regarding collision with other robots. 

This class inherits from `Sampler` in `sampler.cpp`, which has an `upload_data()` method for uploading sampled data from a position. In the original vision for this project, different implementations of robots could all inherit from a `Sampler` class containing the sampling/coordination code; in practice, this didn't happen as much, and the `simple_robot.cpp` file is the only one to implement a robot's behavior.

This project also contains two helper scripts in Python, one for visualizing the simulation and one for controlling it. 

The controller is stored in `sim_controller.py` and simply allows for the program to be paused or resumed by pressing SPACE or ENTER/RETURN, respectively. The simulation only starts running when ENTER/RETURN is pressed. 

The visualization node is stored in `visualize.py`, which uses Matplotlib to plot the robots in their environment and visualize the reward functions they use to navigate. This node subscribes to the robot position topics, environment topic, and model state / reward function topics, allowing it to plot the environment, model mean, model variance, as well as weighted reward function from the perspective of specfic robots. Each robot is represented as a colored dot on the graph. 

The other scripts, `gp_test_distance.cpp`, `gp_test_func.cpp`, and `gp_test_pub.cpp`, were used for testing isolated parts of the simulation before the full simulation ran. These scripts let position and data points be manually added to the model without the use of a robot node running, but are no longer necessary for the project to function. 

### Setup

To run this code, ensure ROS2 Humble is installed. From the workspace directory in which you intend to use this project, clone the repository in `src/`, and build the project using `colcon build --packages-select adaptive_sample` from the workspace root. 

To ensure the right version of numpy and Matplotlib are used in the Python code, I recommend creating a virtual environment in some way (e.g. `conda create --name sampling python==3.10`) Then, from the repository root, install the right dependencies via 
```bash
pip install -r requirements.txt
```

### Single-robot sampling

Simulation of a single robot is largely contained in the `single_robot_send_waypoint_launch.py` launch file. To run this:

- Build the project with `colcon build --packages-select adaptive_sample`
- In one terminal, source the workspace install directory with `source install/setup.bash`. Then, run the launch file with `ros2 launch adaptive_sample single_robot_send_waypoint_launch.py`.
- In a second terminal, source the workspace install directory again, and then run the controller script with `ros2 run adaptive_sample sim_controller.py`. Pressing ENTER in this terminal will start or resume the simulation, while pressing SPACE will pause the simulation. 
- If you want to adjust any of the simulation parameters on the fly, the three weights of the reward function (mean, variance, and distance coefficients) can be adjusted via `ros2 param set` on the command line. The speed of the simulated robot and the standard deviation of the random noise applied to the waypoint picking script can also be adjusted as ROS2 params.

### Multi-robot sampling

Simulation of multiple robots can be achieved using the `multi_robot_launch.py` launch file. This initializes the simulation just as the single robot launch file does, except it also takes an integer to describe the number of robots. e.g., 
```bash
ros2 launch adaptive_sample multi_robot_launch.py num_robots:=5
```

If no value is provided, the default value of 2 robots is used. This still requires the sim controller be used via `ros2 run adaptive_sample sim_controller.py` to initiate the simulation.

### Challenges & Next Steps

Working on this project, I ran into a few challenges and difficulties that made progress more difficult in some regions. Sometimes, these were due to my relative unfamiliarity with ROS2 in C++ as opposed to ROS2 in Python, though this wasn't the case for most of these challenges.

#### Poor model performance

Throughout the project, the model was consistently poor and would not approximate the ground truth well, even for simple ground truths like a single gaussian. I always noted that upon retraining the hyperparameters, the length scale of the model would massively increase, resulting in a model that was much wider and flatter than how it should have been, even when the high points of the model aligned with the higher points of the environment. I tried taking more samples before retraining the hyperparameters, but this was not helpful. 

Some of this has been disappointing --- with such a poor model, it's difficult to measure the performance of the actual sampling algorithm, because the reward function and waypoint sampling is using a poor approximation of the environment.

However, understanding and refining the Gaussian Process model has never been my main goal for this project --- I primarily wanted to write out the overall structure / system architecture and get a feel for what will work, and I'll have more time in the future to learn the theory and math behind Gaussian Processes, along with gaining insights into how I could use one better in this case, in the future.

#### Compute time

As the project is currently implemented, the performance of it is very poor with regard to time. All of the heavy processing, using the model, is performed in the orchestrator node in a single process. This makes simulations with large numbers of robots slow to run, as every single added sample is processed on its own, and the entire field is sampled from the model for every waypoint generated. 

I've identified a few good approaches for speeding up this time, which I outline in the next steps section. 

#### Parameter tuning

The behavior of the simulation is very dependent on the coefficients used for the reward function, of which there are three (absolute scale of the coefficients does not matter, so we can divide by one of them to get three free variables.) I didn't have the time to implement a script calculating metrics to measure the performance of the code by, so I wasn't able to measure the success of different reward function parameters in any way other than observing the simulation and remarking on whether it looks to be working better or worse.

#### Next steps

While there are many avenues to explore for improving this project, I've isolated a few that are smaller in scope and would be beneficial to achieve. 

For one, I would define metrics outlining the performance of the simulation and record them automatically. Metrics could be average reward score per sample (normalized), average sampled value, or error in model prediction versus ground truth. Calculating these metrics would not be difficult, and would let me quantify certain choices of reward functions over others.

I'd also implement a few changes for the program to run faster. These are some ideas:
- The model doesn't need to update with every sample. Instead, update after every N samples or M seconds have elapsed. 
- Similarly, don't draw from the model every sample. Instead, keep a mean/variance map stored around for N samples of M seconds, and simply update it given the distance values for each robot. The data would be slightly more stale, but the speed would be faster.
- Give multiple waypoints per request. The robots support a queue of multiple waypoints, which we don't use so far. 
- Communicate the ground truth data to the robots so they actually figure out what value is sampled, saving some compute on the orchestrator. 
- *If* the distance weight adjustment is taking a noticeable amount of compute time, I could approximate it for faraway points. I.e., if the robot is at 0,0, we may not care about the distance at 9.8,9.8 versus 9.9,9.7 as much as we care about 0.2,0.2 versus 0.1,0.3. Coarses distance approximation further from the robot could potentially save on some time. 

Additionally, I think it would be cleaner to refactor the code such that `Sampler` is a class which `SimpleRobot` contains as an attribute, rather than being the parent class, which feels more complicated for little benefit. 
