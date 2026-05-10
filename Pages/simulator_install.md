# Simulator setup
## Setting up F1TENTH Gym Environment with ROS2 Communication Bridge

Description: This guide provides step-by-step instructions on how to set up the F1TENTH Gym environment with a ROS2 communication bridge on different systems including Ubuntu. By following this guide, you'll have a containerized ROS communication bridge for the F1TENTH gym environment turning it into a simulation in ROS2.

## Table of Contents
1. [Prerequisites](#prerequisites)
2. [Installation on Ubuntu 20.04](#native-ubuntu)

<a name="prerequisites"></a>
## 1. Prerequisites
- Ubuntu 20.04 (for native installation)

<a name="native-ubuntu"></a>
## 2. Installation on Ubuntu 20.04 
### Dependencies Installation:
1. Install ROS 2 Foxy by following the instructions [here](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html).
2. Set up the F1TENTH Gym:
   ```bash
   git clone https://github.com/f1tenth/f1tenth_gym
   cd f1tenth_gym && pip3 install -e .
   ```

### Simulation Installation:
1. Create a workspace:
   ```bash
   cd $HOME && mkdir -p sim_ws/src
   ```
2. Clone the repository into the workspace:
   ```bash
   cd $HOME/sim_ws/src
   git clone https://github.com/f1tenth/f1tenth_gym_ros
   ```
3. Update the map file path in `sim.yaml`:
   - Navigate to [sim.yaml](https://github.com/f1tenth/f1tenth_gym_ros/blob/main/config/sim.yaml) in your cloned repository.
   - Change the `map_path` parameter to `<your_home_dir>/sim_ws/src/f1tenth_gym_ros/maps/levine`.
4. Install dependencies with rosdep:
   ```bash
   source /opt/ros/foxy/setup.bash
   cd $HOME/sim_ws/
   rosdep install -i --from-path src --rosdistro foxy -y
   ```
5. Build the workspace:
   ```bash
   colcon build
   ```
Ensure to test the setup to confirm the simulation runs successfully in the ROS2 environment.

# Launching the Simulation

The provided container includes `tmux`, which allows for creating multiple bash sessions in the same terminal. Follow the steps below to launch the simulation.

1. Source Scripts:
    Ensure you source both the ROS2 setup script and the local workspace setup script in a bash session from the container.
    ```bash
    $ source /opt/ros/foxy/setup.bash
    $ source install/local_setup.bash
    $ source $HOME/ros2_foxy/install/local_setup.bash
    ```

2. Launch Simulation:
    Now launch the simulation using the following command:
    ```bash
    $ ros2 launch f1tenth_gym_ros gym_bridge_launch.py
    ```

    A `rviz` window should pop up showing the simulation. The display will appear either on your host system or in the browser window, depending on the display forwarding option you chose.

3. Running Additional Nodes:
    To run another node, create a new bash session in `tmux`. For example, to create a new session, you can use the following command:
    ```bash
    $ tmux new -s session_name
    ```

    Replace `session_name` with a name of your choice for the session. Now, in this new session, you can run additional ROS2 nodes as needed.
