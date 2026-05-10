# F1-Tenth_Duke
## SLAM
SLAM is a technique for Robots to simultaneously do Localization and Mapping. 

SLAM stands for “Simultaneous Localization and Mapping”. It is a computational problem in robotics and computer vision that involves creating a map of an unknown environment while at the same time locating the robot or camera within that environment.

`Make sure you have already tuned the odometry on your car!`

## Resouces
- https://navigation.ros.org/tutorials/docs/navigation2_with_slam.html?highlight=slam
- [Here](https://www.youtube.com/watch?v=RanGbHii2m8&t=34s) is the video of F1TENTH SLAM

## Implementation

`slam_toolbox` is the one we want to use for car to generate maps.

Installing is as easy as running the following command:
   ```bash
    sudo apt install ros-$ROS_DISTRO-slam-toolbox 
   ```

Launch `Teleop` on one window (which means the car can be controlled by joystick), and start SLAM using the following command:
   ```bash
    ros2 launch slam_toolbox online_async_launch.py 
   ```

This will start the SLAM node, and start publishing the generated map to the `/map` topic.
If you want to launch with custom parameter files, run
   ```bash
    ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/f1tenth_ws/src/f1tenth_system/f1tenth_stack/config/f1tenth_online_async.yaml
   ```
On another terminal, open rviz by running
   ```bash
    rviz2
   ```

Then,

- Add `/map` by topic
- Add `/graph_visualization` by topic (optional, I personally do like this)
- On top left corner of rviz, panels → add new panel → add SlamToolBoxPlugin panel
- Once you’re done mapping, save the map using the plugin
- You can give it a name in the text box next to Save Map. Map will be saved in whichever directory you ran slam_toolbox

## Reference:

F1TENTH. https://f1tenth.org/learn.html
slam_toolbox. https://github.com/SteveMacenski/slam_toolbox