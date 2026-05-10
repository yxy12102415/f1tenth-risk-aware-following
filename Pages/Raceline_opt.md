# Race Line Optimization

## Thanks for Steve Gong. We followed some of his instruction (https://stevengong.co/notes/F1TENTH-Field-Usage). However, please use the code we left in "Code" section to reduce your debugging time. We didn't successfully run though the whole process. Docker is STRONGLY RECOMMENDED!!! The main environment of this part is Conda, and if you don't want to have 4 versions of numpy in your laptop, Docker!

## Generate Waypoints / Raceline (Offline with LINUX Computer)

These need to be generated on a Linux computer. You can just use VSCode and remote SSH from your Macbook.

## Option 1: Generate using Raceline Optimization

This is the preferred pipeline. SSH into the remote computer from VSCode. Then,

```sh
cd ~/Desktop/Raceline-Optimization
conda activate raceline
```

Run map_converter.ipynb, changing MAP_NAME to the name of the map. Make sure to look at the waypoints and see if everything is generated properly.

Run sanity_check.ipynb to make sure everything works.

Change inside main_globaltrac_f110.py the MAP_NAME to the name of the map.

```sh
python3 main_globaltraj_f110.py
```
The waypoints should be generated in 'outputs/<MAP_NAME>'.

### Common Errors
You might get a “At least two spline normals are crossed”. In this case, just reduce the number of points generated in map_converter.ipynb (e.g., take half of the points).

Sometimes, the solver can seem to get stuck for a long time. In this case, go into f110.ini, and play around with increasing the numbers below:

```sh
stepsize_opts={"stepsize_prep": 0.5,
               "stepsize_reg": 1.5,
               "stepsize_interp_after_opt": 1}
```
To get a good safety margin, add the following:
```sh
optim_opts_mintime={"width_opt": 0.6}
```
However, if you make this value too large, the solver will run into an error.

You can run visualize_raceline.ipynb to sanity check the raceline generated.

Once you are happy with the raceline generated, right-click the file and download it locally, onto the F1TENTH pure_pursuit/racelines.

You should also push on the RRT node in case you want to do dynamic obstacle avoidance:
```sh
cp ~/Desktop/Raceline-Optimization/outputs/<MAP_NAME>/<OUTPUT_NAME>.csv ~/Desktop/f1tenth_ws/src/rrt/racelines
```
Push those changes. Now, from your Macbook, pull these changes, so then in the next step you scp the nodes onto the F1TENTH.

## Option 2: Generate using the Waypoint Logger in Simulation
Use the waypoint generator node to manually create waypoints:
```sh
ros2 run waypoint_generator waypoint_subscriber
```
