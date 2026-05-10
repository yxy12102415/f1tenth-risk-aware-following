# F1-Tenth_Duke
## Particle Filter-Localization

We need to the particle filter to do Localization. It is a recursive Bayes Filter. Similar to Kalman Filter but relaxes some of the assumptions.

This is a video of implementing particle filter for real F1TENTH car in Hudson Hall.
![particle filter](/Images/particle_filter.mp4)

## Resources

- [Here](https://www.youtube.com/watch?v=YBeVDxTHiYM) is a 5 minutes summary video for you to easily know what is particle filter and why we use it for localization.
- [F1TENTH Particle Filter Lecture video](https://www.youtube.com/watch?v=SRBdpoPl57Q), and [slides](https://docs.google.com/presentation/d/1SrWjDIDoI1kIZAeT6ZplrCQYnG2HPIw_ATynDCdLiWo/edit#slide=id.gd1ccde3aa4_0_594).

## Algorithm

1. Sample the particles using the proposal distribution
2. Compute the importance weights
3. Resampling: Draw sample i with probability w and repeat J times

## Running Particle Filter for F1TENTH

On the F1TENTH with an NVIDIA Jetson Xavier NX, the pose is published ar ~30-40Hz.

Clone the repository by running
   ```bash
   git clone https://github.com/f1tenth/particle_filter.git
   ```
   ```bash
   sudo apt install ros-$ROS_DISTRO-tf-transformations
   ```

To install the `range_libc` library
   ```bash
   sudo pip install cython
   git clone http://github.com/kctess5/range_libc
   cd range_libc/pywrappers
   # on VM
   ./compile.sh
   # on car - compiles GPU ray casting methods
   ./compile_with_cuda.sh
   ```

Put the map file(image+.yaml) in `particle_filter/mpas`. Change the `map` parameter inside `particle_filter/config/localize.yaml` to reflect the map you want to use.

To start the particle filter, run
   ```bash
   ros2 launch particle_filter localize_launch.py
   ```

You can then set an initial pose in rviz by clicking the "2D pose Estimate" button.

## Reference:

F1TENTH. https://f1tenth.org/learn.html