Driver Stack Setup
==================

Equipment Required:
-------------------
- Fully built F1TENTH vehicle
- Pit/Host computer OR
- External monitor/display, HDMI cable, keyboard, mouse

.. warning:: 
   This instruction is only for Jetson Xaviers NX, which has JetPack versions after 5.0 and ROS2 only. 

Overview
--------
Since the release of `JetPack 5.0 Developer Preview <https://developer.nvidia.com/jetpack-sdk-50dp>`, the Jetson can now run on Ubuntu 20.04, and we can install ROS 2 natively and conveniently.
We use ROS 2 Foxy for communication and run the car. You can find a tutorial on ROS 2 [here](https://docs.ros.org/en/foxy/Tutorials.html).

In the following section, we'll go over how to set up the **drivers** for sensors and the motor control.

Everything in this section is done on the **Jetson NX** so you will need to connect to it via SSH from the Pit laptop or plug in the monitor, keyboard, and mouse.

.. _udev_rules:

1. udev Rules Setup
----------------------

When you connect the VESC to the Jetson, the operating system will assign them device names of the form ``/dev/ttyACMx``, where ``x`` is a number that depends on the order in which they were plugged in. For example, if you plug in the mouse before you plug in the VESC, the mouse will be assigned the name ``/dev/ttyACM0``, and the VESC will be assigned ``/dev/ttyACM1``. This is a problem, as the car’s configuration needs to know which device names the devices are assigned, and these can vary every time we reboot the Jetson, depending on the order in which the devices are initialized.

Fortunately, Linux has a utility named udev that allows us to assign each device a “virtual” name based on its vendor and product IDs. For example, if we plug a USB device in and its vendor ID matches the ID for joypad (c219), udev could assign the device the name ``/dev/input/joypad`` instead of the more generic ``/dev/ttyACMx``. This allows our configuration scripts to refer to things like ``/dev/sensors/vesc``, which do not depend on the order in which the devices were initialized. We will use udev to assign persistent device names to the VESC, and joypad by creating two configuration files (“rules”) in the directory ``/etc/udev/rules.d``. Since our LiDar is using Ethernet as the input, check out the `Hokuyo LiDar Setup </Hokuyo_Lidar/Hokuyo.md>`_

First, **as root**, open ``/etc/udev/rules.d/99-vesc.rules`` and copy in the following rule for the VESC:

.. code-block:: bash

    KERNEL=="ttyACM[0-9]*", ACTION=="add", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0666", GROUP="dialout", SYMLINK+="sensors/vesc"

Then open ``/etc/udev/rules.d/99-joypad-f710.rules`` and add this rule for the joypad:  

(The default setting of joypad is "D" mode. For some unknown reasons, our NX can't recognize our Joypad in "D" mode, even we can read the device by command ``$ lsusb``. Therefore, we are using "X" mode, and changed the joypad_config file axis setting and "idProduct" to c21X. If you have same issue, try it out.) 

.. code-block:: bash

    KERNEL=="js[0-9]*", ACTION=="add", ATTRS{idVendor}=="046d", ATTRS{idProduct}=="c219", SYMLINK+="input/joypad-f710"

Finally, trigger (activate) the rules by running

.. code-block:: bash

    sudo udevadm control --reload-rules
    sudo udevadm trigger

Reboot your system, and you should find three new devices by running

.. code-block:: bash

    ls /dev/sensors

and:

.. code-block:: bash

    ls /dev/input

If you want to add additional devices and don’t know their vendor or product IDs, you can use the command

.. code-block:: bash

    sudo udevadm info --name=<your_device_name> --attribute-walk

making sure to replace ``<your_device_name>`` with the name of your device (e.g. ttyACM0 if that’s what the OS assigned it. The Unix utility dmesg can help you find that). The topmost entry will be the entry for your device; lower entries are for the device’s parents.



.. _install_ros2:
2. Installing ROS 2 and its Utilities
---------------------------------------

Begin by following the instructions provided in `the official ROS 2 Foxy Installation Guide <https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html>`_ to install ROS 2 via Debian Packages.

Subsequently, ensure to install ``colcon`` as the primary build tool for ROS 2. Execute the steps outlined in the tutorial available `here <https://docs.ros.org/en/foxy/Tutorials/Colcon-Tutorial.html?highlight=colcon#install-colcon>`_.

Finally, install ``rosdep`` as the dependency resolution tool, as directed in the following `instructions <https://docs.ros.org/en/foxy/How-To-Guides/Building-a-Custom-Debian-Package.html?highlight=rosdep#install-dependencies>`_ and initialize it using the provided `guidelines <https://docs.ros.org/en/foxy/How-To-Guides/Building-a-Custom-Debian-Package.html?highlight=rosdep#install-dependencies>`_.

Should you encounter missing dependencies due to the version being outdated, manually install them using the following command format:

.. code-block:: bash

    sudo apt-get install ros-[distro]-[package-name]

.. _software_stack:
3. Setting up the Driver Stack
----------------------------------

To initialize the ROS 2 workspace for our driver stack, execute the commands below, utilizing ``f1tenth_ws`` as the designated workspace name for this section:

.. code-block:: bash

    cd $HOME
    mkdir -p f1tenth_ws/src
    cd f1tenth_ws
    colcon build

Proceed to clone the repository into the ``src`` directory of our workspace:

.. code-block:: bash

    cd src
    git clone https://github.com/f1tenth/f1tenth_system.git

Update the git submodules and pull in all the necessary packages:

.. code-block:: bash

    cd f1tenth_system
    git submodule update --init --force --remote

After completing the cloning process, install all dependencies for our packages using ``rosdep``:

.. code-block:: bash

    cd $HOME/f1tenth_ws
    rosdep update
    rosdep install --from-paths src -i -y

Finally, build the workspace again to incorporate the driver stack packages:

.. code-block:: bash

    colcon build

For more detailed information on the driver setup, refer to the README of the `f1tenth_system repository <https://github.com/f1tenth/f1tenth_system>`_.

.. _teleop_setup:

4. Launching Teleop and Testing the LiDAR
----------------------------------------------

This section assumes that the lidar has already been connected to the Ethernet port. If you are utilizing the Hokuyo 10LX, ensure that you have completed the `Hokuyo LiDar Setup </Hokuyo_Lidar/Hokuyo.md>`_ section before proceeding.

Before initiating the bringup launch, ensure to configure the parameters according to the LiDAR being used in the ``sensors.yaml`` params file, located at:

.. code-block:: bash

    $HOME/f1tenth_ws/src/f1tenth_system/f1tenth_stack/config/

Set the ``ip_address`` field to correspond with the IP address of your LiDAR.

Prior to launching the ROS2 launch files, remember to connect to your car using ``$ ssh -X f1tenth@ip_address``. Failure to do so will result in the inability to view the Rviz GUI. Additionally, on Ubuntu 20.04, you need to install **OpenGL** on your host laptop to open the Rviz GUI. You can install OpenGL using the following command:

.. code-block:: bash

    glxinfo | grep "OpenGL version"
    sudo apt-get install mesa-utils
    glxgears

To set up the ROS 2 underlay and our workspace's overlay, run the following commands in your running container:

.. code-block:: bash

    source /opt/ros/foxy/setup.bash
    cd $HOME/f1tenth_ws
    source install/setup.bash

Subsequently, launch the bringup with:

.. code-block:: bash

    ros2 launch f1tenth_stack bringup_launch.py

Executing the bringup launch will initiate the VESC drivers, LiDAR drivers, joystick drivers, and all necessary packages for operating the car. To view the LaserScan messages, open a new terminal window and execute:

.. code-block:: bash

    source /opt/ros/foxy/setup.bash
    cd $HOME/f1tenth_ws
    source install/setup.bash
    rviz2

The Rviz window should appear, allowing you to add a LaserScan visualization on the ``/scan`` topic. The map used in bring_up.py is "Base_link".

**Reference:**

xLab at the University of Pennsylvania. (2021). Build. https://f1tenth.org/build