.. _doc_drive_autonomous:

Autonomous Control
=====================

This section goes through how to subscribe to sensor topics, and how to publish drive topic to control the car.


Sensor Topics
---------------
* ``/scan``: this topic maintains the ``LaserScan`` messages published by the LiDAR.
* ``/odom``: this topic maintains the ``Odometry`` messages published by the VESC.
* ``/sensors/imu/raw``: this topic maintains the ``Imu`` messages published by the VESC.
* ``/sensors/core``: this topic maintains the ``VescStateStamped`` messages published by the VESC on telemetry data.

Drive Topic
---------------
* ``/drive``: this topic is listened to by the VESC, needs ``AckermannDriveStamped`` messages. The ``speed`` and ``steering_angle`` fields in the ``drive`` field of these messages are used to command desired steering and velocity to the car.

Bringup and Deadman's Switch
-------------------------------
To enable autonomous control, all you have to do is launch the bring up launch described in `Driver stack Setup <driver_stack_setup.rst>`_. Then, publish to the ``drive`` topic, and hold the Deadman's Switch for those messages to pass through. The Deadman's Switch for Autonomous Control is by default the **RB** button on the joystick.

Autonomous Control Node
--------------------------------------------------
In our node, it is designed to control a vehicle's movement by sending AckermannDriveStamped messages to the ``/drive`` topic, allowing the vehicle to advance a set distance. The node subscribes to the ``/odom`` topic to track the vehicle's position and to the ``/emergency_breaking`` topic for implementing safety features. An automatic emergency braking (AEB) system is integrated within this node. The AEB system activates to halt the vehicle immediately if an impending collision is detected, enhancing safety during operation.

.. image:: /Images/autonomous_drive.gif
   :alt: Demonstration of Autonomous Driving
   :align: center

