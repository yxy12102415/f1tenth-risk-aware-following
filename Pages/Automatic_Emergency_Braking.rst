Automatic Emergency Braking (AEB)
=================================

Overview
--------

The goal of this lab is to develop a safety node for the race cars that will stop the car from colliding when travelling at higher velocities. We will implement Instantaneous Time to Collision (iTTC) using the ``LaserScan`` message in the simulator.

The TTC Calculation
-------------------

Time to Collision (TTC) is the time it would take for the car to collide with an obstacle if it maintained its current heading and velocity. We approximate the time to collision using Instantaneous Time to Collision (iTTC), which is the ratio of instantaneous range to range rate calculated from current range measurements and velocity measurements of the vehicle.

As discussed in the lecture, we can calculate the iTTC as:

.. math::

   iTTC = \frac{r}{\{- \dot{r}\}_{+}}

where:

- :math:`r` is the instantaneous range measurements.
- :math:`\dot{r}` is the current range rate for that measurement.
- :math:`\{\}_{+}` is defined as :math:`\{x\}_{+} = \text{max}(x, 0)`.

The instantaneous range :math:`r` to an obstacle is easily obtained by using the current measurements from the ``LaserScan`` message, as the LiDAR effectively measures the distance from the sensor to an obstacle. The range rate :math:`\dot{r}`, the rate of change along each scan beam, can be calculated in two ways:

1. By mapping the vehicle's current longitudinal velocity onto each scan beam's angle using :math:`v_x \cos{\theta_{i}}`. Be careful with assigning the range rate a positive or a negative value. The angles can also be determined by the information in ``LaserScan`` messages.

2. By taking the difference between the previous range measurement and the current one, dividing it by how much time has passed in between (timestamps are available in message headers), to calculate the range rate.

Note the negation in the calculation. This is to correctly interpret whether the range measurement should be decreasing or increasing. For a vehicle travelling forward towards an obstacle, the corresponding range rate for the beam right in front of the vehicle should be negative, as the range measurement should be shrinking. Conversely, the range rate corresponding to the vehicle travelling away from an obstacle should be positive, as the range measurement should be increasing. The operator is in place so the iTTC calculation will be meaningful. When the range rate is positive, the operator will ensure iTTC for that angle goes to infinity.

After your calculations, you should end up with an array of iTTCs that correspond to each angle. When a time to collision drops below a certain threshold, it means a collision is imminent.

Automatic Emergency Braking with iTTC
-------------------------------------

**Run the Safety Node:**

Use the following command to run the safety node:

.. code-block:: bash

    ros2 run safety_node safety_node

**Node Operation:**

Upon launching the safety node, it performs the following actions:

- **Sensor Data Subscription:** The node subscribes to the ``/scan`` and ``/odom`` topics from the LiDAR and vehicle odometry respectively, gathering data on the vehicle's current position, speed, and distance to obstacles.

- **iTTC Calculation:** We use the fisrt method to calulate iTTC. It calculates iTTC for each scan angle by analyzing the distances and vehicle speed from the LiDAR data. iTTC serves as an indicator of the time remaining before a collision might occur if the current speed and trajectory are maintained.

- **Emergency Braking Decision:** If any iTTC value falls below a predefined safety threshold, the node deduces that a collision is imminent and issues an emergency braking command, setting the vehicle speed to zero to avert a potential collision.

- **Emergency Braking Signal Publishing:** In addition to controlling the vehicle's speed, the node broadcasts a Boolean signal over the ``/emergency_braking`` topic to activate other safety measures or notify other parts of the system.

This node can be effectively integrated and tested within a simulated or real-world environment for enhanced vehicle safety.

Limitations with Joypad Control
-------------------------------

**Caution with Joypad Use:**

It is important to note that the effectiveness of the ``safety_node`` is reduced when the vehicle is controlled via a joypad. The underlying mechanism of the node involves sending a message with ``speed=0`` to the ``drive`` topic to initiate braking. However, during joypad-controlled operation, collisions often occur due to delayed reactions of our human and the inability to adjust the speed swiftly enough—even with braking initiated.

When emergency braking is triggered, if the joypad control continues to send speed messages to the ``drive`` topic, it conflicts with the ``speed=0`` message sent by the ``safety_node``. This conflict results in the vehicle's speed stuttering rather than coming to a prompt halt. 

**Primary Use Case: Autonomous Drive:**

Therefore, the primary application of the ``safety_node`` is in autonomous driving scenarios. In these scenarios, there are no continuous speed messages from a joypad, which allows the emergency braking system to function effectively without interference. This setup ensures that the vehicle can achieve a timely and effective stop in response to imminent collisions detected by the system. You can find more information in autonomous drive.

.. image:: /Images/autonomous_drive.gif
   :alt: Demonstration of Autonomous Driving
   :align: center

