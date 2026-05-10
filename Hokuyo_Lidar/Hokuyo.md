# F1-Tenth_Duke
## Hokuyo 10LX Ethernet Connection Setup

## Table of Contents
1. [What is Hokuyo](#WhatisHokuyo)
2. [Equipment Required](#equipment)
3. [Steps](#Steps)



<a name="WhatisHokuyo"></a>
### What is Hokuyo
Hokuyo 10LX is a Lidar for map generation and vehicle location in real time. Hokuyo 10LX has two versions to choose way of connection. What we got for this vehicle is connected by Ethernet. Please skip this section if you have a 30LX or a Lidar that connects via USB.

<a name="equipment"></a>
### Equipment Required
1. Fully built F1TENTH vehicle with a Hokuyo 10LX Lidar
2. Pit/Host Laptop OR
3. External Monitor/display, HDMI cable, keyboard, mouse

<a name="Steps"></a>
### Steps
1. Connect to the Jetson NX either via SSH or a wired connection (monitor, keyboard, mouse)

2. In order to utilize the 10LX you must first configure the eth0 network. From the factory the 10LX is assigned the following ip: ``192.168.0.10``. Note that the lidar is on subnet 0.

3. Open Network Configuration in the Linux GUI on the Jetson NX. In the ipv4 tab, add a route such that the eth0 port is assigned

IP address ``192.168.0.15``
Subnet mask ``255.255.255.0``
Gateway ``192.168.0.10``

Call the connection ``Hokuyo``. Save the connection and close the network configuration GUI.

When you plug in the 10LX, make sure that the Hokuyo connection is selected. If everything is configured properly, you should now be able to ping ``192.168.0.10``.





