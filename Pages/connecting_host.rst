
Connecting the Pit/Host Computer to the NVIDIA Jetson NX
========================================================

**Required Equipment:**

- A Pit/Host laptop or computer equipped with Mac, Windows, or Linux operating system.
- An F1TENTH vehicle fully assembled with its NVIDIA Jetson NX, along with peripherals such as a keyboard, mouse, and an external monitor connected via HDMI.
- A WiFi connection.

Introduction
------------

Gaining remote access to the NVIDIA Jetson NX on the F1TENTH vehicle enhances the driving experience by allowing operations from a distance. This guide walks you through configuring network settings on both the Jetson NX and your Pit/Host computer. Accurate configuration is crucial to avoid connection issues, primarily due to incorrect IP address usage.

For WiFi connectivity, ensure both the Pit/Host computer and the F1TENTH vehicle are connected to a wireless router. This guide will refer to the router's WiFi network as ``F1TENTH_WIFI``, which you will replace with your router's actual SSID.

Steps for Configuration
------------------------

1. Setting Up the Vehicle Hardware
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The NVIDIA Jetson NX is equipped with a network card. First, ensure its antennas are correctly attached. Connect the vehicle's battery and switch on the Powerboard.

2. WiFi Connection for NVIDIA Jetson NX
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Refer to the instructions in `Configuring the NVIDIA Jetson NX <configuring_nx.rst>`_
for WiFi and SSH setup. Once connected to the same wifi your car is connecting, verify the connection by opening a terminal and running:

.. code-block:: bash

   ifconfig

This command displays network interface information. Your car's IP address is listed under ``wlan0`` following ``inet``. In the provided example, the address is ``195.0.0.5``.

3. Pit/Host Computer WiFi Setup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Connect your Pit/Host laptop to ``F1TENTH_WIFI`` and determine its IP address. The process varies by operating system:

- On Linux or macOS, use the ``ifconfig`` command. The IP might be listed under ``en0`` or ``en1`` for macOS users.
- For Linux running on a VM, ensure the network adapter is set to NAT mode for shared internet access.

4. Establishing Connection Between Pit/Host and NVIDIA Jetson NX
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

With both devices on the same network, verify connectivity by pinging each other:

- From the Jetson NX, ping the Pit/Host using its IP address.
- From the Pit/Host, ping the Jetson NX using its IP address.

Replace the sample IP addresses with the actual ones from your setup.

To remotely access the Jetson NX from the Pit/Host, use ``ssh`` (on macOS or Linux) or ``PuTTY`` (on Windows). It's advisable to use ``tmux`` for a persistent session that remains active even if the terminal is closed.

**Reference:**

xLab at the University of Pennsylvania. (2021). Build. https://f1tenth.org/build
