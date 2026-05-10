


Configuring the NVIDIA Jetson NX
=========================================
**Required Equipment:**

- A Pit/Host laptop or computer equipped with Linux operating system.
- An NVIDIA Jetson NX, along with peripherals such as a keyboard, mouse, and an external monitor connected via HDMI.
- A WiFi connection.
- A USB-typeC/Micro-USB calbe to connect Jeston NX to Host laptop 

Introduction
------------
A Jeston NX board need to intall an operation system and other necessary components on it. This is an instruction to teach you how to install those necessary components and don't blow up the memory.


1. Flash Jetson NX with NVIDIA SDK Manager
----------------------------------

Before we start, you need to connect FC REC and GND using jumpers to enter the recovery mode.

1. Install SDK manager in your host Linux computer.



2. Open SDK manager with command 'sdkmanager' and login to your NVIDIA account. If you don't have a NVIDIA account, you can sign up in the NVIDIA official website.

.. code-block:: bash

   sdkmanager

3. Choose Target hardware and operating system in SDK manager.

Use the USB to Micro-USB line to connect your target device and host computer. Then the target hardware shoulb be deteced automatically.

We suggest not select Host Machine if you don't need NVIDIA components in your host computer. And highly recoomand that don't select DeepStream, becasue 16GB storage space for our board is very limitted. 
As for Target Operating System, you can choose a version that is compatible with your system.


        .. figure:: /Images/Screenshot-6.jpg
                :align: center





4. Select components you want. 

If your board have enough storage space, we recommand you to install CUDA kit as well, which is necessary for SLAM. If you are using Jetson NX like us, please only install Jetson OS in this step. We will tell you how to expand your memory via SSD in following step. *Please Don't Try Even You Have 16GB*

        .. figure:: /Images/Screenshot-4.jpg
                :align: center

            
5. Install the system

Enter your passward to start installation.

        .. figure:: /Images/Screenshot-1.jpg
                :align: center

You'll be prompted to configure the new system for your reComputer. Select 'Manual setup' for forced recovery mode and opt for the default 'Pre-Config'.

        .. figure:: /Images/Screenshot-5.jpg
                :align: center

After that, setup the new USERNAME and PASSWAORD for your system.


6. Installation Process 

It needs a while to install all target components. Please wait patiently until it's all set up.

        .. figure:: /Images/Screenshot-9.jpg
                :align: center





2. Set the SSD as the boot disk
---------------------------

With the Jetson NX powered off, install the SSD and power it on. You can check the instruction `SSD Setup <SSD.md>`_.

Then, we can move all system file to SSD and set the SSD as your boost drive. We were using the package from "JetsonHacks" https://jetsonhacks.com/2020/05/29/jetson-xavier-nx-run-from-ssd/.

.. code-block:: bash

   $ git clone https://github.com/jetsonhacks/rootOnNVMe

Switch over to that repo's directory.

.. code-block:: bash

   $ cd rootOnNVMe

Next, copy the rootfs of the eMMC/SD card to the SSD

.. code-block:: bash

        $ ./copy-rootfs-ssd.sh

Finally, we will add a service which will run a script when the system starts up. The script will “pivot the root” to the SSD so that the system will run from the SSD.

.. code-block:: bash

        $ ./setup-service.sh


Reboot Jetson NX and view Disks. If Flilesystem Root is mounted to an SSD, the system has been boosted on the SSD.

        .. figure:: /Images/successSSD.png
                :align: center

Now you have enough storage space to download any NVIDIA component as long as you need.

3. Configuring WiFi and SSH (Skip this step for temporarily using)
-------------------------------
This instruction is only for static IP setup. Static IP is only useful for long term testing under same Wifi environment, so we can ssh the car with same IP address. If you would not connect the same wifi for long term, you don't need to setup static IP. Connect the wifi as usual. Use the command ``ip addr show dev wlan0``, then use that IP address for ssh. (e.g. ``ssh f1tenth@your_current_IP``)

If you are trying to setup a static IP in school or other public Wifi. Please contact your IT office to assign a static IP for your device. You also can find the MAC address by using command ``ip addr showdev wlan0``, and find ``link/ether XX:XX:XX:XX:XX:XX``.
For Duke students, if your are using a new Jetson NX or something, create a ticket to request a static IP and register your device. This process might take 3-5 days. And skip the following step of setup.

1. Before you setup the static IP for your wifi, turn off wifi radio and forget the wifi you want to connect. Follow the command to configure the WiFi. To determine your WiFi adapter's interface name, run nmcli d. Use the displayed name (e.g., wlan0) in upcoming commands, replacing it with your actual interface name as required.

        .. figure:: /Images/Wifi_1.png
                :align: center

                WiFi network selection. 

2. To activate your WiFi radio, enter ``nmcli r wifi on`` in the terminal. There won’t be any output if the command is successful.
3. To see the list of WiFi SSIDs that your WiFi adapter can see, type ``nmcli d wifi list`` and hit ENTER. After the list is printed, hit ``q`` to continue.

        .. figure:: /Images/Wifi_2.png
                :align: center

                WiFi SSID selection. 

4. To connect to a specific WiFi SSID, use the command ``sudo nmcli d wifi connect [SSID] password [PASSWORD]`` where ``[SSID]`` is replaced with the SSID with which you want to connect and ``[PASSWORD]`` is replaced with the password to connect to that SSID. Hit ENTER.
5. If the connection was successful, you should see the message ``Device 'wlan0' successfully activated with [GUID]``.
6. WiFi defaults to DHCP, giving a new IP on each reboot. We'll set a static IP instead, needing your network's subnet, IP range, and gateway.
7. To get the currently-assigned IP address use the command ``ip addr show dev wlan0``. You can see the highligt in the following picture. Save this IP address and set it as your static IP later.

        .. figure:: /Images/Wifi_3.png
                :align: center

                Currently-connected WiFi IP address. 

8. To set a static IP address, you will also need to know the name of the connection. This is usually the same as the SSID of the WiFi network but not always. To see the list of current connections, use the command ``nmcli c show``.

        .. figure:: /Images/nx-wifi-step-8.png
                :align: center

                List of connections. (F1Tenth, 2021)

9. To configure a static IP, use sudo nmcli c mod [CONNECTION_NAME] ipv4.address [NEW_ADDRESS]/[CIDR]. Replace [CONNECTION_NAME] with your WiFi name from step 8, [NEW_ADDRESS] with your current IP, and [CIDR] with your subnet's CIDR (often 24 for family wifi). The new IP address has to be the same IP as above.

        .. figure:: /Images/nx-wifi-step-9.png
                :align: center

                Setting static IP address. (F1Tenth, 2021)

10. Set the connection's gateway with sudo nmcli c mod [CONNECTION_NAME] ipv4.gateway [GATEWAY_IP], replacing [CONNECTION_NAME] with your WiFi name from step 8 and [GATEWAY_IP] with your router's IP.

        .. figure:: /Images/nx-wifi-step-10.png
                :align: center

                Setting IP gateway. (F1Tenth, 2021)
 
11. Set DNS servers via `sudo nmcli c mod [CONNECTION_NAME] ipv4.dns "[DNS_SERVERS]"`, substituting `[CONNECTION_NAME]` with your WiFi name from step 8 and `[DNS_SERVERS]` with DNS IPs, e.g., 8.8.8.8, 8.8.4.4.
12. Disable DHCP for a static IP with `sudo nmcli c mod [CONNECTION_NAME] ipv4.method manual`, replacing `[CONNECTION_NAME]` with your WiFi connection name from step 8. 

        .. figure:: /Images/nx-wifi-step-12.png
                :align: center

                Setting connection to always use static IP. (F1Tenth, 2021)

13. To save the changes you've made, run the command ``sudo nmcli c up [CONNECTION_NAME]`` where ``[CONNECTION_NAME]`` is replaced with the name of your WiFi connection that you got from step 8.

14. To test SSH into the NVIDIA Jetson Xavier NX, ensure it's on the same network as the Host PC. Use `ssh f1tenth@[IP_ADDRESS]` with the Developer Kit's static IP. Once SSH is confirmed, close the terminal.

4. Updating Packages
------------------------

All further steps assume that your NVIDIA Jetson Xavier NX Developer Kit is connected to the internet. You can execute all the commands directly in the terminal application of the NVIDIA Jetson. Now we are updating the Ubuntu system on the Jetson NX.

1. To update the list of available packages, run ``sudo apt update``.
2. To install all available updates, run ``sudo apt full-upgrade``.
3. Once all packages have been upgraded run ``sudo reboot`` to restart the Developer Kit and apply any changes.

**Reference:** 

xLab at the University of Pennsylvani. (2021). Build. https://f1tenth.org/build 

NVIDIA Flash JestPack https://wiki.seeedstudio.com/reComputer_J2021_J202_Flash_Jetpack/

