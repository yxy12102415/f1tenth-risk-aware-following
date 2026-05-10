Depth_Camera_D415 Configuration
================================

You can follow the instructions on the website `here <https://www.intelrealsense.com/get-started-depth-camera/>`_ to configure, or you can follow the instructions in this document.

Connect Device
--------------

Firstly, you need to mount the camera on the bracket, then use the provided USB-C cable to connect the camera to the computer.  It is very easy.

        .. figure:: /Images/Depth_Camera_1.jpg
                :align: center

Install necessary packages
-----------------------------

**Add the public key of the server to the registry:**

.. code-block:: bash

   sudo mkdir -p /etc/apt/keyrings
   curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null

**Ensure that the system has support for HTTPS in apt:**

.. code-block:: bash

   sudo mkdir -p /etc/apt/keyrings
   sudo apt-get install apt-transport-https

**Include the server in the repository list:**

.. code-block:: bash

   echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
   sudo tee /etc/apt/sources.list.d/librealsense.list
   sudo apt-get update

**Install the required libraries:**

.. code-block:: bash

   sudo apt-get install librealsense2-dkms
   sudo apt-get install librealsense2-utils

Realsense viewer
----------------

Reconnect the Intel RealSense depth camera and execute the command to confirm that the installation was successful.

.. code-block:: bash

   realsense-viewer

You could turn on the button of Stereo Module and RGB Camera to check out.
If everything is ok, you should be able to see 2D and 3D view as follow.

        .. figure:: /Images/Depth_Camera_2.png
                :align: center

        .. figure:: /Images/Depth_Camera_3.png
                :align: center



