==========================
Object Detection System
==========================

Overview
============
This repository hosts a Yolo V5-based object detection system designed to identify obstacles and measure their distance from the origin in a vehicle's coordinate system, as well as the width of the obstacles. Utilizing the Intel RealSense D415 depth camera, the system provides accurate depth information that is integrated with the object detection capabilities of Yolo V5 to enhance autonomous navigation tasks and environmental awareness for robotic applications.

This system integrates advanced image processing techniques with deep learning and 3D sensing technologies. By leveraging the high-resolution color and depth data from the RealSense D415 camera, the system can accurately detect objects and calculate their distances and dimensions, which are crucial for tasks such as autonomous driving and robotic navigation.

Introduction
=============
The main components of this function include:

- **RealSense Camera Integration**: Manages the data streams from the Intel RealSense D415 camera, capturing both RGB and depth information.
- **Yolo V5 Object Detection**: Utilizes a pre-trained Yolo V5 model to detect objects in the RGB images. The model is configured through a YAML file, which defines parameters such as input size, model weights, and detection thresholds.
- **Image Processing**: Processes both the depth and RGB images to align them and scale the depth data for visualization and further analysis.
- **Detection and Measurement**: After detecting objects, the system calculates their distances from the camera and estimates their dimensions using the depth data provided by the RealSense camera.
- **Visualization**: Displays the detection results in real-time, overlaying bounding boxes and measurements on the video feed.

Usage
=====
To use the object detection system, navigate to the system's directory and run the Python script:

.. code-block:: bash

    cd Yolov5/RealSenseD415/obstacle_detection
    python3 obstacle_detect.py

This will start the system, and you will begin to see the video feed with detected objects. Each object will be highlighted, and its distance from the camera along with its dimensions will be displayed.

Below is a GIF demonstrating the expected output when the system is running correctly:

.. image:: ../Images/yolo.gif
   :alt: Object Detection Demo
   :align: center

Note that the system returns the width and depth of detected obstacles as well as their positions in camera coordinates. However, it is important to understand that the reported dimensions and distances pertain to the bounding boxes around the detected objects, not the actual physical dimensions or the exact center depth of the objects. 

Model Configuration
===================
You can customize the detection system by modifying parameters in the configuration file. Here is an example of the `config` file used for model setup:

.. code-block:: yaml

    ####################################
    # YoloV5 model weight path
    weight: "weights/best.pt"
    # Input image size
    input_size: 640
    # Number of classes
    class_num:  1
    # Class names
    class_name: [ 'Box_Obstacle' ]
    # Threshold settings
    threshold:
      iou: 0.45
      confidence: 0.1
    # Computing device
    # - cpu
    # - 0 <- Use GPU
    device: '0'

The configuration file allows you to customize parameters such as the path to the model weights, input image size, number of classes, class names, thresholds for intersection-over-union and confidence, and the computing device. Adjust these settings to optimize performance for your specific requirements.

Additionally, you have the option to train your own model using the YoloV5 framework. It is important to use the version compatible with our setup (YoloV5-5.0 is OK). For training, approximately 150 images with bounding boxes were used. Ensure that your dataset includes images from various angles and under diverse lighting conditions to enhance model robustness and detection accuracy. This varied dataset helps the model generalize better to new environments.



Future Work
===========
Future developments for this system include:

- Integrating the detected obstacle information into the Rviz dynamic layer map to assist with dynamic routing decisions.

- Developing predictive models to forecast the movements of dynamic obstacles based on observed behaviors over short time intervals.

References
==========
- https://github.com/ultralytics/yolov5
- https://github.com/mushroom-x/yolov5-simple

