# Import dependencies
import random
from utils.torch_utils import select_device, load_classifier, time_sync
from utils.general import (
    check_img_size, non_max_suppression, apply_classifier, scale_coords,
    xyxy2xywh, strip_optimizer, set_logging)
from utils.datasets import LoadStreams, LoadImages, letterbox
from models.experimental import attempt_load

import torch.backends.cudnn as cudnn
import torch

import pyrealsense2 as rs
import math
import yaml
import argparse
import os
import time
import numpy as np
import sys

import cv2
# PyTorch
# YoloV5-PyTorch

pipeline = rs.pipeline()  # Define the pipeline
config = rs.config()  # Define the configuration
config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)
profile = pipeline.start(config)  # Start the pipeline
align_to = rs.stream.color  # Align to the color stream
align = rs.align(align_to)


def get_aligned_images():
    frames = pipeline.wait_for_frames()  # Wait to obtain frames
    aligned_frames = align.process(frames)  # Get aligned frames
    aligned_depth_frame = aligned_frames.get_depth_frame()  # Get the depth frame from aligned frames
    color_frame = aligned_frames.get_color_frame()  # Get the color frame from aligned frames

    ############### Camera Parameters Retrieval #######################
    intr = color_frame.profile.as_video_stream_profile().intrinsics  # Get camera intrinsics
    depth_intrin = aligned_depth_frame.profile.as_video_stream_profile(
    ).intrinsics  # Get depth parameters (used for pixel coordinates to camera coordinates conversion)


    #######################################################

    depth_image = np.asanyarray(aligned_depth_frame.get_data())  # Depth image (default 16-bit)
    depth_image_8bit = cv2.convertScaleAbs(depth_image, alpha=0.03)  # Depth image (8-bit)
    depth_image_3d = np.dstack(
        (depth_image_8bit, depth_image_8bit, depth_image_8bit))  # 3-channel depth image
    color_image = np.asanyarray(color_frame.get_data())  # RGB image

    # Return camera intrinsics, depth parameters, color image, depth image, aligned depth frame
    return intr, depth_intrin, color_image, depth_image, aligned_depth_frame

#yolov5s.yaml

class YoloV5:
    def __init__(self, yolov5_yaml_path='config/yolo5s.yaml'):
        '''Initialization'''
        # Load configuration file
        with open(yolov5_yaml_path, 'r', encoding='utf-8') as f:
            self.yolov5 = yaml.load(f.read(), Loader=yaml.SafeLoader)
        # Randomly generate colors for each class
        self.colors = [[np.random.randint(0, 255) for _ in range(
            3)] for class_id in range(self.yolov5['class_num'])]
        # Initialize model
        self.init_model()

    @torch.no_grad()
    def init_model(self):
        '''Model Initialization'''
        # Set logging output
        set_logging()
        # Choose compute device
        device = select_device(self.yolov5['device'])
        # Use half-precision floats if using GPU
        is_half = device.type != 'cpu'
        # Load model
        model = attempt_load(
            self.yolov5['weight'], map_location=device)  # Load full precision model
        input_size = check_img_size(
            self.yolov5['input_size'], s=model.stride.max())  # Check model size
        if is_half:
            model.half()  # Convert model to half precision
        # Set BenchMark to speed up inference of fixed image size
        cudnn.benchmark = True
        # Initialize image buffer
        img_torch = torch.zeros(
            (1, 3, self.yolov5['input_size'], self.yolov5['input_size']), device=device)  # init img
        # Create model
        # Run once
        _ = model(img_torch.half() if is_half else img_torch) if device.type != 'cpu' else None
        self.is_half = is_half  # Enable half precision
        self.device = device  # Compute device
        self.model = model  # Yolov5 model
        self.img_torch = img_torch  # Image buffer

    def preprocessing(self, img):
        '''Image Preprocessing'''
        # Image resizing
        # Note: auto must be set to False -> image width and height are not the same
        img_resize = letterbox(img, new_shape=(
            self.yolov5['input_size'], self.yolov5['input_size']), auto=False)[0]
        # Add an extra dimension
        img_arr = np.stack([img_resize], 0)
        # Image conversion (Convert) BGR to RGB
        img_arr = img_arr[:, :, :, ::-1].transpose(0, 3, 1, 2)
        # Normalize values
        img_arr = np.ascontiguousarray(img_arr)
        return img_arr

    @torch.no_grad()
    def detect(self, img, canvas=None, view_img=True):
        '''Model Prediction'''
        # Image preprocessing
        img_resize = self.preprocessing(img)  # Image resizing
        self.img_torch = torch.from_numpy(img_resize).to(self.device)  # Convert image format
        self.img_torch = self.img_torch.half() if self.is_half else self.img_torch.float()  # Format conversion uint8-> float
        self.img_torch /= 255.0  # Normalize image
        if self.img_torch.ndimension() == 3:
            self.img_torch = self.img_torch.unsqueeze(0)
        # Model inference
        t1 = time_sync()
        pred = self.model(self.img_torch, augment=False)[0]
        # NMS (Non-Maximum Suppression)
        pred = non_max_suppression(pred, self.yolov5['threshold']['confidence'],
                                   self.yolov5['threshold']['iou'], classes=None, agnostic=False)
        t2 = time_sync()
        # Get detection results
        det = pred[0]
        gain_whwh = torch.tensor(img.shape)[[1, 0, 1, 0]]  # [w, h, w, h]

        if view_img and canvas is None:
            canvas = np.copy(img)
        xyxy_list = []
        conf_list = []
        class_id_list = []
        if det is not None and len(det):
            # Rescale coordinates to original image size
            det[:, :4] = scale_coords(
                img_resize.shape[2:], det[:, :4], img.shape).round()
            for *xyxy, conf, class_id in reversed(det):
                class_id = int(class_id)
                xyxy_list.append(xyxy)
                conf_list.append(conf)
                class_id_list.append(class_id)
                if view_img:
                    # Draw bounding box and label
                    label = '%s %.2f' % (
                        self.yolov5['class_name'][class_id], conf)
                    self.plot_one_box(
                        xyxy, canvas, label=label, color=self.colors[class_id], line_thickness=3)
        return canvas, class_id_list, xyxy_list, conf_list

    def plot_one_box(self, x, img, color=None, label=None, line_thickness=None):
        '''Draw Rectangle + Label'''
        tl = line_thickness or round(
            0.002 * (img.shape[0] + img.shape[1]) / 2) + 1  # line/font thickness
        color = color or [random.randint(0, 255) for _ in range(3)]
        c1, c2 = (int(x[0]), int(x[1])), (int(x[2]), int(x[3]))
        cv2.rectangle(img, c1, c2, color, thickness=tl, lineType=cv2.LINE_AA)
        if label:
            tf = max(tl - 1, 1)  # font thickness
            t_size = cv2.getTextSize(
                label, 0, fontScale=tl / 3, thickness=tf)[0]
            c2 = c1[0] + t_size[0], c1[1] - t_size[1] - 3
            cv2.rectangle(img, c1, c2, color, -1, cv2.LINE_AA)  # filled
            cv2.putText(img, label, (c1[0], c1[1] - 2), 0, tl / 3,
                        [225, 255, 255], thickness=tf, lineType=cv2.LINE_AA)


if __name__ == '__main__':
    print("[INFO] YoloV5 Object Detection - Program Start")
    print("[INFO] Starting YoloV5 Model Load")
    # Path to the YOLOV5 model configuration file (YAML format) yolov5_yaml_path
    model = YoloV5(yolov5_yaml_path='config/yolov5s.yaml')
    print("[INFO] YoloV5 Model Load Complete")

    try:
        while True:
            # Wait for a coherent pair of frames: depth and color
            intr, depth_intrin, color_image, depth_image, aligned_depth_frame = get_aligned_images()  # Get aligned images and camera parameters
            if not depth_image.any() or not color_image.any():
                continue
            # Convert images to numpy arrays
            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(
                depth_image, alpha=0.03), cv2.COLORMAP_JET)
            # Stack both images horizontally
            images = np.hstack((color_image, depth_colormap))
            
            # Show images

            t_start = time.time()  # Start timing
            # YoloV5 Object Detection
            canvas, class_id_list, xyxy_list, conf_list = model.detect(
                color_image)

            t_end = time.time()  # End timing
            #canvas = np.hstack((canvas, depth_colormap))
            #print(class_id_list)

            camera_xyz_list=[]
            if xyxy_list:
                for i in range(len(xyxy_list)):
                    ux = int((xyxy_list[i][0]+xyxy_list[i][2])/2)  # Calculate x in pixel coordinates
                    uy = int((xyxy_list[i][1]+xyxy_list[i][3])/2)  # Calculate y in pixel coordinates
                    
                    w_pixel = xyxy_list[i][2] - xyxy_list[i][0]  # Calculate the pixel width of the bounding box
                    
                    dis = aligned_depth_frame.get_distance(ux, uy)
                    camera_xyz = rs.rs2_deproject_pixel_to_point(
                        depth_intrin, (ux, uy), dis)  # Calculate xyz in camera coordinates
                    
                    #width
                    w_real = w_pixel * dis / intr.fx

                    
                    camera_xyz = np.round(np.array(camera_xyz), 3)  # Round to three decimal places
                    camera_xyz = camera_xyz.tolist()
                    cv2.circle(canvas, (ux,uy), 4, (255, 255, 255), 5)  # Mark the center point
                    
                    # Label to show xyz and width
                    label = "Distance: {:.2f} Width: {:.2f}".format(camera_xyz[2], w_real)
                    
                    cv2.putText(canvas, label, (ux+20, uy+10), 0, 1,
                                [225, 255, 255], thickness=2, lineType=cv2.LINE_AA)  # Mark the coordinates
                    camera_xyz_list.append(camera_xyz)
                    
            # Coordinate: camera_xyz ; Width: w_real

            # Add fps display
            fps = int(1.0 / (t_end - t_start))
            cv2.putText(canvas, text="FPS: {}".format(fps), org=(50, 50),
                        fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, thickness=2,
                        lineType=cv2.LINE_AA, color=(0, 0, 0))
            cv2.namedWindow('detection', flags=cv2.WINDOW_NORMAL |
                            cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED)
            cv2.imshow('detection', canvas)
            key = cv2.waitKey(1)
            # Press esc or 'q' to close the image window
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break
    finally:
        # Stop streaming
        pipeline.stop()

