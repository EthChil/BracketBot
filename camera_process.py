import time
import matplotlib.pyplot as plt
import numpy as np
import math
import asyncio
import cv2

import sys
import os.path
import os

plt.switch_backend('Agg')
plot_dir = './plots/'

def update_position(x, y, theta, d_left, d_right, W):
        distance_left = d_left
        distance_right = d_right

        distance_avg = (distance_left + distance_right) / 2
        delta_theta = (distance_right - distance_left) / W

        x += distance_avg * math.cos(theta + delta_theta / 2)
        y += distance_avg * math.sin(theta + delta_theta / 2)
        theta += delta_theta

        return x, y, theta
    
prefix = "orbslam_independant_files/"

def gstreamer_pipeline(sensor_id=0, capture_width=1280, capture_height=720, display_width=1280, display_height=720, framerate=10, flip_method=0):
    return (
        "nvarguscamerasrc sensor-id=%d !"
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )

def run_camera(termination_event):
    video_capture = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
    if not video_capture.isOpened():
        print("Error: Unable to open camera")
        return
    
    path_to_del = os.path.join(os.getcwd(), "orbslam_independant_files/vid")
    for item in os.listdir(path_to_del):
        item_path = os.path.join(path_to_del, item)
        
        if os.path.isfile(item_path):
            os.remove(item_path)
    
    while not termination_event.is_set():
        time.sleep(0.5)     

        
        timestamp = time.time()
        ret_val, frame = video_capture.read()
        if not ret_val:
            continue
        
        cv2.imwrite(prefix+"/vid/"+"slam_image_" + str(timestamp)+".png", frame)