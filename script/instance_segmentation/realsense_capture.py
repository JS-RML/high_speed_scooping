import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages') #prevent using cv2 in python2.7

import os
import pyrealsense2 as rs
import numpy as np
import cv2
import skimage.io
import re

SAVE_DIR = "capture_image/"
if not os.path.exists(SAVE_DIR):
    os.makedirs(SAVE_DIR)

def extract_jpg_file_name_number(f):
    s = re.findall("(\d+).jpg",f)
    return (int(s[0]) if s else -1,f)

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

def capture():
    # Start streaming
    pipeline.start(config)

    try:
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        # Convert images to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())

        f = os.listdir(SAVE_DIR) 
        if not f: 
            # if no file found in target directory
            save_idx = 1
        else:
            # find max index in directary and plus 1
            save_idx = int(max(f,key=extract_jpg_file_name_number).replace('.jpg','')) + 1 

        skimage.io.imsave(os.path.join(SAVE_DIR, str(save_idx) + ".jpg"), color_image[..., ::-1])

        # # Show images
        # cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        # cv2.imshow('RealSense', color_image)
        # cv2.waitKey(0)

    finally:
        # Stop streaming
        pipeline.stop()

if __name__ == "__main__":
    capture()
