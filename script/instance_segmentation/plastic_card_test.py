import os
import sys
try:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
except:
    pass
import random
import math
import numpy as np
import skimage.io
import matplotlib
import matplotlib.pyplot as plt

import pyrealsense2 as rs
import cv2

# import Mask RCNN
from mrcnn.config import Config
from mrcnn import utils
import mrcnn.model as modellib
from mrcnn import visualize

ROOT_DIR = os.path.abspath("Mask_RCNN/")

# Import COCO config
sys.path.append(os.path.join(ROOT_DIR, "samples/coco/"))  # To find local version
import coco

sys.path.append(os.path.abspath("object_config/"))
import plastic_card
Stones_DIR = "dataset/plastic_card/"

# %matplotlib inline 
matplotlib.use('TkAgg')

# Directory to save logs and trained model
MODEL_DIR = os.path.join(ROOT_DIR, "logs")

# Local path to trained weights file
COCO_MODEL_PATH = os.path.join(ROOT_DIR, "mask_rcnn_coco.h5")
# Download COCO trained weights from Releases if needed
if not os.path.exists(COCO_MODEL_PATH):
    utils.download_trained_weights(COCO_MODEL_PATH)

MODEL_PATH = "model/mask_rcnn_multi_category_0200.h5"
# MODEL_PATH = "model/mask_rcnn_stones_0100.h5"


# Directory of images to run detection on
IMAGE_DIR = os.path.join(ROOT_DIR, "images")
TEST_IMAGE_DIR = "test_image/"


class InferenceConfig(plastic_card.StonesConfig):
    # Set batch size to 1 since we'll be running inference on
    # one image at a time. Batch size = GPU_COUNT * IMAGES_PER_GPU
    GPU_COUNT = 1
    IMAGES_PER_GPU = 1    

config = InferenceConfig()
# config.display()



# Create model object in inference mode.
model = modellib.MaskRCNN(mode="inference", model_dir=MODEL_DIR, config=config)

# Load weights trained on MS-COCO
model.load_weights(MODEL_PATH, by_name=True)

# dataset = plastic_card.StonesDataset()
# dataset.load_stones(Stones_DIR, "train")
# dataset.prepare()
# class_names = dataset.class_names
# print(class_names)
class_names = ['BG', 'domino', 'triangle', 'plastic_card', 'stone']



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

# Start streaming
pipeline.start(config)


try:
    while True:
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if color_frame is not None:
            # Convert images to numpy arrays
            color_image = np.asanyarray(color_frame.get_data())

            # Show images
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', color_image)
            key = cv2.waitKey(1)
            if key != -1:
                break



finally:
    # Stop streaming
    pipeline.stop()

results = model.detect([color_image], verbose=1)

# Visualize results
r = results[0]
visualize.display_instances(color_image, r['rois'], r['masks'], r['class_ids'], 
                            class_names, r['scores'])


# Load a random image from the images folder
# file_names = next(os.walk(IMAGE_DIR))[2]
# image = skimage.io.imread(os.path.join(IMAGE_DIR, random.choice(file_names)))
# image = skimage.io.imread(os.path.join(TEST_IMAGE_DIR, "0.jpeg"))

# Run detection
# results = model.detect([image], verbose=1)

# Visualize results
# r = results[0]
# visualize.display_instances(image, r['rois'], r['masks'], r['class_ids'], class_names, r['scores'])

