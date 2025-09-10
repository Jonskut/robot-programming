#!/usr/bin/env python3

from ignition.transport import Node
import numpy as np
import cv2

# Topics for RGB and depth
rgb_topic = "/world/shapes/model/realsense_d435/link/link/sensor/realsense_d435/image"
depth_topic = "/world/shapes/model/realsense_d435/link/link/sensor/realsense_d435/depth_image"

saved = {"rgb": False, "depth": False}

def rgb_callback(msg):
    if not saved["rgb"]:
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
        cv2.imwrite("rgb_image.png", img)
        print("Saved rgb_image.png")
        saved["rgb"] = True
        check_done()

def depth_callback(msg):
    if not saved["depth"]:
        # Depth data is float32, reshape accordingly
        depth = np.frombuffer(msg.data, dtype=np.float32).reshape((msg.height, msg.width))
        # Optionally normalize for visualization
        depth_vis = cv2.normalize(depth, None, 0, 65535, cv2.NORM_MINMAX)
        cv2.imwrite("depth_image.png", depth_vis.astype(np.uint16))
        print("Saved depth_image.png")
        saved["depth"] = True
        check_done()

def check_done():
    if saved["rgb"] and saved["depth"]:
        exit(0)

node = Node()
node.subscribe(rgb_topic, rgb_callback)
node.subscribe(depth_topic, depth_callback)

# Keep running until both images are saved
import time
while True:
    time.sleep(0.1)
