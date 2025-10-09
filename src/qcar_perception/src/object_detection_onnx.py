#!/usr/bin/env python3

import os

os.environ['KMP_DUPLICATE_LIB_OK'] = 'True'
os.environ['CUDA_VISIBLE_DEVICES'] = '0'


import argparse
import os
import sys
from pathlib import Path

import cv2
# Python imports
import numpy as np
# ROS imports
import rospy
import scipy.io as sio
import std_msgs.msg
# Deep Learning imports
import torch
import torch.backends.cudnn as cudnn
import yaml
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point32, Polygon
from numpy import random
from rospkg import RosPack
from sensor_msgs.msg import Image
from skimage.transform import resize
from std_msgs.msg import UInt8
from torch.autograd import Variable
from qcar_perception.msg import BoundingBox, BoundingBoxes

from models.experimental import attempt_load
from utils.datasets import LoadImages, LoadStreams
from utils.general import (apply_classifier, check_img_size,
                           check_requirements, increment_path,
                           non_max_suppression, scale_coords, set_logging,
                           strip_optimizer)
# util + model imports
from utils.torch_utils import select_device

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # YOLOv5 root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

from models.common import DetectMultiBackend
from utils.datasets import IMG_FORMATS, VID_FORMATS, LoadImages, LoadStreams
from utils.general import (LOGGER, check_file, check_img_size, check_imshow,
                           check_requirements, colorstr, increment_path,
                           non_max_suppression, print_args, scale_coords,
                           strip_optimizer, xyxy2xywh)
from utils.plots import Annotator, colors, save_one_box
from utils.torch_utils import select_device, time_sync

import onnxruntime as rt
import onnx
import torch.nn.functional as F


package = RosPack()
package_path = package.get_path('qcar_perception')


class Detector:
    def __init__(self):
        # Load weights parameter
        self.weights_path = rospy.get_param('~weights_path')
        rospy.loginfo("Found weights, loading %s", self.weights_path)

        # Raise error if it cannot find the model
        if not os.path.isfile(self.weights_path):
            raise IOError(('{:s} not found.').format(self.weights_path))

        # Load image parameter and confidence threshold
        self.image_topic = rospy.get_param(
            '~image_topic', '/camera/rgb/image_raw')

        log_str = "Subscribing to image topic: %s" % self.image_topic
        rospy.loginfo(log_str)

        self.conf_thres = float(rospy.get_param('~confidence', 0.8))

        # Load other parameters
        self.device_name = '0'
        self.device = select_device(self.device_name)
        self.gpu_id = rospy.get_param('~gpu_id', 0)
        self.network_img_size = rospy.get_param('~img_size', 640)
        self.publish_image = rospy.get_param('~publish_image')
        self.iou_thres = 0.8
        self.augment = True

        self.classes = None
        self.agnostic_nms = False

        self.w = 0
        self.h = 0

        print("self.network_img_size", self.network_img_size)

        # Second-stage classifier
        self.classify = False

        # Initialize
        self.half = self.device.type != 'cpu'  # half precision only supported on CUDA
        
        ## ONNX model
        self.sess = rt.InferenceSession(self.weights_path, providers=[
            # "TensorrtExecutionProvider", 
            "CUDAExecutionProvider"
            ])
        self.input_name = self.sess.get_inputs()[0].name
        self.label_name = self.sess.get_outputs()[0].name

        self.model = onnx.load(self.weights_path)

        for metadata in self.model.metadata_props:
            if metadata.key == 'cfg':
                self.names = eval(metadata.value)
                self.names = list(self.names.values())
            elif metadata.key == 'stride':
                self.stride = eval(metadata.value)
        
        self.network_img_size = check_img_size(
            self.network_img_size, s=self.stride)  # check img_size

        self.scale = 1

        # if self.half:
        #     self.model.half()  # to FP16

        # Get names and colors
        self.colors = [[random.randint(0, 255)
                        for _ in range(3)] for _ in self.names]

        # # Run inference
        # if self.device.type != 'cpu':
        #     # self.data_type = torch.FloatTensor
        #     self.model(torch.zeros(1, 3, self.network_img_size, self.network_img_size).to(
        #         self.device).type(dtype=torch.float))  # run once

        # Load CvBridge
        self.bridge = CvBridge()

        # Load publisher topic
        self.detected_objects_topic = rospy.get_param(
            '~detected_objects_topic')
        self.published_image_topic = rospy.get_param('~detections_image_topic')

        # Define subscribers
        self.image_sub = rospy.Subscriber(
            self.image_topic, Image, self.image_cb, queue_size=1, buff_size=2**24)

        # Define publishers
        self.pub_ = rospy.Publisher(
            self.detected_objects_topic, BoundingBoxes, queue_size=10)
        self.pub_viz_ = rospy.Publisher(
            self.published_image_topic, Image, queue_size=10)
        rospy.loginfo("Launched node for object detection")

        # Spin
        rospy.spin()

    def image_cb(self, data):
        # Convert the image to OpenCV
        try:
            self.cv_img = self.bridge.imgmsg_to_cv2(data, "rgb8")
        except CvBridgeError as e:
            print(e)

        # Initialize detection results
        detection_results = BoundingBoxes()
        detection_results.header = data.header
        detection_results.image_header = data.header
        input_img = self.preprocess(self.cv_img)
        if self.device.type == 'cpu':
            input_img = Variable(input_img.type(torch.FloatTensor))

        # Get detections from network
        # with torch.no_grad():
        #     detections = self.model(input_img)[0]
        #     detections = detections.permute(0, 2, 1)
        #     print("detections", detections.shape)
        #     detections = non_max_suppression(detections, self.conf_thres, self.iou_thres,
        #                                      classes=self.classes, agnostic=self.agnostic_nms)
    
        detections = self.sess.run([self.label_name], {self.input_name: input_img.cpu().numpy()})[0]
        pred_onx = np.transpose(detections, (0, 2, 1))
        
        
        boxes = []
        scores = []
        class_ids = []

        # Iterate through output to collect bounding boxes, confidence scores, and class IDs
        for i in range(pred_onx.shape[1]):
            classes_scores = pred_onx[0][i][4:]
            (minScore, maxScore, minClassLoc, (x, maxClassIndex)) = cv2.minMaxLoc(classes_scores)
            if maxScore >= 0.25:
                box = [
                    pred_onx[0][i][0] - (0.5 * pred_onx[0][i][2]),  # x center - width/2 = left x
                    pred_onx[0][i][1] - (0.5 * pred_onx[0][i][3]),  # y center - height/2 = top y
                    pred_onx[0][i][2],  # width
                    pred_onx[0][i][3],  # height
                ]
                boxes.append(box)
                scores.append(maxScore)
                class_ids.append(maxClassIndex)

        # Apply NMS (Non-maximum suppression)
        result_boxes = cv2.dnn.NMSBoxes(boxes, scores, self.conf_thres, 0.45, 0.5)

        detections = []

        # Iterate through NMS results to draw bounding boxes and labels
        for i in range(len(result_boxes)):
            index = result_boxes[i]
            box = boxes[index]

            pad_x = max(self.h - self.w, 0) // 2
            pad_y = max(self.w - self.h, 0) // 2

            xmin = round(box[0] * self.scale - pad_x)
            ymin = round(box[1] * self.scale - pad_y)
            xmax = round((box[0] + box[2]) * self.scale- pad_x)
            ymax = round((box[1] + box[3]) * self.scale- pad_y)

            detection = {
                "class_id": class_ids[index],
                "class_name": self.names[class_ids[index]],
                "confidence": scores[index],
                "box": box,
            }
            detections.append(detection)

            # Populate darknet message
            detection_msg = BoundingBox()
            detection_msg.xmin = int(xmin)
            detection_msg.xmax = int(xmax)
            detection_msg.ymin = int(ymin)
            detection_msg.ymax = int(ymax)
            detection_msg.probability = float(scores[index])
            detection_msg.Class = self.names[class_ids[index]]

            # Append in overall detection message
            detection_results.bounding_boxes.append(detection_msg)
        
        print("detections:", detections)
        # Publish detection results
        self.pub_.publish(detection_results)

        # Visualize detection results
        if (self.publish_image):
            self.visualize_and_publish(detection_results, self.cv_img)
        return True

    def preprocess(self, img):
        # Extract image and shape
        img = np.copy(img)
        img = img.astype(float)
        height, width, channels = img.shape
        if (height != self.h) or (width != self.w):
            self.h = height
            self.w = width
            # Determine image to be used
            self.padded_image = np.zeros(
                (max(self.h, self.w), max(self.h, self.w), channels)).astype(float)

            self.scale = max(self.h, self.w)/self.network_img_size

        # Add padding
        if (self.w > self.h):
            self.padded_image[(self.w-self.h)//2: self.h +
                              (self.w-self.h)//2, :, :] = img
        else:
            self.padded_image[:, (self.h-self.w)//2: self.w +
                              (self.h-self.w)//2, :] = img

        input_img = torch.from_numpy(self.padded_image).float().to(self.device)
        input_img = input_img.permute(2, 0, 1)
        input_img = input_img.expand([1, *input_img.shape])
        # print("Shape of input_img", input_img.shape)

        input_img = F.interpolate(input_img, size=(self.network_img_size, self.network_img_size), mode='bilinear', align_corners=False) / 255.

        input_img = input_img[0][None]

        return input_img

    def visualize_and_publish(self, output, imgIn):
        # Copy image and visualize
        imgOut = imgIn.copy()
        font = cv2.FONT_HERSHEY_SIMPLEX
        for index in range(len(output.bounding_boxes)):
            label = output.bounding_boxes[index].Class
            x_p1 = output.bounding_boxes[index].xmin
            y_p1 = output.bounding_boxes[index].ymin
            x_p3 = output.bounding_boxes[index].xmax
            y_p3 = output.bounding_boxes[index].ymax
            confidence = output.bounding_boxes[index].probability

            # Set class color
            color = self.colors[self.names.index(label)]

            # Create rectangle
            cv2.rectangle(imgOut, (int(x_p1), int(y_p1)), (int(x_p3), int(
                y_p3)), (color[0], color[1], color[2]), 2)
            text = ('{:s}: {:.3f}').format(label, confidence)
            cv2.putText(imgOut, text, (int(x_p1), int(y_p1 - 15)), font,
                        0.5, (255, 255, 255), 2)

        # Publish visualization image
        image_msg = self.bridge.cv2_to_imgmsg(imgOut, "rgb8")
        image_msg.header.frame_id = 'camera'
        image_msg.header.stamp = rospy.Time.now()
        self.pub_viz_.publish(image_msg)


if __name__ == '__main__':
    rospy.init_node('detector')

    # Define detector object
    dm = Detector()
