#!/usr/bin/env python3

from __future__ import division, print_function, absolute_import

import roslib
import rospy
import numpy as np
import cv2
import pickle 

# from qcar.q_essential import Camera2D
# from pal.utilities.vision import Camera2D
from pal.products.qcar import QCarCameras
from hal.utilities.image_processing import ImageProcessing

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError



#region : QCar CSI Node

class CSINode(object):
	def __init__(self):
		super().__init__()

		# Properties for image being streamed by all 4 cameras
		self.imageWidth = 640
		self.imageHeight = 480
		self.sampleRate = 30.0

		self.rightCamPublisher = rospy.Publisher('/qcar/csi_right',
												Image,
												queue_size=10)

		self.backCamPublisher = rospy.Publisher('/qcar/csi_back',
												Image,
												queue_size=10)

		self.leftCamPublisher = rospy.Publisher('/qcar/csi_left',
												Image,
												queue_size=10)

		self.frontCamPublisher = rospy.Publisher('/qcar/csi_front',
												Image,
												queue_size=10)

		self.bridge = CvBridge()
		self.qcarCameras = QCarCameras(frameWidth  = self.imageWidth,
                                    frameHeight = self.imageHeight,
                                    frameRate   = self.sampleRate,
                                    enableRight = True,
                                    enableBack  = True,
                                    enableLeft  = True,
                                    enableFront = True)

		cam_id = self.qcarCameras.csiFront.url[-1]
		path_to_front_cam_matrix = rospy.get_param('front_cam_matrix', 
				'/home/nvidia/auto-ws/src/autonomous-driving/src/qcar/src/camera_matrix_obj_3.pkl'
				)
		with open(path_to_front_cam_matrix, "rb") as f:
			cam_dict = pickle.load(f)
		
		self.qcarCameraMatrices = {
			cam_id: cam_dict
		}									
		self.im_proc = ImageProcessing()

		while not rospy.is_shutdown():
			self.qcarCameras.readAll()

			self.process_cam_data(self.qcarCameras.csiRight,
									self.rightCamPublisher)
			self.process_cam_data(self.qcarCameras.csiBack,
									self.backCamPublisher)
			self.process_cam_data(self.qcarCameras.csiLeft,
									self.leftCamPublisher)
			self.process_cam_data(self.qcarCameras.csiFront,
									self.frontCamPublisher)

	def process_cam_data(self,cameraNumber, cameraInfo):
		cam_id = cameraNumber.url[-1]
		
		# Extract the image from buffer
		streamImage = cameraNumber.imageData

		if cam_id in self.qcarCameraMatrices:
			streamImage = self.im_proc.undistort_img(
					streamImage, 
					self.qcarCameraMatrices["3"]["matrix"], 
					self.qcarCameraMatrices["3"]["coef"]
				)

		# COnfigure the image publisher
		imgPublisher = self.bridge.cv2_to_imgmsg(streamImage, "bgr8")
		imgPublisher.header.stamp =  rospy.Time.now()
		imgPublisher.header.frame_id = 'cam_img_input'
		cameraInfo.publish(imgPublisher)
#endregion

if __name__ == '__main__':
	rospy.init_node('csi_node')
	r = CSINode()

	rospy.spin()
		