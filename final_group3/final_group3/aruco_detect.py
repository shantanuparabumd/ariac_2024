#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import cv2
from cv_bridge import CvBridge
import numpy as np

class ArucoDetector(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.bridge = CvBridge()
        self.image_subs = self.create_subscription(
            Image, '/ariac/sensors/kts2_camera/rgb_image', self.image_callback, 10
        )
        self.camera_info_subs = self.create_subscription(
            CameraInfo, '/ariac/sensors/kts2_camera/camera_info', self.camera_info_callback, 10
        )

        self.camera_matrix = None
        self.dist_coeffs = None
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_params.adaptiveThreshWinSizeMin = 3
        self.aruco_params.adaptiveThreshWinSizeMax = 23
        self.aruco_params.adaptiveThreshWinSizeStep = 10

        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        print("init done")

    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.k).reshape((3, 3))
        self.dist_coeffs = np.array(msg.d)
        # print("cam_mat:",self.camera_matrix, "dist:",self.dist_coeffs)

    def image_callback(self, msg):
        if self.camera_matrix is None or self.dist_coeffs is None:
            self.get_logger().warn("Camera info not received yet.")
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # gray =  gray[190:250,290:355]
        # gray = cv2.equalizeHist(gray)
        # gray = cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,11,2)
        # _, gray = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

        corners, ids, _ = self.detector.detectMarkers(gray)
        print(ids)

        if ids is not None:
            self.get_logger().info(f"Detected ids: {ids}")
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.001, self.camera_matrix, self.dist_coeffs)
            for i in range(len(ids)):
                cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.1)
                # cv2.drawDetectedMarkers(cv_image, corners)
        cv2.imshow('gray_image', gray)
        cv2.imshow('raw_image', cv_image)
        cv2.waitKey(1) 


