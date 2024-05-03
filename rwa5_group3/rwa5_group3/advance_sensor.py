#!/usr/bin/env python3

# Ultralytics library for YOLO
from ultralytics import YOLO

# ROS libraries and messages
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import  Pose  #<-- Pose message type 

# OpenCV libraries for image processing
from cv_bridge import CvBridge
import cv2

# Custom functions and messages
from rwa5_group3.pre_processing import canny_edge_detection, get_color
from ariac_msgs.msg import Part,PartPose,AdvancedLogicalCameraImage, BasicLogicalCameraImage

# OS module 
from ament_index_python.packages import get_package_share_directory
import os

# Frame transformation library
import PyKDL



# Map the x and y coordinates to the bins slots using pixel coordinates
x_map = {
    1: (90,160),
    2: (160, 220),
    3: (220,280),
    4:(335,400),
    5:(400,460),
    6:(460,520)
}
y_map = {
    1: (30,90),
    2: (90,150),
    3: (150,210),
    4: (270,330),
    5: (330,390),
    6: (390,450)
}


# Color to Part color mapping for ARIAC Messages
color_dict = {
    "red": Part.RED,
    "blue": Part.BLUE,
    "green": Part.GREEN,
    "purple":Part.PURPLE,
    "orange":Part.ORANGE
}
        
# Part to Part type mapping for ARIAC Messages
type_dict = {
    "Pump": Part.PUMP,
    "Regulator": Part.REGULATOR,
    "Battery": Part.BATTERY,
    "Sensor": Part.SENSOR
}
        

class Camera_subscriber(Node):

    def __init__(self, node_name):
        super().__init__(node_name)
            
        # Path to the trained model
        model_path = os.path.join(get_package_share_directory('rwa5_group3'), 'scripts', 'best.pt')
        
        # Load the YOLO model
        self.model = YOLO(model_path)

        # Create subscription to the left and right bins camera
        self.subscription_left = self.create_subscription(
            Image,
            '/ariac/sensors/left_bins_camera/rgb_image',
            self.camera_callback_left,
            10)
        
        self.subscription_right = self.create_subscription(
            Image,
            '/ariac/sensors/right_bins_camera/rgb_image',
            self.camera_callback_right,
            10)
        
        # Bridge for converting ROS Image messages to OpenCV images
        self.bridge = CvBridge()
        
        # Create publishers for the Yolo results with bounding boxes
        self.right_bins_camera_image_pub_ = self.create_publisher(Image, "yolo/right_bins_camera/inference_result", 1)
        self.left_bins_camera_image_pub_ = self.create_publisher(Image, "yolo/left_bins_camera/inference_result", 1)
        
        # Subscrition to the logical camera for the part poses
        self.left_pose_sub_ = self.create_subscription(BasicLogicalCameraImage, "/ariac/sensors/left_bins_camera_logical/image", self.left_pose_callback, qos_profile_sensor_data)
        self.right_pose_sub_ = self.create_subscription(BasicLogicalCameraImage, "/ariac/sensors/right_bins_camera_logical/image", self.right_pose_callback, qos_profile_sensor_data)
        
        # Publsiher for the parts and poses to sensors manager
        self.part_pose_left_pub_ = self.create_publisher(AdvancedLogicalCameraImage, "/rwa_group3/sensors/left_bins_camera_yolo", 1)
        self.part_pose_right_pub_ = self.create_publisher(AdvancedLogicalCameraImage, "/rwa_group3/sensors/right_bins_camera_yolo", 1)
        
        # Flags for checking if the camera and pose data is read
        self.left_pose_read_ = False
        self.left_camera_read_ = False
        self.right_pose_read_ = False
        self.right_camera_read_ = False
        
        
        # Flags to check the functionality
        self.parts_published = False
        
        self.timer_ = self.create_timer(1, self.publish_parts)
        
        self.pose_dict_left = {}
        self.parts_dict_left = {}
        
        self.pose_dict_right = {}  
        self.parts_dict_right = {}
        
        print("YOLO Initialized")
        
    def camera_callback_left(self, data):
        # Check to only capture one frame
        if self.left_camera_read_:
            return
        
        # Reading image and converting compartiable with CV library
        if data is not None:
            image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.left_camera_read_ = True
        else:
            # Handle empty data here
            return
        
        # Edge detection to pass to model
        img = canny_edge_detection(image)
        results = self.model(img)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        
        # print("Left bins camera callback")
        
        # bbox = None
        # processing the coordinates from yolo model to get bounding boxes
        for r in results:
            boxes = r.boxes
            for box in boxes:
                b = box.xyxy[0].to('cpu').detach().numpy().copy()  # get box coordinates in (top, left, bottom, right) format
                c = box.cls
                color = get_color(image, int(b[0]), int(b[1]), int(b[2]), int(b[3]),self.model.names[int(c)])
                cv2.rectangle(image, (int(b[0]), int(b[1])), (int(b[2]), int(b[3])), (0,0,0), 2)  # Draw bounding box on image
                
                x, y = int((b[0] + b[2]) / 2), int((b[1] + b[3]) / 2)
                for x_key, x_value in x_map.items():
                    if x_value[0] <= x <= x_value[1]:
                        new_x = x_key
                        break
                for y_key, y_value in y_map.items():
                    if y_value[0] <= y <= y_value[1]:
                        new_y = y_key
                        break
                new_key = (new_x, new_y)
                self.parts_dict_left[new_key] = (self.model.names[int(c)], color)
                            
        img_msg = self.bridge.cv2_to_imgmsg(image, encoding="rgb8")  
        # Sorting the cooridinates rowwise to match ids with basic logical camera message poses
        self.parts_dict_left = dict(sorted(self.parts_dict_left.items(), key=lambda item: (item[0][1], item[0][0])))

        # Publish image
        self.left_bins_camera_image_pub_.publish(img_msg)
        

    def camera_callback_right(self, data):
        
        # Check to only capture one frame
        if self.right_camera_read_:
            return
        
        # Reading image and converting compartiable with CV library
        if data is not None:
            image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.right_camera_read_ = True
        else:
            # Handle empty data here
            return
        
        # Edge detection to pass to model
        img = canny_edge_detection(image)
        results = self.model(img)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # print("Right bins camera callback")

        # processing the coordinates from yolo model to get bounding boxes
        for r in results:
            boxes = r.boxes
            for box in boxes:
                b = box.xyxy[0].to('cpu').detach().numpy().copy()  # get box coordinates in (top, left, bottom, right) format
                c = box.cls
                color = get_color(image, int(b[0]), int(b[1]), int(b[2]), int(b[3]),self.model.names[int(c)])
                cv2.rectangle(image, (int(b[0]), int(b[1])), (int(b[2]), int(b[3])), (0,0,0), 2)  # Draw bounding box on image
                
                x, y = int((b[0] + b[2]) / 2), int((b[1] + b[3]) / 2)
                for x_key, x_value in x_map.items():
                    if x_value[0] <= x <= x_value[1]:
                        new_x = x_key
                        break
                for y_key, y_value in y_map.items():
                    if y_value[0] <= y <= y_value[1]:
                        new_y = y_key
                        break
                new_key = (new_x, new_y)
                self.parts_dict_right[new_key] = (self.model.names[int(c)], color)
                            
        img_msg = self.bridge.cv2_to_imgmsg(image, encoding="rgb8")  
        
        # Sorting the cooridinates rowwise to match ids with basic logical camera message poses
        self.parts_dict_right = dict(sorted(self.parts_dict_right.items(), key=lambda item: (item[0][1], item[0][0])))

        img_msg = self.bridge.cv2_to_imgmsg(image, encoding="rgb8")  

        # Publish image
        self.right_bins_camera_image_pub_.publish(img_msg)
        

   
   
    def left_pose_callback(self, msg):
        # Check if the left pose has already been read
        if self.left_pose_read_:
            pass
        else:
            # Mark left pose as read
            self.left_pose_read_ = True

            # Retrieve the camera pose from the message
            camera_pose = msg.sensor_pose

            # Loop through each part pose in the message
            for part in msg.part_poses:
                # Compute the world pose of the part using the camera pose
                part_pose = self.compute_part_pose_in_world(part, camera_pose)
                # Store the part pose in a dictionary, using rounded position coordinates as keys
                self.pose_dict_left[(round(part_pose.position.x,2), round(part_pose.position.y,2))] = part_pose

            # Sort the dictionary of part poses by the x and y coordinates of the keys
            self.pose_dict_left = dict(sorted(self.pose_dict_left.items(), key=lambda item: (item[0][0], item[0][1])))

                
    def right_pose_callback(self, msg):
        # Check if the right pose has already been read
        if self.right_pose_read_:
            pass
        else:
            # Mark right pose as read
            self.right_pose_read_ = True

            # Retrieve the camera pose from the message
            camera_pose = msg.sensor_pose

            # Loop through each part pose in the message
            for part in msg.part_poses:
                # Compute the world pose of the part using the camera pose
                part_pose = self.compute_part_pose_in_world(part, camera_pose)
                # Store the part pose in a dictionary, using rounded position coordinates as keys
                self.pose_dict_right[(round(part_pose.position.x,2), round(part_pose.position.y,2))] = part_pose

            # Sort the dictionary of part poses by the x and y coordinates of the keys
            self.pose_dict_right = dict(sorted(self.pose_dict_right.items(), key=lambda item: (item[0][0], item[0][1])))
           
        
    def publish_parts(self):
        # Check if the left camera pose and left part information has been read and stored
        if self.left_pose_read_ and self.left_camera_read_:
            
            part_pose_array = []

            # Loop over items in both the pose dictionary and parts dictionary simultaneously
            for (pose_key, pose_value), (part_key, part_value) in zip(self.pose_dict_left.items(), self.parts_dict_left.items()):
                # New part msg using Part message type
                part = Part()

                # Assign type and color to the part from dictionaries using the values retrieved from the parts dictionary
                part.type = type_dict[part_value[0]]
                part.color = color_dict[part_value[1]]

                # New pose msg using Pose message type
                pose = Pose()
                pose.position.x = pose_value.position.x
                pose.position.y = pose_value.position.y
                pose.position.z = pose_value.position.z
                pose.orientation.x = pose_value.orientation.x
                pose.orientation.y = pose_value.orientation.y
                pose.orientation.z = pose_value.orientation.z
                pose.orientation.w = pose_value.orientation.w

                # New part pose msg using PartPose message type
                part_pose = PartPose()
                part_pose.pose = pose
                part_pose.part = part
                part_pose_array.append(part_pose)

            # Create an instance of AdvancedLogicalCameraImage to store the poses
            adv_msg = AdvancedLogicalCameraImage()
            adv_msg.part_poses = part_pose_array

            # Publish the message with part poses using the left part publisher
            self.part_pose_left_pub_.publish(adv_msg)
        
        #Similar to above if condition
        if self.right_pose_read_ and self.right_camera_read_ :
            part_pose_array = []
            for (pose_key, pose_value), (part_key, part_value) in zip(self.pose_dict_right.items(), self.parts_dict_right.items()):
                # print(f"{part_value[1]} {part_value[0]} at {pose_key}")
                part = Part()
                part.type = type_dict[part_value[0]]
                part.color = color_dict[part_value[1]]
                pose = Pose()
                pose.position.x = pose_value.position.x
                pose.position.y = pose_value.position.y
                pose.position.z = pose_value.position.z
                pose.orientation.x = pose_value.orientation.x
                pose.orientation.y = pose_value.orientation.y
                pose.orientation.z = pose_value.orientation.z
                pose.orientation.w = pose_value.orientation.w
                part_pose = PartPose()
                part_pose.pose = pose
                part_pose.part = part
                part_pose_array.append(part_pose)
            
            adv_msg = AdvancedLogicalCameraImage()
            adv_msg.part_poses = part_pose_array
            self.part_pose_right_pub_.publish(adv_msg)
        
            
            
            
            
    def compute_part_pose_in_world(
        self, part_pose_in_camera_frame, camera_pose_in_world_frame
    ):
        # First frame
        camera_orientation = camera_pose_in_world_frame.orientation
        camera_x = camera_pose_in_world_frame.position.x
        camera_y = camera_pose_in_world_frame.position.y
        camera_z = camera_pose_in_world_frame.position.z

        frame_camera_world = PyKDL.Frame(
            PyKDL.Rotation.Quaternion(
                camera_orientation.x,
                camera_orientation.y,
                camera_orientation.z,
                camera_orientation.w,
            ),
            PyKDL.Vector(camera_x, camera_y, camera_z),
        )

        # Second frame
        part_orientation = part_pose_in_camera_frame.orientation
        part_x = part_pose_in_camera_frame.position.x
        part_y = part_pose_in_camera_frame.position.y
        part_z = part_pose_in_camera_frame.position.z

        frame_part_camera = PyKDL.Frame(
            PyKDL.Rotation.Quaternion(
                part_orientation.x,
                part_orientation.y,
                part_orientation.z,
                part_orientation.w,
            ),
            PyKDL.Vector(part_x, part_y, part_z),
        )

        # Multiply the two frames
        frame_part_world = frame_camera_world * frame_part_camera

        # return the resulting pose from frame3
        pose = Pose()
        pose.position.x = frame_part_world.p.x()
        pose.position.y = frame_part_world.p.y()
        pose.position.z = frame_part_world.p.z()

        q = frame_part_world.M.GetQuaternion()
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]


        return pose
