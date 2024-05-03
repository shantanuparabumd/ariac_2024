#!/usr/bin/env python3
# ROS library
import rclpy 

# Importing Camera_subscriber from advance_sensor.py
from rwa5_group3.advance_sensor import Camera_subscriber

# main function
def main():
    rclpy.init(args=None)
    # created node
    node = Camera_subscriber('camera_subscriber')
    try: 
        rclpy.spin(node)
    except KeyboardInterrupt:  
        node.destroy_node()
        rclpy.shutdown()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()