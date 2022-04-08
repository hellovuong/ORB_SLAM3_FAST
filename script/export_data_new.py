#!/usr/bin/python

import argparse
import rosbag
import rospy
import cv2
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
import os

# only required for simulated scenes
try:
    import OpenEXR, Imath
    import numpy as np
except ImportError as e:
    print("To extract depth image of simulated scenes, please install OpenEXR and Imath")
    pass

# arguments
parser = argparse.ArgumentParser()
parser.add_argument("bag", help="ROS bag file to extract")
parser.add_argument("--event_topic", default="/dvs/events", help="Event topic")
parser.add_argument("--image_topic", default="/dvs/image_raw", help="Image topic")
parser.add_argument("--right_image_topic", default="/dvs/image_raw", help="Right Image topic")
parser.add_argument("--side_left", default="/dvs/image_raw", help="Left Side View Image topic")
parser.add_argument("--side_right", default="/dvs/image_raw", help="Right Side View Image topic")
parser.add_argument("--depthmap_topic", default="/dvs/depthmap", help="Depth map topic")
parser.add_argument("--imu_topic", default="/dvs/imu", help="IMU topic")
parser.add_argument("--odom_topic", default="/dvs/odom", help="Odom topic")
parser.add_argument("--calib_topic", default="/dvs/camera_info", help="Camera info topic")
parser.add_argument("--groundtruth_topic", default="/optitrack/davis", help="Ground truth topic")
parser.add_argument("--reset_time", help="Remove time offset", action="store_true")
args = parser.parse_args()

# Use a CvBridge to convert ROS images to OpenCV images so they can be saved.
bridge = CvBridge()

def get_filename(i):
    return "zed2_left/frame_" + str(i).zfill(8)  + ".png"
def get_filename_right(i):
    return "zed2_right/frame_" + str(i).zfill(8)  + ".png"   
def get_filename_side_left(i):
    return "side_left/frame_" + str(i).zfill(8)  + ".png"   
def get_filename_side_right(i):
    return "side_right/frame_" + str(i).zfill(8)  + ".png"   
def get_depth_filename(i):
    return "depthmaps/frame_" + str(i).zfill(8)  + ".exr"

def timestamp_str(ts):
    return str(ts.secs) + "." + str(ts.nsecs).zfill(9)

# Create folders and files
if not os.path.exists("zed2_left"):
    os.makedirs("zed2_left")
if not os.path.exists("zed2_right"):
    os.makedirs("zed2_right")
if not os.path.exists("side_left"):
    os.makedirs("side_left")
if not os.path.exists("side_right"):
    os.makedirs("side_right")
if not os.path.exists("depthmaps"):
    os.makedirs("depthmaps")

image_index = 0
right_image_index = 0
side_left_image_index = 0
side_right_image_index = 0
depthmap_index = 0
event_sum = 0
imu_msg_sum = 0
odom_msg_sum = 0
groundtruth_msg_sum = 0
calib_written = False

events_file = open('events.txt', 'w')
images_file = open('zed2_left.txt', 'w')
right_images_file = open('zed2_right.txt', 'w')
side_left_images_file = open('side_left.txt', 'w')
side_right_images_file = open('side_right.txt', 'w')
depthmaps_file = open('depthmaps.txt', 'w')
imu_file = open('zed2_imu.txt', 'w')
imu_file.write('#timestamp [s],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]'+'\n')
odom_file = open('odom.txt', 'w')
odom_file.write('#timestamp [s],vx [m s^-1],vy [m s^-1],vz [m s^-1],wx [rad s^-1],wy [rad s^-1],wz [rad s^-1], tx [m], ty [m], tz [m], qx [rad],qy [rad],qz [rad], qw[rad]'+'\n')
calib_file = open('calib.txt', 'w')
groundtruth_file = open('groundtruth.txt', 'w')

with rosbag.Bag(args.bag, 'r') as bag:
    # reset time?
    reset_time = rospy.Time()
    if args.reset_time:
        first_msg = True
        for topic, msg, t in bag.read_messages():
            got_stamp = False
            if topic == args.image_topic:
                stamp = msg.header.stamp
                got_stamp = True
            elif topic == args.event_topic:
                stamp = msg.events[0].ts
                got_stamp = True
            elif topic == args.depthmap_topic:
                stamp = msg.header.stamp
                got_stamp = True
            elif topic == args.imu_topic:
                stamp = msg.header.stamp
                got_stamp = True

            if got_stamp:
                if first_msg:
                    reset_time = stamp
                    first_msg = False
                else:
                    if stamp < reset_time:
                        reset_time = stamp
    print ("Reset time: " + timestamp_str(reset_time))

    for topic, msg, t in bag.read_messages():
        # Images
        if topic == args.image_topic:
            try:
                image_type = msg.encoding
                cv_image = bridge.imgmsg_to_cv2(msg, image_type)
            except CvBridgeError as e:
                print( e)

            images_file.write(timestamp_str(msg.header.stamp - reset_time) + " ")
            images_file.write(get_filename(image_index) + "\n")

            cv2.imwrite(get_filename(image_index), cv_image)

            image_index = image_index + 1
        elif topic == args.right_image_topic:
            try:
                image_type = msg.encoding
                cv_image = bridge.imgmsg_to_cv2(msg, image_type)
            except CvBridgeError as e:
                print( e)

            right_images_file.write(timestamp_str(msg.header.stamp - reset_time) + " ")
            right_images_file.write(get_filename_right(right_image_index) + "\n")

            cv2.imwrite(get_filename_right(right_image_index), cv_image)

            right_image_index = right_image_index + 1
        elif topic == args.side_left:
            try:
                image_type = msg.encoding
                cv_image = bridge.imgmsg_to_cv2(msg, image_type)
            except CvBridgeError as e:
                print( e)

            side_left_images_file.write(timestamp_str(msg.header.stamp - reset_time) + " ")
            side_left_images_file.write(get_filename_side_left(side_left_image_index) + "\n")

            cv2.imwrite(get_filename_side_left(side_left_image_index), cv_image)

            side_left_image_index = side_left_image_index + 1
        elif topic == args.side_right:
            try:
                image_type = msg.encoding
                cv_image = bridge.imgmsg_to_cv2(msg, image_type)
            except CvBridgeError as e:
                print( e)

            side_right_images_file.write(timestamp_str(msg.header.stamp - reset_time) + " ")
            side_right_images_file.write(get_filename_side_left(side_right_image_index) + "\n")

            cv2.imwrite(get_filename_side_right(side_right_image_index), cv_image)

            side_right_image_index = side_right_image_index + 1
        # events
        elif topic == args.event_topic:
            for e in msg.events:
                events_file.write(timestamp_str(e.ts - reset_time) + " ")
                events_file.write(str(e.x) + " ")
                events_file.write(str(e.y) + " ")
                events_file.write(("1" if e.polarity else "0") + "\n")
                event_sum = event_sum + 1

        # IMU
        elif topic == args.imu_topic:
            imu_file.write(timestamp_str(msg.header.stamp - reset_time) + " ")
            
            imu_file.write(str(msg.angular_velocity.x) + " ")
            imu_file.write(str(msg.angular_velocity.y) + " ")
            imu_file.write(str(msg.angular_velocity.z) + " ")
            imu_file.write(str(msg.linear_acceleration.x) + " ")
            imu_file.write(str(msg.linear_acceleration.y) + " ")
            imu_file.write(str(msg.linear_acceleration.z) + "\n")
            imu_msg_sum = imu_msg_sum + 1
        # odom
        elif topic == args.odom_topic:
            odom_file.write(timestamp_str(msg.header.stamp - reset_time) + " ")
            odom_file.write(str(msg.pose.pose.position.x) + " ")
            odom_file.write(str(msg.pose.pose.position.y) + " ")
            odom_file.write(str(msg.pose.pose.position.z) + " ")
            odom_file.write(str(msg.pose.pose.orientation.x) + " ")
            odom_file.write(str(msg.pose.pose.orientation.y) + " ")
            odom_file.write(str(msg.pose.pose.orientation.z) + " ")
            odom_file.write(str(msg.pose.pose.orientation.w) + " ")
            odom_file.write(str(msg.twist.twist.linear.x) + " ")
            odom_file.write(str(msg.twist.twist.linear.y) + " ")
            odom_file.write(str(msg.twist.twist.linear.z) + " ")
            odom_file.write(str(msg.twist.twist.angular.x) + " ")
            odom_file.write(str(msg.twist.twist.angular.y) + " ")
            odom_file.write(str(msg.twist.twist.angular.z) + "\n")
            odom_msg_sum = odom_msg_sum + 1

        # calibration
        elif topic == args.calib_topic:
            if not calib_written:
                calib_file.write(str(msg.K[0]) + " ")
                calib_file.write(str(msg.K[4]) + " ")
                calib_file.write(str(msg.K[2]) + " ")
                calib_file.write(str(msg.K[5]) + " ")
                calib_file.write(str(msg.D[0]) + " ")
                calib_file.write(str(msg.D[1]) + " ")
                calib_file.write(str(msg.D[2]) + " ")
                calib_file.write(str(msg.D[3]))
                if len(msg.D) > 4:
                    calib_file.write(" " + str(msg.D[4]))
                calib_file.write("\n")
                calib_written = True

        # ground truth
        elif topic == args.groundtruth_topic:
            # case for Optitrack (geometry_msgs/PoseStamped)
            if hasattr(msg, 'pose'):
                pose = msg
            # case for linear slider (std_msgs/Float64)
            elif hasattr(msg, 'data'):
                pose = PoseStamped()
                pose.header.stamp = t
                pose.pose.position.x = msg.data
                pose.pose.orientation.w = 1.

            groundtruth_file.write(timestamp_str(pose.header.stamp - reset_time) + " ")
            groundtruth_file.write(str(pose.pose.position.x) + " ")
            groundtruth_file.write(str(pose.pose.position.y) + " ")
            groundtruth_file.write(str(pose.pose.position.z) + " ")
            groundtruth_file.write(str(pose.pose.orientation.x) + " ")
            groundtruth_file.write(str(pose.pose.orientation.y) + " ")
            groundtruth_file.write(str(pose.pose.orientation.z) + " ")
            groundtruth_file.write(str(pose.pose.orientation.w) + "\n")

            groundtruth_msg_sum = groundtruth_msg_sum + 1
            
        # Depth maps
        elif topic == args.depthmap_topic:
            try:
                cv_depth = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough').astype(np.float32)
            except CvBridgeError as e:
                print(e)

            depthmaps_file.write(timestamp_str(msg.header.stamp - reset_time) + " ")
            depthmaps_file.write(get_depth_filename(depthmap_index) + "\n")

            # Write depth map to EXR file
            exr_header = OpenEXR.Header(cv_depth.shape[1], cv_depth.shape[0])
            exr_header['channels'] = {'Z': Imath.Channel(Imath.PixelType(Imath.PixelType.FLOAT))}
            exr = OpenEXR.OutputFile(get_depth_filename(depthmap_index), exr_header)
            exr.writePixels({'Z' : cv_depth.tostring()})
            exr.close()

            depthmap_index = depthmap_index + 1

# statistics (remove missing groundtruth or IMU file if not available)
print ("All data extracted!")
print ("Events:       " + str(event_sum))
print ("Images:       " + str(image_index))
print ("Right Images: " + str(right_image_index))
print ("Odom: " + str(odom_msg_sum))
print ("Depth maps:   " + str(depthmap_index))
print ("IMU:          " + str(imu_msg_sum))
print ("Ground truth: " + str(groundtruth_msg_sum))

# close all files
events_file.close()
images_file.close()
right_images_file.close()
depthmaps_file.close()
imu_file.close()
groundtruth_file.close()

# clean up
if event_sum == 0:
    os.remove("events.txt")
if calib_written == False:
    os.remove("calib.txt")
if imu_msg_sum == 0:
    os.remove("zed2_imu.txt")
    print ("Removed IMU file since there were no messages.")

if odom_msg_sum == 0:
    os.remove("odom.txt")
    print ("Removed odom file since there were no messages.")

if groundtruth_msg_sum == 0:
    os.remove("groundtruth.txt")
    print ("Removed ground truth file since there were no messages.")

if image_index == 0:
    os.remove("zed2_left.txt")
    os.removedirs("zed2_left")
    print ("Removed left image file since there were no messages.")

if right_image_index == 0:
    os.remove("zed2_right.txt")
    os.removedirs("zed2_right")
    print ("Removed right image file since there were no messages.")
    
if depthmap_index == 0:
    os.remove("depthmaps.txt")
    os.removedirs("depthmaps")
    print ("Removed depthmaps file since there were no messages.")