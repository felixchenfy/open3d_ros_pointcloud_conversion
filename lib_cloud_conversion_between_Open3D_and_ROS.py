#!/usr/bin/env python
# -*- coding: utf-8 -*-

import open3d
import numpy as np

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2

# The data structure of each point in ros PointCloud2: 16 bits = x + y + z + rgb
FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
]

# Bit operations
BIT_MOVE_16 = 2**16
BIT_MOVE_8 = 2**8
convert_rgbUint32_to_tuple = lambda rgb_uint32: (
    (rgb_uint32 & 0x00ff0000)>>16, (rgb_uint32 & 0x0000ff00)>>8, (rgb_uint32 & 0x000000ff)
)

# Convert the datatype of point cloud from Open3D to ROS PointCloud2 (XYZRGB only)
def convertCloudFromOpen3dToRos(open3d_cloud, frame_id="odom"):
    # -- Change rgb color from "three float" to "one 24-byte int"
    # 0x00FFFFFF is white, 0x00000000 is black.
    colors = np.floor(np.asarray(open3d_cloud.colors)*255) # nx3 matrix
    colors = colors[:,0] * BIT_MOVE_16 +colors[:,1] * BIT_MOVE_8 + colors[:,2] 
    
    # -- Combine (points, colors) to get ros_cloud
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id
    return pc2.create_cloud(header, FIELDS ,
            np.c_[np.asarray(open3d_cloud.points), colors]) #column stack

def convertCloudFromRosToOpen3d(ros_cloud):
    # -- Get xyz and rgb from ros_cloud
    x_y_z_rgb = list(pc2.read_points(ros_cloud, skip_nans=True, field_names = ("x", "y", "z","rgb")))
    xyz = [(x,y,z) for x,y,z,rgb in x_y_z_rgb ]
    rgb = [convert_rgbUint32_to_tuple(rgb) for x,y,z,rgb in x_y_z_rgb ]
    
    # -- Get open3d_cloud
    open3d_cloud = open3d.PointCloud()
    open3d_cloud.points = open3d.Vector3dVector(np.array(xyz))
    open3d_cloud.colors = open3d.Vector3dVector(np.array(rgb)/255.0)
    return open3d_cloud

# -- Example of usage
if __name__ == "__main__":
    rospy.init_node('test_pc_conversion_between_Open3D_and_ROS', anonymous=True)
    
    # -- Read point cloud from file
    import os
    PYTHON_FILE_PATH=os.path.join(os.path.dirname(__file__))+"/"
    filename=PYTHON_FILE_PATH+"test_cloud_a_bottle.pcd"

    open3d_cloud = open3d.read_point_cloud(filename)
    rospy.loginfo("Loading cloud from file by open3d.read_point_cloud: ")
    print(open3d_cloud)
    print("")

    # -- Set publisher
    topic_name="kinect2/qhd/points"
    pub = rospy.Publisher(topic_name, PointCloud2, queue_size=1)
    
    # -- Set subscriber
    global received_ros_cloud
    received_ros_cloud = None
    def callback(ros_cloud):
        global received_ros_cloud
        received_ros_cloud=ros_cloud
        rospy.loginfo("-- Received ROS PointCloud2 message.")
    rospy.Subscriber(topic_name, PointCloud2, callback)      
    
    # -- Convert open3d_cloud to ros_cloud, and publish. Until the subscribe receives it.
    while received_ros_cloud is None and not rospy.is_shutdown():
        rospy.loginfo("-- Not receiving ROS PointCloud2 message yet ...")

        if True: # Use the cloud converted from open3d
            rospy.loginfo("Converting cloud from Open3d to ROS PointCloud2 ...")
            ros_cloud = convertCloudFromOpen3dToRos(open3d_cloud)

        else: # Use a simple cloud with 3 points
            rospy.loginfo("Converting a 3-point cloud into ROS PointCloud2 ...")
            TEST_CLOUD_POINTS = [
                [1.0, 0.0, 0.0, 0xff0000],
                [0.0, 1.0, 0.0, 0x00ff00],
                [0.0, 0.0, 1.0, 0x0000ff],
            ]
            ros_cloud = pc2.create_cloud(
                Header(frame_id="odom"), FIELDS , TEST_CLOUD_POINTS)

        # publish cloud
        pub.publish(ros_cloud)
        rospy.loginfo("Conversion and publish success ...\n")
        rospy.sleep(1)
        
    # -- After subscribing the ros cloud, convert it back to open3d, and draw
    received_open3d_cloud = convertCloudFromRosToOpen3d(received_ros_cloud)
    print(received_open3d_cloud)
    # open3d.write_point_cloud(PYTHON_FILE_PATH+"tmp.pcd", received_open3d_cloud)
    open3d.draw_geometries([received_open3d_cloud])
    rospy.loginfo("-- Finish display. The program is terminating ...\n")