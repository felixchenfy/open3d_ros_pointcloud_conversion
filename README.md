
## Content: Two functions for Open3D and ROS point cloud conversion
## Supported cloud type: XYZ and XYZRGB

I didn't see any good **Python** function for **converting point cloud datatype between Open3D and ROS**, so I made this repo.

The script [lib_cloud_conversion_between_Open3D_and_ROS.py](lib_cloud_conversion_between_Open3D_and_ROS.py)
contains 2 functions:   
* convertCloudFromOpen3dToRos  
* convertCloudFromRosToOpen3d  

where the ROS cloud format is indicating this: "[sensor_msgs/PointCloud2.msg](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointCloud2.html)".

The script also contains a test case, which does such a thing:  
(1) Read a open3d_cloud from .pcd file by Open3D.
(2) Convert it to ros_cloud.
(3) Publish ros_cloud to topic.
(4) Subscribe the ros_cloud from the same topic.
(5) Convert ros_cloud back to open3d_cloud.
(6) Display it and save it to .pcd file.  

You can test it by:  
> $ rosrun open3d_ros_pointcloud_conversion lib_cloud_conversion_between_Open3D_and_ROS.py

## References
* A very useful but bad documented github repo:  
    https://github.com/karaage0703/open3d_ros  
    It (only) contains codes for converting XYZ point cloud from open3d to ros. I copied pieces of codes from it.  
    However, when converting cloud from ros to open3d, it writes the cloud to file and then use open3d to read file, which is slower.  
    In my view, its scripts and function/variable namings are not well organized. So I decided to rewrite it. 

* PointCloud2 message type:  
    http://docs.ros.org/melodic/api/sensor_msgs/html/msg/PointCloud2.html

* Function of sensor_msgs.point_cloud2.create_cloud()  
    http://docs.ros.org/jade/api/sensor_msgs/html/namespacesensor__msgs_1_1point__cloud2.html#ad456fcf9391ad2ed2279df69572ca71d

* Open3d: from numpy to open3d pointcloud  
http://www.open3d.org/docs/tutorial/Basic/working_with_numpy.html#from-numpy-to-open3d-pointcloud

## Some explanation and testing of sensor_msgs::PointCloud2's contents
1. To test the format of PointCloud2 before writing these python conversion functions, I used the following c++ pcl functions to generate a PointCloud2 message from pcl's cloud.
```
  pcl::PointCloud<PointXYZRGB>::Ptr pcl_cloud = (some function to read in a cloud);
  sensor_msgs::PointCloud2 ros_cloud;
  pcl::toROSMsg(*pcl_cloud, ros_cloud);
```
2. Publish ros_cloud, and view it by "rostopic echo", we get the following contents (see below).
3. We can see the message has:  
    a) 32 byte length,  
    b) "rgb" at 16th byte,   
    c) "rgb" datatype 7, which is float (4 bytes).    
4. Despite the above format converted from pcl::toROSMsg, we can still set things like this (And this is what this script is using):  
    a) 16 byte length,  
    b) "rgb" at 12th byte,  
    c) "rgb" datatype 6, which is uint32.  

----- Contents of PointCloud2 converted from pcl::toROSMsg and viewed by "rostopic echo" -----
```
header: 
  seq: 16
  stamp: 
    secs: 0
    nsecs:         0
  frame_id: ""
height: 1
width: 1706
fields: 
  - 
    name: "x"
    offset: 0
    datatype: 7
    count: 1
  - 
    name: "y"
    offset: 4
    datatype: 7
    count: 1
  - 
    name: "z"
    offset: 8
    datatype: 7
    count: 1
  - 
    name: "rgb"
    offset: 16
    datatype: 7
    count: 1
is_bigendian: False
point_step: 32
row_step: 54592
data: ...
```