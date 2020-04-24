PointCloud_DynamicObjectFinder ROS Package
===============================================

ROS package to find and track dynamic obstacles in a point cloud

This package was designed to work with a Velodyne VLP-16 LiDAR

The maximum range is 17.5 meters. This is the maximum reliable range with a LiDAR that scans at 10 Hz and generates about 1100 points per scan line. Running at a slower rate or with more points per scan line can increase the range

This package is very lightweight and should only use a fraction of a computers processing power 

How it works
-----------------------------------------------

This package takes user specified rings of a point cloud and voxelizes them

These voxelized point clouds are then subtracted to find the change between two frames

After filtering, moving objects are extracted from the voxelized data

The package attempts to track the dynamic objects, so unseen objects will still be published

The output is a custom message that includes the bounding boxes and classification of detected objects

The output can be visualized in rviz as well as a MarkerArray

Requirements
===============================================
	1) ROS Kinetic Desktop-Full Install
	2) Python 2.7.x
	3) OpenCV for Python (pip install opencv-python) (OpenCV should come with ROS as well)
	4) Scipy (sudo apt-get install python-scipy)

This software was developed for ROS Kinetic. Newer versions of ROS should work

Requirements to run our Dataset
===============================================
	1) All of the above requirements
	2) dbw_mkz_ros (https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/master/) and its dependencies
		- ROS CAN open (sudo apt-get install ros-kinetic-ros-canopen)
		- ROS Dataspeed CAN (sudo apt-get install ros-kinetic-dataspeed-can)

How to run this software
===============================================

I. Running this on a custom robot
-----------------------------------------------
You will only need the regular requirements. 

You will need to change the configuration files to match your topics

Run voxelizer.launch and ObjectFinder.launch

If you want to visualize your data, run visualizer.launch as well

II. Running this on our dataset
-----------------------------------------------
You will need the regular requirements and our dataset requirements

The configuration files are already set and tuned for our dataset, no need to change them

First run all.launch located in the voxelizer node

Then run the rosbag from the LIDAR_mapping_bicycle_pedestrian_detection dataset

To properly visualize, open rviz and add a Point Cloud and a Marker Array

Set the Point Cloud topic to /ns2/velodyne_points

Set the Marker Array topic to /visualized_bounding_boxes

For additional information, add an Image and set the topic to /data_image

III. Visualization Information
-----------------------------------------------
Blue bounding boxes are pedestrians

Yellow bounding boxes are vehicles

Green vounding boxes are cyclists

In the image, green countours are detected static contours, red circles are detected blobs, white spaces are moving objects (these are often covered by the detected blobs), and blue contours are contours considered to be dynamic objects

Current Issues
===============================================
The method used here does not tend to work well at detecting vechicles

Low to the ground noisy objects can cause considerable issues (mainly small trees). These are occasionally detected as vehicles and they can prevent detection of pedestrians if they get too close

The tracker will ocasionally detect the same object twice. This tends to happen to tracked objects when a new object appears. This is currently being investigated

Speed of the LiDAR plays an important role in overall effectiveness. When the LiDAR is traveling at speeds close to multiples of 0.1 meters/sec the results are better. In between values like 0.15 meters/sec result in a noisier result







 
