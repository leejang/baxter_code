Baxter Learning From Egocentric Videos
====================
This is ROS package for Baxter learning from egocentric videos project.

## Requirements

* ROS build environment (catkin)
  * Currently, this package uses ROS Indigo
  * http://www.ros.org/install/

* Caffe
  * http://caffe.berkeleyvision.org/

* Run

``` bash
// 0) lunch ZED Camera
roslaunch zed_wrapper zed.launch

// 1) Run ROS image viewer
rosrun image_view image_view image:=/zed/rgb/image_rect_color

// 2) (optional) Run the find-object package
// for calibration cameras and check arm positions in image coordinates
roslaunch find_object_2d find_object_3d.launch



