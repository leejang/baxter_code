#!/usr/bin/env python

import os
import sys
import threading
import rospy
import cv2
import numpy as np
import time

#sys.path.insert(0, '/home/leejang/lib/ssd_caffe/caffe/python')

caffe_root = '/home/leejang/lib/ssd_caffe/caffe'
os.chdir(caffe_root)
sys.path.insert(0, 'python')

import caffe
import re

# for Caffe
from google.protobuf import text_format
from caffe.proto import caffe_pb2

# for ROS
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from baxter_learning_from_egocentric_video.msg import Target
from cv_bridge import CvBridge, CvBridgeError
from baxter_core_msgs.msg import EndpointState
import image_geometry
import tf

labelmap_file = 'data/egohands_future/labelmap_voc.prototxt'
file = open(labelmap_file, 'r')
labelmap = caffe_pb2.LabelMap()
text_format.Merge(str(file.read()), labelmap)

def get_labelname(labelmap, labels):
    num_labels = len(labelmap.item)
    labelnames = []
    if type(labels) is not list:
        labels = [labels]
    for label in labels:
        found = False
        for i in xrange(0, num_labels):
            if label == labelmap.item[i].label:
                found = True
                labelnames.append(labelmap.item[i].display_name)
                break
        assert found == True
    return labelnames

pat = re.compile("(\d+)\D*$")
def key_func(x):
        mat=pat.search(os.path.split(x)[-1]) # match last group of digits
        if mat is None:
            return x
        return "{:>10}".format(mat.group(1)) # right align to 10 digits.

# SSD 500 x 500 with auto encoder
model_def = '/home/leejang/lib/ssd_caffe/caffe/models/VGGNet/egohands/SSD_BEST_AUTOENC/deploy.prototxt'
model_weights = '/home/leejang/lib/ssd_caffe/caffe/models/VGGNet/egohands/SSD_BEST_AUTOENC/egohands_SSD_2_500x500_iter_50000.caffemodel'

# future regression model for hands
# ten con
reg_model_def = '/home/leejang/lib/ssd_caffe/caffe/models/robot_regression/robot_regression_7cv_2fc_single_test.prototxt'
reg_model_weights = '/home/leejang/lib/ssd_caffe/caffe/models/robot_regression/7cv_2fc_single_iter_100000.caffemodel'


class hands_forecasting:

    def __init__(self):
     self.bridge = CvBridge()
     #############################################################
     # subscribers
     self.image_sub = rospy.Subscriber("/zed/rgb/image_rect_color", Image, self.img_callback, queue_size=1)
     #self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.img_callback, queue_size=1000)
     self.left_end_sub = rospy.Subscriber("/robot/limb/left/endpoint_state", EndpointState, self.left_end_cb)
     self.right_end_sub = rospy.Subscriber("/robot/limb/right/endpoint_state", EndpointState, self.right_end_cb)
     #self.camera_info_sub = rospy.Subscriber("/usb_cam/camera_info", CameraInfo, self.cam_info_cb)
     self.camera_info_sub = rospy.Subscriber("/zed/rgb/camera_info", CameraInfo, self.cam_info_cb)
     #self.camera_info_sub = rospy.Subscriber("/zed/depth/camera_info", CameraInfo, self.cam_info_cb)

     ##############################################################
     # pubishers
     self.screen_pub = rospy.Publisher('/robot/xdisplay', Image, latch=True)
     self.detection_pub = rospy.Publisher('/detection/right/target_pos',Target)

     # listner for TF
     self.listener = tf.TransformListener()

     # Target Psition to move Baxter Hands
     self.right_target_msg = Target()
     self.right_target_msg.x = 0
     self.right_target_msg.y = 0

     self.lock = threading.Lock()

     self.image_cnt = 0;
     self.gesture_cnt = 1;

    def img_callback(self,data):
      # processing time check
      t = time.time()


      print('img_calback')
      #print("image_cnt {:d} .".format(self.image_cnt))
      print("Proesssed in {:.3f} seconds.".format(time.time() - t))


      self.image_cnt += 1

    def cam_info_cb(self,data):
      # camera model
      self.cam_model = image_geometry.PinholeCameraModel()
      self.cam_model.fromCameraInfo(data)
      print('cam info callback!')

      # processing time check
    def left_end_cb(self, msg):
      tt = 1
      #print('left end state callback!')

    def right_end_cb(self, msg):
      tt = 1
      #print('right end state callback!')
    
    def do_hands_forecasting(self):

      # processing time check
      t = time.time()

      print('start hands forecasting!')

      cur_image = '/home/leejang/ros_ws/src/baxter_learning_from_egocentric_video/cur_image/cur_image.jpg'
      rhand_image = '/home/leejang/ros_ws/src/baxter_learning_from_egocentric_video/cur_image/rhand_new.jpg'

      cur_image_w_r = '/home/leejang/ros_ws/src/baxter_learning_from_egocentric_video/cur_image/cur_image_w_r.jpg'

      #new_target_image = '/home/leejang/ros_ws/src/baxter_learning_from_egocentric_video/new_robot_video/0110/'+str(self.image_cnt)+'.jpg'

      # img_size: 142 x 108
      rhand_cv_img = cv2.imread(rhand_image)

      # screen test
      cv_img = cv2.imread(cur_image)

      cv2.imwrite(cur_image_w_r, cv_img, [int(cv2.IMWRITE_JPEG_QUALITY), 100])
      #cv2.imwrite(cur_image, cv_img, [int(cv2.IMWRITE_JPEG_QUALITY), 100])

      # to publish image on Baxter's screen
      #img_msg = self.bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
      #self.screen_pub.publish(img_msg)

      # resize (batch,dim,height,width)
      #net.blobs['data'].reshape(1,3,360,640)

      # load image
      cur_image = '/home/leejang/ros_ws/src/baxter_learning_from_egocentric_video/cur_image/cur_image_w_r.jpg'
      image = caffe.io.load_image(cur_image)


      # to publish image on Baxter's screen
      img_msg = self.bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
      self.screen_pub.publish(img_msg)

      self.gesture_cnt += 1

def main(args):
    print 'initialize hands forecasting node (python)'

    rospy.init_node('hands_forecasting_node', anonymous=True)

    try:
      hands_forcasting_node = hands_forecasting()
      rospy.spin()
    except KeyboardInterrupt:
      print "Shutting down"

if __name__ == '__main__':
    main(sys.argv)
