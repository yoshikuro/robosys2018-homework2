#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import cv2
import numpy as np
import math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class BallSearch(object):
  def __init__(self):
#Subscriber
    self._image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback_image)

#Publisher
    self._blue_pub = rospy.Publisher("image_blue", Image, queue_size = 10)
    self._red_pub = rospy.Publisher("image_red", Image, queue_size = 10)
    self._yellow_pub = rospy.Publisher("image_yellow", Image, queue_size = 10)
    self._result_pub = rospy.Publisher("image_result", Image, queue_size = 10)

#Function
    self._bridge = CvBridge()
    self.image_result = []

#ノイズ処理
  def noise_cut(self, befor_image):
    kernel = np.ones((5, 5), np.uint8)
    after_image = cv2.morphologyEx(befor_image, cv2.MORPH_OPEN, kernel)
    after_image = cv2.morphologyEx(after_image, cv2.MORPH_CLOSE, kernel)

    return after_image

#色抽出
  def get_color(self, cv_image, lower, upper):
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    mask_image = cv2.inRange(hsv_image, lower, upper)
    mask_image = self.noise_cut(mask_image)

    return mask_image

#カメラの映像で実行する関数
  def callback_image(self, data):
    try:
      cv_image = self._bridge.imgmsg_to_cv2(data, "bgr8")
      self.image_result = cv_image
    except CvBridgeError, e:
      print e
#各色の閾値
    lower_blue = np.array([100, 75, 23])
    upper_blue = np.array([120, 255, 255])
    lower_red1 = np.array([0, 30, 30])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 30, 30])
    upper_red2 = np.array([180, 255, 255])
    lower_yellow = np.array([18, 94, 69])
    upper_yellow = np.array([30, 255, 255])

    mask_blue = self.get_color(cv_image, lower_blue, upper_blue)
    red_image1 = self.get_color(cv_image, lower_red1, upper_red1)
    red_image2 = self.get_color(cv_image, lower_red2, upper_red2)
    mask_red = cv2.bitwise_or(red_image1, red_image2)
    mask_yellow = self.get_color(cv_image, lower_yellow, upper_yellow)

    try:
      self._blue_pub.publish(self._bridge.cv2_to_imgmsg(mask_blue, "mono8"))
      self._red_pub.publish(self._bridge.cv2_to_imgmsg(mask_red, "mono8"))
      self._yellow_pub.publish(self._bridge.cv2_to_imgmsg(mask_yellow, "mono8"))
    except CvBridgeError, e:
      print e

    self.ball_search(mask_blue, "blue")
    self.ball_search(mask_red, "red")
    self.ball_search(mask_yellow, "yellow")
    self._result_pub.publish(self._bridge.cv2_to_imgmsg(self.image_result, "bgr8"))

  def ball_search(self, mask, color):
    cnt_img, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for i in range(len(contours)):
      area = cv2.contourArea(contours[i])
      cnt_center, cnt_radius = cv2.minEnclosingCircle(contours[i])
      circle_level, radius = self.calCircleLevel(contours[i], area)
      if circle_level >= 0.60:
        if radius >= 10.0:
          if color is "blue":
            cv2.rectangle(self.image_result, (int(cnt_center[0]-radius), int(cnt_center[1]-radius)), (int(cnt_center[0]+radius), int(cnt_center[1]+radius)), (255, 0, 0), 5)
          elif color is "red":
            cv2.rectangle(self.image_result, (int(cnt_center[0]-radius), int(cnt_center[1]-radius)), (int(cnt_center[0]+radius), int(cnt_center[1]+radius)), (0, 0, 255), 5)
          elif color is "yellow":
            cv2.rectangle(self.image_result, (int(cnt_center[0]-radius), int(cnt_center[1]-radius)), (int(cnt_center[0]+radius), int(cnt_center[1]+radius)), (0, 255, 255), 5)
          else:
            pass

        else:
          pass 
      else:
        pass

  def calCircleLevel(self, contour, area):
    perimeter = cv2.arcLength(contour, True)
    if perimeter != 0.0:
      circle_level = 4.0 * np.pi * area / (perimeter * perimeter)
      r = perimeter / (2 * np.pi)

      return circle_level, r
    else:
      return 0.0, 0.0

if __name__ == "__main__":
  rospy.init_node("ball_search")
  func = BallSearch()

  try:
    rospy.spin()

  except KeyboardInterrupt:
    pass
