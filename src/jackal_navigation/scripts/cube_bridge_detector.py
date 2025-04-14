#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Bridge Detector 节点
功能：
  1. 当订阅到 /count_over 触发消息后，开始检测桥上锥筒区域（橙色部分）。
  2. 通过颜色分割方法检测图像中的锥筒区域，提取其外接矩形。
  3. 利用摄像头内参、TF 与激光扫描数据，将锥筒在图像中的中心位置转换到地图坐标系中，
     得到锥筒（桥中部）的全局位置。
  4. 根据“桥是直的”和“锥筒放置在桥中间”的先验，将锥筒位置的 x 坐标加 3，
     得到桥头中心在地图坐标系中的位置。
  5. 最终以 JSON 格式发布桥头中心位置到 /bridge_detector/bridge_head 话题。
"""

import rospy
import cv2
import numpy as np
import math
import json
import tf2_ros
import tf2_geometry_msgs
from sensor_msgs.msg import Image, LaserScan, CameraInfo
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped
from cv_bridge import CvBridge, CvBridgeError
from threading import Thread
import time


class BridgeDetector:
    def __init__(self):
        rospy.init_node('bridge_detector', anonymous=False)

        self.robot_pose = None

        # 标志：当收到 /count_over 触发后开始检测
        self.start_detection = False
        # 防止重复检测：只检测一次
        self.bridge_detected = False

        # CvBridge 用于图像转换
        self.bridge = CvBridge()

        # TF2 相关：Buffer 与 Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # 全局缓存：最新激光扫描数据及其坐标系
        self.latest_scan = None
        self.scan_frame = None

        # 相机内参参数（通过 /front/camera_info 获取）
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.camera_frame = None

        # 当收到 /crossed_bridge 消息且为 True 时，不再检测和发布
        self.crossed_bridge = False

        # 保存最新一帧图像（cv2 格式），用于检测锥筒
        self.latest_image = None

        # 最小有效区域面积占图像总面积的比例，用于过滤噪声
        self.area_thresh_ratio = [0.001, 0.02]

        # 订阅话题：摄像头图像、激光扫描、相机内参、触发检测信号
        self.image_sub = rospy.Subscriber('/front/image_raw', Image, self.image_callback, queue_size=1)
        self.scan_sub = rospy.Subscriber('/front/scan', LaserScan, self.scan_callback, queue_size=10)
        self.camera_info_sub = rospy.Subscriber('/front/camera_info', CameraInfo, self.camera_info_callback,
                                                queue_size=1)
        self.count_over_sub = rospy.Subscriber('/count_over', Bool, self.count_over_callback, queue_size=1)
        self.crossed_bridge_sub = rospy.Subscriber('/crossed_bridge', Bool, self.crossed_bridge_callback, queue_size=1)
        self.robot_pose_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.pose_callback)

        # 发布桥头位置检测结果
        self.bridge_pub = rospy.Publisher('/bridge_detector/bridge_head', String, queue_size=10)

        rospy.loginfo("BridgeDetector 节点初始化完成，等待 /count_over 触发...")

        # 开启独立线程用于图像显示，避免在回调中直接调用 cv2.imshow 导致卡住
        self.display_thread = Thread(target=self.image_display_loop)
        self.display_thread.daemon = True
        self.display_thread.start()

    def pose_callback(self, msg):
        self.robot_pose = msg.pose.pose

    def crossed_bridge_callback(self, msg):
        # 当收到 /crossed_bridge 消息且其 data 为 True 时，取消订阅 /front/scan 并停止图像检测和桥头位置发布
        if msg.data:
            rospy.loginfo("收到 /crossed_bridge 消息，取消 /front/scan 订阅，不再进行图像检测和桥头位置发布")
            if self.scan_sub:
                self.scan_sub.unregister()
                self.scan_sub = None
            self.crossed_bridge = True
            self.crossed_bridge_sub.unregister()

    def count_over_callback(self, msg):
        # 当收到 /count_over 消息且其 data 为 True 时，启动桥检测
        if msg.data:
            rospy.loginfo("收到 /count_over 触发消息，开始检测桥位置")
            self.start_detection = True
            self.count_over_sub.unregister()

    def camera_info_callback(self, msg):
        # 从 CameraInfo 消息中获取内参
        self.fx = msg.K[0]
        if self.fx is None:
            rospy.logwarn("相机内参 fx 获取无效，请检查相机标定结果")
            return

        self.fy = msg.K[4]
        if self.fy is None:
            rospy.logwarn("相机内参 fy 获取无效，请检查相机标定结果")
            return

        self.cx = msg.K[2]
        if self.cx is None:
            rospy.logwarn("相机内参 cx 获取无效，请检查相机标定结果")
            return

        self.cy = msg.K[5]
        if self.cy is None:
            rospy.logwarn("相机内参 cy 获取无效，请检查相机标定结果")
            return

        self.camera_frame = msg.header.frame_id
        if self.camera_frame is None:
            rospy.logwarn("相机内参 frame 获取无效，请检查相机标定结果")
            return

        rospy.loginfo("已获取相机内参:")
        rospy.loginfo("fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f, frame=%s",
                      self.fx, self.fy, self.cx, self.cy, self.camera_frame)
        # 获取内参后可取消订阅以减少带宽
        self.camera_info_sub.unregister()

    def scan_callback(self, msg):
        # 缓存最新激光扫描数据
        self.latest_scan = msg
        if self.scan_frame is None:
            self.scan_frame = msg.header.frame_id

    def image_callback(self, msg):
        if self.crossed_bridge:
            # rospy.loginfo("已收到 /crossed_bridge True，cube停止图像检测")
            return

        try:
            # 转换 ROS 图像消息到 OpenCV BGR 格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge转换错误: %s", str(e))
            return

        # 更新最新图像（复制一份，避免多线程冲突）
        self.latest_image = cv_image.copy()

        # 如果未收到 /count_over 或已经检测到，则直接返回
        if not self.start_detection:
            # rospy.loginfo("未收到 /count_over 触发消息，等待触发...")
            return
        if self.bridge_detected:
            # rospy.loginfo("桥头位置已检测，等待下一次触发...")
            return

        # 调用橙色锥筒检测函数（颜色分割方式）
        found, bbox, processed_image = self.detect_cone_in_image(cv_image, area_thresh_ratio=self.area_thresh_ratio)
        if found:
            # 锥筒区域检测到，计算其中心像素
            x, y, w, h = bbox
            u = x + w / 2.0
            v = y + h / 2.0
            rospy.loginfo("检测到锥筒区域，bounding box = (%d, %d, %d, %d)，中心像素 (%.2f, %.2f)", x, y, w, h, u, v)

            # 下面利用与 cube_localizer 类似的方法将图像坐标转换到地图坐标
            # 检查是否已获取相机内参与激光扫描数据
            if self.fx is None:
                rospy.logwarn("等待相机内参数据进行锥筒检测...")
                return

            if self.latest_scan is None:
                rospy.logwarn("等待激光扫描数据进行锥筒检测...")
                return

            # 构造相机光学坐标系下的射线（取 z=1）
            ray_cam = [(u - self.cx) / self.fx, (v - self.cy) / self.fy, 1.0]
            point_cam = PointStamped()
            point_cam.header.stamp = rospy.Time(0)
            point_cam.header.frame_id = self.camera_frame
            point_cam.point.x = ray_cam[0]
            point_cam.point.y = ray_cam[1]
            point_cam.point.z = ray_cam[2]

            # 利用 TF 将该点从相机坐标系转换到激光坐标系
            try:
                point_laser = self.tf_buffer.transform(point_cam, self.scan_frame, rospy.Duration(1.0))
            except Exception as ex:
                rospy.logwarn("TF转换错误（从 %s 到 %s）：%s", self.camera_frame, self.scan_frame, str(ex))
                return

            # 计算在激光坐标系中该点的水平角度（假设 x 为前方，y 为左侧）
            angle = math.atan2(point_laser.point.y, point_laser.point.x)

            # 利用激光数据获得该角度处的距离
            scan = self.latest_scan
            if angle < scan.angle_min or angle > scan.angle_max:
                rospy.logwarn("检测角度 %.2f 超出激光扫描范围 [%.2f, %.2f]", angle, scan.angle_min, scan.angle_max)
                return
            index = int((angle - scan.angle_min) / scan.angle_increment)
            if index < 0 or index >= len(scan.ranges):
                rospy.logwarn("激光索引 %d 超出范围", index)
                return
            range_measure = scan.ranges[index]
            if math.isinf(range_measure) or math.isnan(range_measure):
                rospy.logwarn("激光测量无效，角度 %.2f", angle)
                return

            # 根据激光测量，计算锥筒在激光坐标系下的 2D 坐标（假设地面 z=0）
            cone_x_laser = range_measure * math.cos(angle)
            cone_y_laser = range_measure * math.sin(angle)
            cone_point_laser = PointStamped()
            cone_point_laser.header.stamp = rospy.Time(0)
            cone_point_laser.header.frame_id = self.scan_frame
            cone_point_laser.point.x = cone_x_laser
            cone_point_laser.point.y = cone_y_laser
            cone_point_laser.point.z = 0.0

            # # 将锥筒位置从激光坐标系转换到全局坐标系（先转换到 target_frame，例如 "odom"）
            # try:
            #     cone_point_odom = self.tf_buffer.transform(cone_point_laser, "odom", rospy.Duration(1.0))
            # except Exception as ex:
            #     rospy.logwarn("TF转换错误（从 %s 到 odom）：%s", self.scan_frame, str(ex))
            #     return

            # 再转换到地图坐标系 ("map")
            try:
                cone_point_map = self.tf_buffer.transform(cone_point_laser, "map", rospy.Duration(1.0))
                # rospy.loginfo("成功由 laser 坐标系转换到地图坐标系")
            except Exception as ex:
                rospy.logwarn("TF转换错误（从 odom 到 map）：%s", str(ex))
                return

            if self.robot_pose.position.y > cone_point_map.point.y:
                cone_y_map = cone_point_map.point.y - 0.78  # 桥在车右端补充0.78米
            elif self.robot_pose.position.y < cone_point_map.point.y:
                cone_y_map = cone_point_map.point.y + 0.78  # 桥在车左端补充0.78米

            # 得到锥筒在地图坐标系中的位置
            cone_x_map = cone_point_map.point.x

            # 防止误检测桥下
            if cone_x_map < (8.8 - 1.6):
                return

            rospy.loginfo("锥筒在地图坐标系中的位置： (%.2f, %.2f)", cone_x_map, cone_y_map)

            # # 根据先验：桥是直的且锥筒位于桥中间，
            # # 将锥筒位置的 x 坐标加 3 得到桥头中心位置
            # bridge_head_x = cone_x_map + 3.0
            # bridge_head_y = cone_y_map
            # rospy.loginfo("计算得到桥头中心位置： (%.2f, %.2f)", bridge_head_x, bridge_head_y)
            #
            # # 构造 JSON 消息发布结果
            # result = {"bridge_head": {"x": bridge_head_x, "y": bridge_head_y}}
            result = {"bridge_head": {"x": cone_x_map + 3.0, "y": cone_y_map}}

            # 如果已收到 /crossed_bridge True，则不发布桥头位置
            if self.crossed_bridge:
                rospy.loginfo("已收到 /crossed_bridge True，不发布桥头位置")
                return
            else:
                self.bridge_pub.publish(json.dumps(result))
            rospy.loginfo("🎯🎯🎯🎯🎯🎯🎯🎯🎯🎯🎯🎯🎯🎯🎯🎯🎯🎯🎯🎯🎯🎯🎯发布桥头检测结果: %s", json.dumps(result))

            # 在图像上绘制检测区域，并更新显示图像
            cv2.rectangle(processed_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            self.latest_image = processed_image.copy()

            # 检测成功后置标志，避免重复检测
            self.bridge_detected = True

    def detect_cone_in_image(self, cv_image, area_thresh_ratio):
        """
        检测图像中是否存在橙色锥筒区域，并锁定其外接矩形
        参数：
          cv_image: OpenCV 格式 BGR 图像
          area_thresh_ratio: 最小有效区域面积占图像总面积的比例，用于过滤噪声
        返回：
          found: Boolean，True 表示检测到锥筒（若存在多个区域，则合并所有轮廓）
          bbox: 如果检测到锥筒，返回外接矩形 (x, y, w, h)；否则为 None
          output_image: 原图（可用于展示检测结果）
        """
        height, width = cv_image.shape[:2]
        image_area = height * width

        # 转换到 HSV 色彩空间
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # 定义橙色范围
        lower_orange = np.array([10, 100, 100])
        upper_orange = np.array([25, 255, 255])
        mask = cv2.inRange(hsv, lower_orange, upper_orange)

        # 形态学操作：先开操作去除小白点，再膨胀
        kernel1 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        open_mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel1)
        kernel2 = np.ones((5, 5), np.uint8)
        dilate_mask = cv2.morphologyEx(open_mask, cv2.MORPH_DILATE, kernel2)

        # 查找轮廓
        contours, _ = cv2.findContours(dilate_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        valid_contours = []

        min_area = area_thresh_ratio[0] * image_area
        max_area = area_thresh_ratio[1] * image_area

        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)

            # 忽略过宽的
            if w > 110 / 640 * width:
                continue

            # 图像上半部分
            if y > 0.5 * height:
                continue

            if cv2.contourArea(cnt) >= min_area and cv2.contourArea(cnt) <= max_area:
                valid_contours.append(cnt)

        if len(valid_contours) == 0:
            return False, None, cv_image
        else:
            # 合并所有有效轮廓
            all_points = np.vstack(valid_contours)
            x, y, w, h = cv2.boundingRect(all_points)

            return True, (x, y, w, h), cv_image

    def image_display_loop(self):
        """
        图像显示循环：
        仅当收到 /count_over（即 self.start_detection 为 True）后，才打开图像窗口进行显示。
        """
        while not rospy.is_shutdown() and not self.crossed_bridge:
            # 只有当触发检测后才显示图像
            if self.start_detection and self.latest_image is not None:
                cv2.imshow("Detected Cone", self.latest_image)
                cv2.waitKey(1)
            time.sleep(0.03)
        cv2.destroyAllWindows()


    def run(self):
        rospy.spin()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        node = BridgeDetector()
        node.run()
    except rospy.ROSInterruptException:
        pass
