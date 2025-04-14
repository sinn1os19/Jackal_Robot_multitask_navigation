#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Cube Target Detector 节点
流程：
    订阅'/cube_tracker/statistics'话题，订阅/crossed_bridge话题。
    当/crossed_bridge取值为true时，获取并存储'/cube_tracker/statistics'中的min_digit，然后开始对图像进行digit识别，检测并存储立方体的表面数字和位置，
    当识别到某个立方体的表面数字与min_digit相等时，直接发布该立方体的位置

功能：
  - 订阅 /cube_tracker/statistics（std_msgs/String，JSON格式，内容例如：{"digit_counts": digit_counts, "min_digit": min_digit, "min_count": min_count}）
  - 订阅 /crossed_bridge（std_msgs/Bool，值为 true 或 false）
  - 当 /crossed_bridge 为 true 时，获取并存储 /cube_tracker/statistics 中的 min_digit，
    开始对 /front/image_raw 图像进行数字识别（利用模板匹配），检测并存储立方体的表面数字和位置，
    当识别到的数字与 min_digit 相等时：
        如果是第一次检测到，则计算目标位置、存储并发布；
        如果已经检测过，则继续发布第一次检测到的位置，不更新为新位置。
  - 同时订阅 /front/scan（LaserScan）及 /front/camera_info（CameraInfo），利用相机内参和 tf 将图像中的 ROI 转换到地图坐标系，
    最终将目标立方体的位置（JSON格式）发布到 /target_cube/position 话题。
"""

import rospy
import cv2
import numpy as np
import os
import json
import math
import tf2_ros
import tf2_geometry_msgs
from sensor_msgs.msg import Image, LaserScan, CameraInfo
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge, CvBridgeError

class CubeTargetDetector:
    def __init__(self):
        rospy.init_node('cube_target_detector', anonymous=False)

        # CvBridge，用于 ROS 图像与 OpenCV 图像之间转换
        self.bridge = CvBridge()

        # 状态变量：是否开始检测、存储从 /cube_tracker/statistics 中获取的 min_digit
        self.active = False
        self.min_digit = None
        self.first_target_position = None  # 第一次检测到目标时计算得到的位置

        # 加载数字模板（要求模板为灰度图，存放在当前目录下的 templates 文件夹中，文件名 0.jpg~9.jpg）
        self.templates = self.load_digit_templates()
        if not self.templates:
            rospy.logerr("数字模板加载失败，请确认 templates 目录下存在 0.jpg~9.jpg 模板。")
            rospy.signal_shutdown("缺少数字模板")

        # 参数设置
        self.thresh_val = rospy.get_param("~thresh_val", 10)          # 灰度阈值
        self.min_area = rospy.get_param("~min_area", 100)               # 轮廓最小面积过滤
        self.match_threshold = rospy.get_param("~match_threshold", 0.6) # 模板匹配得分阈值

        # 激光扫描数据缓存
        self.latest_scan = None
        self.scan_frame = None

        # 相机内参（后续从 /front/camera_info 获取）
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.camera_frame = None

        # tf2：用于坐标转换
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # 订阅话题：
        # /cube_tracker/statistics：包含数字统计信息的 JSON 字符串
        self.statistics_sub = rospy.Subscriber('/cube_tracker/statistics', String, self.statistics_callback)
        # /crossed_bridge：Bool 类型，控制是否启动目标检测
        self.crossed_bridge_sub = rospy.Subscriber('/crossed_bridge', Bool, self.crossed_bridge_callback)
        # 图像数据
        self.image_sub = rospy.Subscriber('/front/image_raw', Image, self.image_callback, queue_size=1)
        # 激光数据
        self.scan_sub = rospy.Subscriber('/front/scan', LaserScan, self.scan_callback, queue_size=10)
        # 相机内参
        self.camera_info_sub = rospy.Subscriber('/front/camera_info', CameraInfo, self.camera_info_callback, queue_size=1)

        # 发布目标立方体位置（JSON 格式）
        self.target_pub = rospy.Publisher('/target_cube/position', String, queue_size=10)

        rospy.loginfo("CubeTargetDetector 节点初始化完成。")

    def load_digit_templates(self):
        """
        加载数字模板，返回一个字典，key 为数字（0~9），value 为模板图像（灰度图）
        """
        templates = {}
        script_dir = os.path.dirname(os.path.realpath(__file__))
        templates_dir = os.path.join(script_dir, "templates")
        for digit in range(10):
            template_path = os.path.join(templates_dir, "{}.jpg".format(digit))
            if not os.path.exists(template_path):
                rospy.logwarn("未找到模板文件: {}".format(template_path))
                continue
            template_img = cv2.imread(template_path, cv2.IMREAD_GRAYSCALE)
            if template_img is None:
                rospy.logwarn("加载模板失败: {}".format(template_path))
                continue
            templates[digit] = template_img
            rospy.loginfo(f"### 加载模板成功: {template_path} ###")
        return templates

    def statistics_callback(self, msg):
        """
        解析 /cube_tracker/statistics 消息，提取 min_digit
        """
        if self.active:
            # 过了桥才获得最少值
            try:
                data = json.loads(msg.data)
                if "min_digit" in data:
                    self.min_digit = data["min_digit"]
                    rospy.loginfo("获得总数最少的数字 min_digit: %s", str(self.min_digit))
            except Exception as e:
                rospy.logerr("解析 /cube_tracker/statistics JSON 失败: %s", str(e))

    def crossed_bridge_callback(self, msg):
        """
        当 /crossed_bridge 消息为 True 时启动检测；否则停止检测
        """
        if msg.data:
            self.active = True
            rospy.loginfo("crossed_bridge 为 True，开始目标检测。")
        else:
            self.active = False
            # rospy.loginfo("crossed_bridge 为 False，停止目标检测。")

    def scan_callback(self, msg):
        """
        缓存最新激光扫描数据
        """
        self.latest_scan = msg
        if self.scan_frame is None:
            self.scan_frame = msg.header.frame_id

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

    def image_callback(self, msg):
        """
        处理图像消息：
          1. 如果检测启动且已获取 min_digit，则进行图像预处理与轮廓检测
          2. 对每个候选区域利用模板匹配识别数字
          3. 当识别出的数字与 min_digit 相等时，
             利用相机内参、激光扫描数据和 tf 转换计算立方体在地图坐标系下的位置，
             并发布该目标位置信息
        """
        if not self.active:
            return
        if self.min_digit is None:
            rospy.logwarn("min_digit 尚未设置，等待 /cube_tracker/statistics 消息")
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge转换错误: {}".format(e))
            return

        # 灰度化及阈值处理
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, self.thresh_val, 255, cv2.THRESH_BINARY)
        # 为便于轮廓检测，取反二值图
        _, thresh = cv2.threshold(gray, self.thresh_val, 255, cv2.THRESH_BINARY_INV)

        # 轮廓检测：提取独立区域
        contours, _ = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 遍历所有轮廓，进行数字识别
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.min_area:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            roi = thresh[y:y+h, x:x+w]

            recognized_digit, score = self.recognize_digit(roi)
            if recognized_digit is not None:
                rospy.loginfo("检测到数字: %d, 得分: %.2f", recognized_digit, score)

                # 如果识别数字与 min_digit 相等，则进行定位发布
                if recognized_digit == self.min_digit:
                    # 计算 ROI 中心的像素坐标
                    u = x + w / 2.0
                    v = y + h / 2.0

                    if self.first_target_position is not None:
                        # 发布目标位置信息（始终为第一次检测到的位置）
                        self.target_pub.publish(json.dumps(self.first_target_position))
                        # 在图像上标注检测区域（调试用）
                        cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                        cv2.putText(cv_image, str(recognized_digit), (x, y - 5),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

                        continue

                    if self.fx is None or self.fy is None:
                        rospy.logwarn("相机内参未设置")
                        continue

                    # 利用针孔模型，将像素坐标转换到相机光学坐标系（构造一条射线）
                    ray_cam = [(u - self.cx) / self.fx, (v - self.cy) / self.fy, 1.0]

                    point_cam = PointStamped()
                    point_cam.header.stamp = rospy.Time(0)
                    point_cam.header.frame_id = self.camera_frame
                    point_cam.point.x = ray_cam[0]
                    point_cam.point.y = ray_cam[1]
                    point_cam.point.z = ray_cam[2]

                    # 利用 tf 将该点从相机坐标系转换到激光坐标系
                    try:
                        point_laser = self.tf_buffer.transform(point_cam, self.scan_frame, rospy.Duration(1.0))
                    except Exception as ex:
                        rospy.logwarn("TF转换错误（从 %s 到 %s）：%s", self.camera_frame, self.scan_frame, str(ex))
                        continue

                    # 计算该点在激光坐标系中的水平角度
                    angle = math.atan2(point_laser.point.y, point_laser.point.x)
                    if self.latest_scan is None:
                        rospy.logwarn("尚未接收到激光扫描数据")
                        continue

                    scan = self.latest_scan
                    if angle < scan.angle_min or angle > scan.angle_max:
                        rospy.logwarn("检测角度 %.2f 超出激光扫描范围 [%.2f, %.2f]", angle, scan.angle_min, scan.angle_max)
                        continue
                    index = int((angle - scan.angle_min) / scan.angle_increment)
                    if index < 0 or index >= len(scan.ranges):
                        rospy.logwarn("计算得到的激光索引 %d 超出范围", index)
                        continue
                    range_measure = scan.ranges[index]
                    if math.isinf(range_measure) or math.isnan(range_measure):
                        rospy.logwarn("激光测量无效，角度 %.2f", angle)
                        continue

                    # 计算立方体在激光坐标系下的 2D 坐标
                    cube_x_laser = range_measure * math.cos(angle)
                    cube_y_laser = range_measure * math.sin(angle)

                    # 构造激光坐标系下的立方体位置点（假设地面 z=0）
                    cube_point_laser = PointStamped()
                    cube_point_laser.header.stamp = rospy.Time(0)
                    cube_point_laser.header.frame_id = self.scan_frame
                    cube_point_laser.point.x = cube_x_laser
                    cube_point_laser.point.y = cube_y_laser
                    cube_point_laser.point.z = 0.0

                    # 利用 tf 将立方体位置从激光坐标系转换到地图坐标系
                    try:
                        cube_point_map = self.tf_buffer.transform(cube_point_laser, "map", rospy.Duration(1.0))
                    except Exception as ex:
                        rospy.logwarn("TF转换错误（从 %s 到 %s）：%s", self.scan_frame, "map", str(ex))
                        continue

                    cube_position = {"x": cube_point_map.point.x, "y": cube_point_map.point.y}
                    out_msg = {"digit": recognized_digit, "position": cube_position, "match_score": score}
                    self.target_pub.publish(json.dumps(out_msg))

                    self.first_target_position = {"digit": recognized_digit, "position": cube_position, "match_score": score}
                    rospy.loginfo("发布目标立方体位置: %s", json.dumps(out_msg))

                    # 标注检测区域（调试用）
                    cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                    cv2.putText(cv_image, str(recognized_digit), (x, y-5),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        # 显示调试图像
        cv2.imshow("Cube Target Detector", cv_image)
        cv2.waitKey(1)

    def recognize_digit(self, roi):
        """
        利用模板匹配对 ROI 内的数字进行识别，返回识别的数字及匹配得分
        """
        try:
            roi_resized = cv2.resize(roi, (28, 28))
        except Exception as e:
            rospy.logwarn("ROI调整尺寸失败: {}".format(e))
            return None, 0.0

        best_score = -1
        best_digit = None
        for digit, template in self.templates.items():
            template_resized = cv2.resize(template, (28, 28))
            result = cv2.matchTemplate(roi_resized, template_resized, cv2.TM_CCOEFF_NORMED)
            _, max_val, _, _ = cv2.minMaxLoc(result)
            if max_val > best_score:
                best_score = max_val
                best_digit = digit
        if best_score >= self.match_threshold:
            return best_digit, best_score
        else:
            return None, best_score

    def run(self):
        rospy.spin()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        detector = CubeTargetDetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass
