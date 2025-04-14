#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Cube Merged Node
功能：
  1. 订阅 /front/image_raw 获取图像，进行灰度化、阈值分割与轮廓检测，
     利用模板匹配识别图像中立方体上的数字。
  2. 利用相机内参、TF、激光雷达数据（/front/scan）将检测到的 ROI 定位到全局坐标系。
  3. 维护全局立方体记录，根据距离判断同一立方体、更新数字集合，并依据预设平原A区域进行统计，
     最终统计平原A中各数字的出现次数及出现次数最少的数字，并定时发布统计结果到 /cube_tracker/statistics。
"""

import rospy
import cv2
import numpy as np
import os
import json
import math
import threading
import tf2_ros
import tf2_geometry_msgs
from sensor_msgs.msg import Image, LaserScan, CameraInfo
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge, CvBridgeError


class CubeMergedNode:
    def __init__(self):
        rospy.init_node('cube_merged_node', anonymous=False)

        # CV Bridge
        self.bridge = CvBridge()

        # TF2 相关：建立 Buffer 与 Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # 参数设置
        self.thresh_val = rospy.get_param("~thresh_val", 10)  # 灰度阈值
        self.min_area = rospy.get_param("~min_area", 100)  # 轮廓最小面积
        self.match_threshold = rospy.get_param("~match_threshold", 0.6)  # 模板匹配得分阈值
        self.target_frame = rospy.get_param("~target_frame", "odom")  # 全局坐标系目标（odom）
        self.map_frame = rospy.get_param("~map_frame", "map")  # 地图坐标系
        self.match_distance_thresh = rospy.get_param("~match_distance_thresh", 0.8 * 1.414)  # 去重距离阈值

        # 平原A区域，多边形点列表，格式如 [[x1, y1], [x2, y2], ..., [xn, yn]]
        self.region_a_polygon = rospy.get_param("~region_a_polygon", [[20.8, -1.5], [8.8, -1.5], [20.8, -23.3], [8.8, -23.3]])
        if not self.region_a_polygon:
            rospy.logerr("请通过参数 ~region_a_polygon 设置平原A区域多边形点列表")
            rospy.signal_shutdown("缺少平原A区域定义")

        # 加载数字模板
        self.templates = self.load_digit_templates()
        if not self.templates:
            rospy.logerr("数字模板加载失败，请确认 templates 目录下存在 0.jpg~9.jpg 模板。")
            rospy.signal_shutdown("缺少数字模板")

        # 订阅话题
        self.scan_sub = rospy.Subscriber('/front/scan', LaserScan, self.scan_callback, queue_size=10)  # tim551
        self.camera_info_sub = rospy.Subscriber('/front/camera_info', CameraInfo, self.camera_info_callback,
                                                queue_size=1)  #
        self.image_sub = rospy.Subscriber('/front/image_raw', Image, self.image_callback, queue_size=1)  # front_camera_optical

        # 发布统计结果
        self.stats_pub = rospy.Publisher('/cube_tracker/statistics', String, queue_size=10)

        # 缓存激光扫描数据
        self.latest_scan = None
        self.scan_frame = None

        # 相机内参参数，初始为空
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.camera_frame = None

        # 全局立方体记录（用于追踪与统计）
        # 格式：{ cube_id: { "id": cube_id, "position": {"x", "y"}, "digits": set(), "last_update": time, "in_region": bool } }
        self.global_cubes = {}
        self.next_cube_id = 1
        self.lock = threading.Lock()

        # 定时发布统计结果
        self.publish_interval = rospy.get_param("~publish_interval", 2.0)
        rospy.Timer(rospy.Duration(self.publish_interval), self.publish_statistics)

    def load_digit_templates(self):
        """
        加载数字模板，要求模板为灰度图，进行二值化处理后存入字典中。
        返回字典：key 为数字（0~9），value 为模板图像（灰度图）。
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
        # 缓存最新的激光扫描数据及其坐标系
        self.latest_scan = msg
        if self.scan_frame is None:
            self.scan_frame = msg.header.frame_id

    def image_callback(self, msg):
        try:
            # 将 ROS 图像转换为 OpenCV 图像
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
        # 存储本帧的局部定位结果
        frame_cube_positions = []

        for cnt in contours:
            rospy.loginfo(f'检测到数字')

            area = cv2.contourArea(cnt)
            if area < self.min_area:
                continue  # 过滤噪声区域
            x, y, w, h = cv2.boundingRect(cnt)
            roi = thresh[y:y + h, x:x + w]

            # 利用模板匹配识别 ROI 中的数字
            recognized_digit, score = self.recognize_digit(roi)
            if recognized_digit is None:
                continue

            # 定位步骤：利用相机内参、TF 与激光扫描数据将图像中的 ROI 定位到全局坐标
            # 检查是否已有内参和激光数据
            if self.fx is None:
                rospy.logwarn("等待相机内参数据...")
                continue

            if self.latest_scan is None:
                rospy.logwarn("等待激光扫描数据...")
                continue

            # 计算 ROI 中心点像素坐标
            u = x + w / 2.0
            v = y + h / 2.0
            # 根据针孔模型转换到相机光学坐标系（构造射线方向，不归一化）
            ray_cam = [(u - self.cx) / self.fx, (v - self.cy) / self.fy, 1.0]

            # 构造相机坐标系下的点（选取 z=1）
            point_cam = PointStamped()
            point_cam.header.stamp = rospy.Time(0)
            point_cam.header.frame_id = self.camera_frame
            point_cam.point.x = ray_cam[0]
            point_cam.point.y = ray_cam[1]
            point_cam.point.z = ray_cam[2]

            # 利用 TF 将该点转换到激光雷达坐标系
            try:
                point_laser = self.tf_buffer.transform(point_cam, self.scan_frame, rospy.Duration(1.0))
                # rospy.loginfo("成功由相机坐标系转换到激光雷达坐标系",)

            except Exception as ex:
                rospy.logwarn("TF转换错误（从 %s 到 %s）：%s", self.camera_frame, self.scan_frame, str(ex))
                continue

            # 在激光坐标系中，计算该点相对于正前方的水平角度
            angle = math.atan2(point_laser.point.y, point_laser.point.x)

            # 从最新激光数据中查找该角度的距离
            scan = self.latest_scan
            if angle < scan.angle_min or angle > scan.angle_max:
                rospy.logwarn("检测角度 %.2f 超出激光扫描范围 [%.2f, %.2f]", angle, scan.angle_min, scan.angle_max)
                continue
            index = int((angle - scan.angle_min) / scan.angle_increment)
            if index < 0 or index >= len(scan.ranges):
                rospy.logwarn("激光索引 %d 超出范围", index)
                continue
            range_measure = scan.ranges[index]
            if math.isinf(range_measure) or math.isnan(range_measure):
                rospy.logwarn("激光测量无效，角度 %.2f", angle)
                continue

            # 根据激光数据计算立方体在激光坐标系中的 2D 坐标（假设地面 z=0）
            cube_x_laser = range_measure * math.cos(angle)
            cube_y_laser = range_measure * math.sin(angle)
            cube_point_laser = PointStamped()
            cube_point_laser.header.stamp = rospy.Time(0)
            cube_point_laser.header.frame_id = self.scan_frame
            cube_point_laser.point.x = cube_x_laser
            cube_point_laser.point.y = cube_y_laser
            cube_point_laser.point.z = 0.0

            # # 转换到全局坐标系：先转换到 target_frame (例如 odom)
            # try:
            #     cube_point_odom = self.tf_buffer.transform(cube_point_laser, self.target_frame, rospy.Duration(1.0))
            #     rospy.loginfo("成功由激光坐标系转换到 odom 坐标系")
            # except Exception as ex:
            #     rospy.logwarn("TF转换错误（从 %s 到 %s）：%s", self.scan_frame, self.target_frame, str(ex))
            #     continue

            # 再转换到地图坐标系 (map)
            try:
                cube_point_map = self.tf_buffer.transform(cube_point_laser, self.map_frame, rospy.Duration(1.0))
                # rospy.loginfo("成功由 odom 坐标系转换到地图坐标系")
            except Exception as ex:
                rospy.logwarn("TF转换错误（从 %s 到 %s）：%s", self.scan_frame, self.map_frame, str(ex))
                continue

            # 在原图上标注检测结果
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(cv_image, str(recognized_digit), (x, y - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

            cube_position = {"x": cube_point_map.point.x, "y": cube_point_map.point.y}

            rospy.loginfo(f"--- 识别到数字: {recognized_digit}, 置信度: {score}, 位置: ({cube_point_map.point.x}, {cube_point_map.point.y}) ---")

            # 将本次定位结果保存，用于全局跟踪更新
            frame_cube_positions.append({
                "digit": recognized_digit,
                "position": cube_position,
                "match_score": score
            })

        if len(frame_cube_positions) > 0:
            # 将本帧检测与定位结果更新到全局立方体记录中（去重追踪）
            self.update_global_cubes(frame_cube_positions)

        # 可选：显示标注后的图像
        cv2.imshow("Cube Merged Node", cv_image)
        cv2.waitKey(1)

    def recognize_digit(self, roi):
        """
        利用模板匹配识别 ROI 中的数字。
        ROI 调整到固定尺寸（28x28），对所有模板进行匹配，
        返回得分最高且超过阈值的数字及得分；否则返回 None。
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

    def update_global_cubes(self, cube_positions):
        """
        根据本帧的立方体定位结果更新全局记录。
        依据距离阈值判断是否为同一立方体，更新识别到的数字集合、位置和最后更新时间，
        同时利用预设区域判断该立方体是否处于平原A内。
        """
        current_time = rospy.get_time()
        with self.lock:
            for cube in cube_positions:
                try:
                    digit = cube["digit"]
                    pos = cube["position"]
                    x = pos["x"]
                    y = pos["y"]
                    # 判断是否在平原A区域内
                    in_region = self.point_in_polygon((x, y), self.region_a_polygon)

                    if in_region:
                        rospy.loginfo("--- 立方体在平原A区域内 ---")
                    else:
                        rospy.loginfo("--- 立方体不在平原A区域内 ---")

                    # 检查是否已有全局记录与该位置匹配（距离小于阈值视为同一立方体）
                    matched_cube_id = None
                    for cube_id, info in self.global_cubes.items():
                        existing_x = info["position"]["x"]
                        existing_y = info["position"]["y"]
                        existing_digits_set = info["digits"]
                        dist = math.sqrt((x - existing_x) ** 2 + (y - existing_y) ** 2)
                        rospy.loginfo(f'位于 ({x}, {y}) 处的数字 {digit} 与仓库中 {cube_id} 号立方体（位置[{existing_x}, {existing_y}]表面数字 {existing_digits_set}）相距 {dist:.4f} m')

                        if dist < self.match_distance_thresh and list(existing_digits_set)[0] == digit:
                            rospy.loginfo(f'数字匹配，距离匹配，为同一立方体！')
                            matched_cube_id = cube_id
                            break


                    if matched_cube_id is not None:
                        # 更新已存在记录
                        existing_cube = self.global_cubes[matched_cube_id]
                        existing_cube["digits"].add(digit)
                        existing_cube["position"]["x"] = (existing_cube["position"]["x"] + x) / 2.0
                        existing_cube["position"]["y"] = (existing_cube["position"]["y"] + y) / 2.0
                        existing_cube["last_update"] = current_time
                        existing_cube["in_region"] = in_region
                    else:
                        # 新立方体，分配新 id
                        cube_id = self.next_cube_id
                        self.next_cube_id += 1
                        self.global_cubes[cube_id] = {
                            "id": cube_id,
                            "position": {"x": x, "y": y},
                            "digits": set([digit]),
                            "last_update": current_time,
                            "in_region": in_region
                        }
                        rospy.loginfo(f'未发现可匹配的立方体，分配新 id 为 {cube_id}！')

                except Exception as ex:
                    rospy.logerr("更新立方体记录时出错: %s", str(ex))
                    continue

            rospy.loginfo(f"### 更新匹配结束 ###")

    def point_in_polygon(self, point, polygon):
        """
        判断二维点是否位于多边形内（射线法）。
        point: (x, y)
        polygon: list of [x, y]
        """
        x, y = point
        inside = False
        n = len(polygon)
        if n < 3:
            return False
        p1x, p1y = polygon[0]
        for i in range(1, n + 1):
            p2x, p2y = polygon[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / float(p2y - p1y) + p1x
                        else:
                            xinters = p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y
        return inside

    def publish_statistics(self, event):
        """
        定时统计平原A区域中各数字出现次数（每个立方体中数字只计一次），
        并找出出现次数最少的数字，将统计结果以 JSON 格式发布。
        """
        digit_counts = {}
        with self.lock:
            for cube_id, info in self.global_cubes.items():
                if not info["in_region"]:
                    continue
                if len(info["digits"]) > 1:
                    rospy.logerr(f"立方体 {cube_id} 记录错误，表面数字为 {info['digits']}，出现不同数字！")

                for digit in info["digits"]:
                    key = str(digit)
                    digit_counts[key] = digit_counts.get(key, 0) + 1

        if not digit_counts:
            stats = {"digit_counts": digit_counts, "min_digit": None, "min_count": 0}
        else:
            min_count = min(digit_counts.values())
            min_digits = [int(d) for d, count in digit_counts.items() if count == min_count]
            min_digit = min(min_digits) if min_digits else None
            stats = {"digit_counts": digit_counts, "min_digit": min_digit, "min_count": min_count}

        stats_json = json.dumps(stats)
        self.stats_pub.publish(stats_json)
        rospy.loginfo("发布统计结果: %s", stats_json)

    def run(self):
        rospy.spin()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        node = CubeMergedNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
