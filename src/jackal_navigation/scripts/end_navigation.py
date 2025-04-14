#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import actionlib
import random
import sys
import time
import json
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped, Quaternion, Twist, PoseWithCovarianceStamped
from std_srvs.srv import Empty
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import math
import subprocess
import tf
import nav_msgs.msg


class BoxNavigation:
    def __init__(self):
        # 节点由主函数初始化，这里不再调用 rospy.init_node()

        # 预设箱子位置 (x, y)
        self.box_positions = {
            1: (2.5, -6.0),
            2: (2.5, -10.0),
            3: (2.5, -14.0),
            4: (2.5, -18.0)
        }

        # 订阅 /crossed_bridge，收到 True 后开始导航，然后注销该订阅
        self.crossed_bridge_sub = rospy.Subscriber('/crossed_bridge', Bool, self.crossed_bridge_callback)

        # 订阅统计数据（用于提取 min_digit）
        self.statistics_sub = rospy.Subscriber('/cube_tracker/statistics', String, self.statistics_callback)
        self.min_digit = None
        self.active = False  # 当收到 /crossed_bridge 为 True 后置为 True

        # 订阅目标立方体位置
        self.target_cube_sub = rospy.Subscriber('/target_cube/position', String, self.target_cube_callback)
        self.target_cube_position = None

        # 订阅 AMCL 位姿
        self.robot_pose = None
        self.amcl_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.max_attempts = 3

        self.connect_to_move_base()

        # 尝试连接清除代价地图服务
        try:
            rospy.wait_for_service('/move_base/clear_costmaps', timeout=2.0)
            self.clear_costmaps = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
            rospy.loginfo("Connected to clear_costmaps service")
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logwarn("Failed to connect to clear_costmaps service: %s. Will continue without clearing costmaps.", str(e))
            self.clear_costmaps = None

        # 等待 AMCL 初始化
        rospy.loginfo("等待AMCL初始化...")
        start_time = rospy.Time.now()
        while self.robot_pose is None and (rospy.Time.now() - start_time) < rospy.Duration(5.0):
            rospy.sleep(0.5)

        if self.robot_pose is None:
            rospy.logwarn("未能接收到AMCL位姿，导航可能不准确")
        else:
            rospy.loginfo("AMCL初始化成功，当前位置: (%.2f, %.2f)",
                          self.robot_pose.position.x, self.robot_pose.position.y)

        rospy.loginfo("Box Navigation 初始化完成。")

    def crossed_bridge_callback(self, msg):
        """订阅 /crossed_bridge，当收到 True 时开始导航，并取消订阅"""
        if msg.data:
            rospy.loginfo("接收到 /crossed_bridge=True，开始导航")
            self.active = True  # 激活统计数据处理
            self.crossed_bridge_sub.unregister()  # 注销该订阅，不再监听
            self.navigate_to_boxes()

    def amcl_callback(self, msg):
        """接收 AMCL 发布的位姿估计"""
        self.robot_pose = msg.pose.pose

    def connect_to_move_base(self):
        """尝试连接到 move_base，不尝试杀死或重启节点"""
        self.move_base_client = None
        try:
            rospy.loginfo("尝试连接到 /move_base 动作服务器...")
            self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            if self.move_base_client.wait_for_server(rospy.Duration(5.0)):
                rospy.loginfo("已连接到 /move_base 服务器")
                return
        except Exception as e:
            rospy.logwarn("连接标准 move_base 时出错: %s", str(e))
        rospy.logwarn("无法连接到 move_base 动作服务器，将使用直接控制方法")
        self.move_base_client = None

    def create_goal(self, x, y, yaw=None):
        """创建一个 MoveBaseGoal，可选指定朝向"""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0.0

        if yaw is not None:
            q = quaternion_from_euler(0, 0, yaw)
            goal.target_pose.pose.orientation = Quaternion(*q)
        else:
            goal.target_pose.pose.orientation.w = 1.0

        return goal

    def try_clear_costmaps(self):
        """尝试清除代价地图"""
        if self.clear_costmaps is not None:
            try:
                self.clear_costmaps()
                rospy.loginfo("Costmaps cleared")
                return True
            except rospy.ServiceException as e:
                rospy.logwarn("Failed to clear costmaps: %s", str(e))
        return False

    def get_current_pose(self):
        """从 AMCL 获取当前位姿"""
        if self.robot_pose is None:
            rospy.logwarn("当前没有可用的AMCL定位数据")
            return None, None, None

        current_x = self.robot_pose.position.x
        current_y = self.robot_pose.position.y
        orientation_q = self.robot_pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, current_yaw = euler_from_quaternion(orientation_list)
        return current_x, current_y, current_yaw

    def direct_control_to_position(self, target_x, target_y, speed=0.5):
        """使用直接控制移动到目标位置，不强制最终朝向"""
        rospy.loginfo("使用直接控制移动到位置 (%.2f, %.2f)", target_x, target_y)
        dist_tolerance = 0.2
        angle_tolerance = 0.1
        rate = rospy.Rate(10)
        timeout = rospy.Duration(60.0)
        start_time = rospy.Time.now()
        twist = Twist()

        while not rospy.is_shutdown():
            current_x, current_y, current_yaw = self.get_current_pose()
            if current_x is None:
                rospy.sleep(0.5)
                continue

            dx = target_x - current_x
            dy = target_y - current_y
            distance = math.sqrt(dx * dx + dy * dy)

            if (rospy.Time.now() - start_time) > timeout:
                rospy.logwarn("移动超时")
                break

            if distance < dist_tolerance:
                rospy.loginfo("到达目标位置！立即继续前往下一个目标点。")
                return True

            target_yaw = math.atan2(dy, dx)
            angle_diff = self.normalize_angle(target_yaw - current_yaw)

            if abs(angle_diff) > angle_tolerance:
                twist.linear.x = 0.1
                twist.angular.z = 0.5 if angle_diff > 0 else -0.5
                rospy.loginfo("调整方向: 当前=%.2f, 目标=%.2f, 差值=%.2f",
                              current_yaw, target_yaw, angle_diff)
            else:
                twist.linear.x = min(speed, distance)
                twist.angular.z = angle_diff * 0.5
                rospy.loginfo("向目标前进: 距离=%.2f米, 角度偏差=%.2f弧度",
                              distance, angle_diff)

            self.cmd_vel_pub.publish(twist)
            rate.sleep()

        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        rospy.loginfo("直接控制移动完成")
        return True

    def normalize_angle(self, angle):
        """将角度规范化到 [-pi, pi] 范围内"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def go_to_position(self, x, y, yaw=None):
        """带重试机制的导航，无法使用 move_base 时采用直接控制"""
        if self.move_base_client is None:
            return self.direct_control_to_position(x, y)

        base_goal = self.create_goal(x, y, yaw)
        for attempt in range(1, self.max_attempts + 1):
            goal = self.create_goal(
                x + (random.random() - 0.5) * 0.5 * (attempt - 1),
                y + (random.random() - 0.5) * 0.5 * (attempt - 1),
                yaw
            )

            rospy.loginfo("Attempt %d/%d: Navigating to position: x=%f, y=%f",
                          attempt, self.max_attempts, goal.target_pose.pose.position.x,
                          goal.target_pose.pose.position.y)

            self.try_clear_costmaps()

            try:
                self.move_base_client.send_goal(goal)
                finished_within_time = self.move_base_client.wait_for_result(rospy.Duration(60.0))
                if not finished_within_time:
                    rospy.logwarn("Navigation timed out, canceling goal")
                    self.move_base_client.cancel_goal()
                    rospy.sleep(1.0)
                    continue

                state = self.move_base_client.get_state()
                if state == actionlib.GoalStatus.SUCCEEDED:
                    rospy.loginfo("Reached the goal position")
                    return True
                else:
                    status_text = {
                        actionlib.GoalStatus.PENDING: "PENDING",
                        actionlib.GoalStatus.ACTIVE: "ACTIVE",
                        actionlib.GoalStatus.PREEMPTED: "PREEMPTED",
                        actionlib.GoalStatus.SUCCEEDED: "SUCCEEDED",
                        actionlib.GoalStatus.ABORTED: "ABORTED",
                        actionlib.GoalStatus.REJECTED: "REJECTED",
                        actionlib.GoalStatus.PREEMPTING: "PREEMPTING",
                        actionlib.GoalStatus.RECALLING: "RECALLING",
                        actionlib.GoalStatus.RECALLED: "RECALLED",
                        actionlib.GoalStatus.LOST: "LOST"
                    }
                    rospy.logwarn("Failed to reach the goal position. Status: %s",
                                  status_text.get(state, f"Unknown({state})"))
            except Exception as e:
                rospy.logwarn("Error during navigation: %s", str(e))

            if attempt < self.max_attempts:
                rospy.sleep(2.0)

        rospy.logwarn("All move_base navigation attempts failed, trying direct control")
        return self.direct_control_to_position(x, y)

    def statistics_callback(self, msg):
        """解析 /cube_tracker/statistics 消息，提取 min_digit"""
        if self.active:
            try:
                data = json.loads(msg.data)
                if "min_digit" in data:
                    self.min_digit = data["min_digit"]
                    rospy.loginfo("获得总数最少的数字 min_digit: %s", str(self.min_digit))
                    self.statistics_sub.unregister()
            except Exception as e:
                rospy.logerr("解析 /cube_tracker/statistics JSON 失败: %s", str(e))

    def target_cube_callback(self, msg):
        """解析目标立方体位置数据"""
        try:
            data = json.loads(msg.data)
            if "position" in data:
                self.target_cube_position = data["position"]
                rospy.loginfo("收到目标立方体位置: x=%.2f, y=%.2f",
                              self.target_cube_position["x"], self.target_cube_position["y"])
        except Exception as e:
            rospy.logerr("解析目标立方体位置JSON失败: %s", str(e))

    def navigate_to_boxes(self):
        """依次访问4个固定箱子位置，然后前往目标立方体"""
        visited_boxes = []

        for box_id in sorted(self.box_positions.keys()):
            x, y = self.box_positions[box_id]
            if self.go_to_position(x, y):
                rospy.loginfo("到达箱子 %d", box_id)
                visited_boxes.append(box_id)
                rospy.loginfo("箱子 %d 巡视完成，继续移动", box_id)
                rospy.sleep(1.0)
            else:
                rospy.logerr("多次尝试后仍未能到达箱子 %d。尝试下一个箱子。", box_id)

        if len(visited_boxes) < len(self.box_positions):
            rospy.logwarn("未能访问所有盒子位置，但将继续执行")

        rospy.loginfo("已完成所有盒子的巡视")

        max_wait = 20  # 最长等待目标立方体位置时间（秒）
        start_time = rospy.Time.now()
        while self.target_cube_position is None and (rospy.Time.now() - start_time) < rospy.Duration(max_wait):
            rospy.loginfo("等待目标立方体位置... (min_digit = %s)", str(self.min_digit))
            rospy.sleep(1.0)

        if self.target_cube_position is not None:
            target_x = self.target_cube_position["x"]
            target_y = self.target_cube_position["y"]
            rospy.loginfo("前往检测到的目标立方体: x=%.2f, y=%.2f", target_x, target_y)

            if self.go_to_position(target_x + 2.5, target_y):
                rospy.loginfo("成功到达目标立方体前沿")
            else:
                rospy.logerr("无法到达目标立方体前沿")
                return False

            if self.go_to_position(target_x + 0.5, target_y):
                rospy.loginfo("成功到达目标立方体位置")
                return True
            else:
                rospy.logerr("无法到达目标立方体位置")
                return False
        else:
            rospy.logerr("未能检测到目标立方体位置")
            return False


def main():
    rospy.init_node('end_navigation')
    rospy.loginfo("等待 /count_over=True 以进行初始化...")
    count_over_msg = rospy.wait_for_message('/count_over', Bool)
    if count_over_msg.data:
        rospy.loginfo("接收到 /count_over=True，进行初始化")
        navigator = BoxNavigation()
        rospy.spin()
    else:
        rospy.loginfo("接收到 /count_over 为 False，不进行任何操作")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("导航节点已终止。")
    except Exception as e:
        rospy.logerr("意外错误: %s", str(e))
