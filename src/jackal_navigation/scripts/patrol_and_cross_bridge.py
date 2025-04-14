#!/usr/bin/env python3
import rospy
import actionlib
import json
import math
from math import sqrt
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf2_ros
from dynamic_reconfigure.client import Client
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Twist
import time
from tf.transformations import euler_from_quaternion  # 新增导入，便于将四元数转换为欧拉角

pi = 3.14159

WAYPOINTS = [
    (10.5, -21.0),
    (10.5, -12.0),
    (10.5, -3.0)
]

# 控制参数（可根据需要调整）
K_linear = 1     # 线速度比例增益
K_angular = 0.1    # 角速度比例增益
# ANGLE_THRESHOLD = 0.1  # 定义一个角度判定阈值（单位：弧度）
DIST_THRESHOLD = 0.4  # 距离判定阈值

class BridgePatrolAndResponder:
    def __init__(self):
        rospy.init_node("bridge_patrol_and_responder")

        # 初始化 move_base client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("⏳ Waiting for move_base server...")
        self.client.wait_for_server()
        rospy.loginfo("✅ Connected to move_base")

        self.robot_pose = None
        self.bridge_entry = None
        self.bridge_exit = None
        self.bridge_triggered = False
        self.current_goal = None

        self.start_patrol_flag = False
        rospy.Subscriber('/count_over', Bool, self.count_over_callback)

        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.pose_callback)
        rospy.Subscriber("/bridge_detector/bridge_head", String, self.json_callback)

        self.vel_client = Client("/move_base/TebLocalPlannerROS")
        # self.vel_client = Client("/move_base/TrajectoryPlannerROS")

        self.bridge_unlock_pub = rospy.Publisher("/cmd_open_bridge", Bool, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.crossed_bridge_pub = rospy.Publisher("/crossed_bridge", Bool, queue_size=1)

    def pose_callback(self, msg):
        self.robot_pose = msg.pose.pose

    def json_callback(self, msg):
        try:
            data = json.loads(msg.data)
            bridge_head = data.get("bridge_head", {})
            x = bridge_head.get("x")
            y = bridge_head.get("y")
            if x is not None and y is not None:
                self.bridge_entry = (x, y)
                self.bridge_exit = (x - 3, y)
                rospy.loginfo(f"✅ 收到桥头位置: ({x:.2f}, {y:.2f})")
                self.bridge_triggered = True
        except json.JSONDecodeError:
            rospy.logerr("❌ 无法解析 JSON 格式的数据")

    def count_over_callback(self, msg):
        if msg.data:
            rospy.loginfo("✅ /count_over received True — starting patrol.")
            self.start_patrol_flag = True

    def has_reached_goal(self):
        if self.robot_pose is None or self.current_goal is None:
            return False
        goal_x = self.current_goal.pose.position.x
        goal_y = self.current_goal.pose.position.y
        dx = self.robot_pose.position.x - goal_x
        dy = self.robot_pose.position.y - goal_y
        dist = sqrt(dx ** 2 + dy ** 2)
        # rospy.loginfo(f"x = {self.robot_pose.position.x:.2f}, y = {self.robot_pose.position.y:.2f}), dist = {dist:.2f} ")
        if dist < DIST_THRESHOLD :
            rospy.loginfo("reached goal")
            return True
        else :
            # rospy.loginfo("not reach goal yet")
            return False

    def has_reached_goal_1(self, goal_yaw=None):
        """
        判断机器人是否到达目标位置与目标朝向（如果指定），
        若距离误差或角度误差超出阈值，则分别发布速度指令引导机器人修正。

        :param goal_yaw: 目标朝向（弧度），默认None时仅考虑距离
        :return: 如果同时达到距离（和角度）要求，返回True；否则发布速度指令后返回False
        """
        # 检查必要数据
        if self.robot_pose is None or self.current_goal is None:
            return False

        # 提取目标位置信息
        goal_x = self.current_goal.pose.position.x
        goal_y = self.current_goal.pose.position.y

        # 当前机器人位置信息
        current_x = self.robot_pose.position.x
        current_y = self.robot_pose.position.y

        # 计算距离误差（单位：米）
        dx = goal_x - current_x
        dy = goal_y - current_y
        distance_error = sqrt(dx ** 2 + dy ** 2)

        # 从当前姿态（四元数）中提取 yaw 值
        orientation_q = self.robot_pose.orientation
        q = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, current_yaw) = euler_from_quaternion(q)

        # 根据是否传入目标角度决定目标朝向
        if goal_yaw is None:
            # 如果没有指定目标角度，则可以认为目标朝向为机器人从当前位置到目标位置的方向
            desired_yaw = math.atan2(dy, dx)
        else:
            desired_yaw = goal_yaw

        # 计算角度误差，并将其归一化到[-pi, pi]
        angle_error = desired_yaw - current_yaw
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        # 如果同时满足距离与（如适用）角度误差要求，则认为目标到达
        if distance_error < DIST_THRESHOLD and (goal_yaw is None or abs(angle_error) < ANGLE_THRESHOLD):
        # if goal_yaw is None or abs(angle_error) < ANGLE_THRESHOLD:
            rospy.loginfo("✅ reached goal with correct orientation")
            # 停止机器人运动（发布零速度）
            stop_msg = Twist()
            stop_msg.linear.x = 0.0
            stop_msg.angular.z = 0.0
            self.cmd_vel_pub.publish(stop_msg)
            return True

        # 未达标时，构建 Twist 消息进行修正
        twist_msg = Twist()

        # 如果距离误差较大，则发布线速度指令（优先考虑直线向目标前进）
        if distance_error >= DIST_THRESHOLD:
            # 如果角度误差较大，则建议先调整朝向，此时可以适当减小线速度
            if abs(angle_error) > ANGLE_THRESHOLD:
                twist_msg.linear.x = 0.0
            else:
                # 简单比例控制，速度与距离误差正比
                twist_msg.linear.x = K_linear * distance_error

        # 如果角度误差较大，则发布角速度指令
        if abs(angle_error) > ANGLE_THRESHOLD:
            twist_msg.angular.z = K_angular * angle_error
        else:
            twist_msg.angular.z = 0.0

        # 发布速度指令
        self.cmd_vel_pub.publish(twist_msg)
        # rospy.loginfo("⚠️ Not reached goal: distance error = {:.2f}, angle error = {:.2f}".format(distance_error, angle_error))
        return False

    def adjust_orientation(self, target_yaw, ANGLE_THRESHOLD):
        """
        检查机器人当前朝向与目标朝向之间的偏差，
        如果偏差超出阈值，则逐步发布旋转指令进行调整，直至达到目标角度。

        :param target_yaw: 目标朝向（弧度）
        """
        # rospy.loginfo("🔒 Disabling local planner by setting max_vel_x to 0")
        # # 关闭局部规划器：这里设置 max_vel_x 为0，确保局部规划器不会产生干扰的速度命令
        # try:
        #     rospy.sleep(1)
        #     self.vel_client.update_configuration({
        #         "max_vel_theta": 0
        #     })
        # except Exception as e:
        #     rospy.logerr("Failed to disable local planner: {}".format(e))


        # 如果当前位姿数据不存在，无法调整
        if self.robot_pose is None:
            rospy.logwarn("当前机器人位姿数据不可用，无法调整朝向。")
            return
        
        # 定义最小角速度阈值（可根据具体机器人性能调节）
        min_angular_speed = 0.1  # 单位：弧度/秒

        # 计算初始角度误差
        orientation_q = self.robot_pose.orientation
        q = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, current_yaw) = euler_from_quaternion(q)
        angle_error = target_yaw - current_yaw
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
        rospy.loginfo(f"🔍 Initial angle error: {angle_error:.2f} radians.")

        # 循环发布旋转指令直至角度误差在允许范围内
        while abs(angle_error) > ANGLE_THRESHOLD and not rospy.is_shutdown():
            twist_msg = Twist()
            twist_msg.linear.x = 0.0  # 不前进，仅旋转

            computed_angular_z = K_angular * angle_error  # 比例控制旋转速度

            # 加入最小输出限制：如果计算值太小，但误差仍然大于阈值，则使用最小角速度
            if abs(computed_angular_z) < min_angular_speed:
                twist_msg.angular.z = min_angular_speed if computed_angular_z >= 0 else -min_angular_speed
            else:
                twist_msg.angular.z = computed_angular_z

            self.cmd_vel_pub.publish(twist_msg)
            rospy.sleep(0.1)  # 等待运动反馈

            # 更新当前朝向
            orientation_q = self.robot_pose.orientation
            q = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            (roll, pitch, current_yaw) = euler_from_quaternion(q)
            angle_error = target_yaw - current_yaw
            angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
            rospy.loginfo(f"🔄 Updating angle error: {angle_error:.8f} radians.")

            rospy.sleep(0.1)

        # 校正完成后停止旋转
        twist_msg = Twist()
        twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_msg)
        rospy.loginfo("✅ Orientation adjusted to target angle.")

        # # 局部规划器：这里设置 max_vel_x 为0.5，确保局部规划器不会产生干扰的速度命令
        # try:
        #     rospy.sleep(1)
        #     self.vel_client.update_configuration({
        #         "max_vel_theta": 0.5
        #     })
        # except Exception as e:
        #     rospy.logerr("Failed to restore local planner: {}".format(e))



    def send_goal(self, x, y, yaw=0.0):
        """
        发送目标点，同时设置期望的偏航角（yaw）
        :param x: 目标点x坐标
        :param y: 目标点y坐标
        :param yaw: 期望偏航角，单位为弧度（默认0.0）
        """
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        # 使用tf转换，将yaw转换为四元数
        q = quaternion_from_euler(0, 0, yaw)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]
        self.current_goal = goal.target_pose
        rospy.loginfo(f"🚩 Navigating to ({x:.2f}, {y:.2f})")
        self.client.send_goal(goal)

    def go_across_bridge(self):
        if not self.bridge_entry or not self.bridge_exit:
            rospy.logwarn("❌ Bridge entry or exit not defined.")
            return
        
        self.vel_client.update_configuration({
            "xy_goal_tolerance": 0.3,
            "yaw_goal_tolerance": 0.2
        })
        rospy.logwarn("tolerance 阈值已调整")

        ex, ey = self.bridge_entry
        t0 = time.time()
        self.send_goal(ex - 0.5, ey, pi)
        rospy.loginfo("📏 Waiting to reach bridge entry...")
        while not rospy.is_shutdown():
            if self.has_reached_goal():
                rospy.sleep(2)
                break
        rospy.loginfo(f"✅ reached the bridge entry [{time.time() - t0} s].")

        # # 调用新封装的函数进行朝向校正（目标为 pi 弧度）
        # self.adjust_orientation(pi, 0.15)

        # # 靠近锥筒
        # t1 = time.time()
        # self.send_goal(ex - 2.2, ey, pi)
        # rospy.loginfo("📏 Waiting to reach unlock entry...")
        # while not rospy.is_shutdown():
        #     if self.has_reached_goal():
        #         rospy.sleep(2)
        #         break
        # rospy.loginfo(f"🎯🎯🎯🎯🎯🎯🎯🎯🎯 reached the unlock entry [{time.time() - t1} s].")

        # # 调用新封装的函数进行朝向校正（目标为 pi 弧度）
        # self.adjust_orientation(pi, 0.20)

        # rospy.sleep(2) 
        # self.client.cancel_all_goals()          


        # 靠近锥筒
        t1 = time.time()
        self.send_goal(ex - 2.3, ey, pi)
        rospy.loginfo("📏 Waiting to reach unlock entry...")
        self.client.wait_for_result()
        rospy.sleep(1)
        rospy.loginfo(f"✅ reached the unlock entry [{time.time() - t1} s].")

        t2 = time.time()

        # 直接进行加速直线行驶，不使用导航
        from geometry_msgs.msg import Twist  # Ensure Twist is imported

        # 设置直线行驶的参数
        twist_msg = Twist()
        twist_msg.linear.x = 4.0  # 加速直线速度
        twist_msg.angular.z = 0.0

        # 定义跨越桥面的行驶时长（根据桥长进行调整）
        bridge_cross_duration = 2.0  # 单位：秒

        rospy.loginfo("🔓 Start Unlocking the bridge...")
        self.bridge_unlock_pub.publish(Bool(data=True))
        rospy.sleep(1)
        rospy.loginfo(f"✅✅✅✅✅✅✅✅✅ Unlocked the bridge [ {time.time() - t2} s]")

        rospy.loginfo("🚀 开始直线行驶过桥...")
        start_time = rospy.Time.now().to_sec()

        while not rospy.is_shutdown() and (rospy.Time.now().to_sec() - start_time < bridge_cross_duration):
            self.cmd_vel_pub.publish(twist_msg)
            rospy.sleep(0.01)

        # # 提前发布 /crossed_bridge 信号
        # self.crossed_bridge_pub.publish(Bool(data=True))
        # rospy.loginfo("✅ 提前发布了 /crossed_bridge True")

        # 停止机器人
        twist_msg.linear.x = 0.0
        self.cmd_vel_pub.publish(twist_msg)
        rospy.loginfo("✅ Crossed the bridge via direct straight line driving.")

        # 持续发布 /crossed_bridge=True
        rate = rospy.Rate(1)  # 每秒发布一次
        while not rospy.is_shutdown():
            self.crossed_bridge_pub.publish(Bool(data=True))
            rate.sleep()

        # ox, oy = self.bridge_exit
        # self.send_goal(ox - 2, oy, pi)
        # # 桥上提速
        # self.vel_client.update_configuration({
        #     "max_vel_x": 3.0   # 加速直线速度
        # })
        # self.client.wait_for_result()
        # self.vel_client.update_configuration({
        #     "max_vel_x": 1.0
        # })
        # self.send_goal(ox-3, oy, pi)
        # rospy.loginfo("✅ Crossed the bridge.")

        # # 持续发布 /crossed_bridge=True
        # rate = rospy.Rate(1)  # 每秒发布一次
        # while not rospy.is_shutdown():
        #     self.crossed_bridge_pub.publish(Bool(data=True))
        #     rate.sleep()

    def patrol_loop(self):
        rospy.loginfo("❌ ❌ ❌ ❌ ❌ Waiting for /count_over to start patrol...")
        while not rospy.is_shutdown() and not self.start_patrol_flag:
            rospy.sleep(0.5)
        rospy.loginfo("🚗 Starting patrol...")
        
        index = 0
        forward = True
        rate = rospy.Rate(1)
        x, y = WAYPOINTS[index]
        self.send_goal(x, y, pi)
        while not rospy.is_shutdown():
            if self.bridge_triggered:
                self.go_across_bridge()
                rospy.loginfo("🎉 Bridge process finished. Stopping patrol.")
                break
            if self.has_reached_goal():
                index += 1 if forward else -1
                print("index =", index)
                if index >= len(WAYPOINTS) or index < 0:
                    forward = not forward
                    index = max(0, min(len(WAYPOINTS) - 1, index))
                x, y = WAYPOINTS[index]
                self.send_goal(x, y, pi)
            rate.sleep()

if __name__ == '__main__':
    try:
        navigator = BridgePatrolAndResponder()
        navigator.patrol_loop()
    except rospy.ROSInterruptException:
        pass