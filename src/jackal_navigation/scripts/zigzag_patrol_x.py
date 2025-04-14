#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.srv import GetPlan
from std_msgs.msg import Bool
from dynamic_reconfigure.client import Client
from tf.transformations import quaternion_from_euler 
from actionlib_msgs.msg import GoalStatusArray
from actionlib_msgs.msg import GoalID
import tf
from nav_msgs.msg import OccupancyGrid
import numpy as np

class ZigzagPatroller:
    def __init__(self):
        rospy.init_node("zigzag_patroller")

        # 目标点发布
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        self.count_over_pub = rospy.Publisher("/count_over", Bool, queue_size=10)
        
        # self.vel_client = Client("/move_base/TrajectoryPlannerROS")
        self.vel_client = Client("/move_base/TebLocalPlannerROS")

        # 小车当前位置
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_callback)
        self.current_pose = None

        # 等待路径规划服务
        rospy.loginfo("Waiting for /move_base/make_plan service...")
        rospy.wait_for_service("/move_base/make_plan")
        rospy.loginfo("Service /move_base/make_plan is available.")
        self.make_plan = rospy.ServiceProxy("/move_base/make_plan", GetPlan)

        self.move_base_status = None
        rospy.Subscriber("/move_base/status", GoalStatusArray, self.status_callback)

        # 巡逻参数
        self.step_x = 4.4
        self.start_x = 19.3
        self.end_x = 10
        self.top_y = -4
        self.bottom_y = -21.5

        self.start_goal = (21.5, -21.5)
        self.end_goal = (10.5, -2.5)

        self.direction = 1
        self.yaw = 0.5 * math.pi * self.direction

        self.costmap = None
        self.tf_listener = tf.TransformListener()
        rospy.Subscriber("/move_base/local_costmap/costmap", OccupancyGrid, self.costmap_callback)

    def amcl_callback(self, msg):
        self.current_pose = msg.pose.pose

    def status_callback(self, msg):
        self.move_base_status = msg.status_list

    def costmap_callback(self, msg):
        self.costmap = msg


    def distance(self, p1, p2):
        return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

    def create_pose_stamped(self, x, y, yaw=math.pi):
        """
        创建一个 PoseStamped 消息，其中目标位置 (x,y) 的朝向由 yaw 指定。
        默认 yaw = math.pi 表示目标朝向与 x 轴正方向相反（旋转180度）。
        """
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = x
        pose.pose.position.y = y

        # 将欧拉角 (roll=0, pitch=0, yaw) 转换为四元数
        q = quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        return pose

    def patrol(self):

        rospy.loginfo(f"🚩 Navigating to start point: {self.start_goal}")
        self.goal_pub.publish(self.create_pose_stamped(self.start_goal[0], self.start_goal[1]))
        # 等待小车到达 start_goal
        while not rospy.is_shutdown():
            if self.current_pose:
                dist = self.distance((self.current_pose.position.x, self.current_pose.position.y), self.start_goal)
                # rospy.loginfo(f"🧠dist:{dist}")
                if dist < 0.5:
                    rospy.loginfo("✅ Reached start point. Begin zigzag patrol.")
                    break
            rospy.sleep(0.5)

        # 修改 local costmap 的 inflation_radius
        inflation_radius = 0.73
        cost_scaling_factor = 1.5
        local_client = Client("/move_base/local_costmap/inflation_layer", timeout=5)
        local_client.update_configuration({"inflation_radius": inflation_radius})
        local_client.update_configuration({"cost_scaling_factor": cost_scaling_factor})
        rospy.loginfo(f"✅ local_costmap inflation_radius 设置为 {inflation_radius}")

        # 修改 global costmap 的 inflation_radius
        global_client = Client("/move_base/global_costmap/inflation_layer", timeout=5)
        global_client.update_configuration({"inflation_radius": inflation_radius})
        global_client.update_configuration({"cost_scaling_factor": cost_scaling_factor})
        rospy.loginfo(f"✅ global_costmap inflation_radius 设置为 {inflation_radius}")

        # 创建 dynamic_reconfigure client 连接 GlobalPlanner（带异常处理）
        try:
            planner_client = Client("/move_base/GlobalPlanner", timeout=5)
            planner_client.update_configuration({"cost_factor": 1})
            rospy.loginfo("✅ GlobalPlanner cost_factor 设置为 1")
        except Exception as e:
            rospy.logwarn(f"⚠️ GlobalPlanner 参数设置失败，跳过：{e}")

        rospy.sleep(1)
        self.vel_client.update_configuration({
            "max_vel_x": 2,
            "max_vel_theta": 0.5
        })
        
        current_x = self.start_x
        T = 4   # 到达目标的时间限制
        turn = 1
        
        while current_x >= self.end_x:
            if self.direction == 1:
                rospy.loginfo("right to left")
                ys = list(np.linspace(self.bottom_y, self.top_y, num=7))  # 插入 5 个点（含首尾）
            else :
                rospy.loginfo("left to right")
                ys = list(np.linspace(self.top_y, self.bottom_y, num=7))

            for y in ys:
                goal = (current_x, y)
                rospy.loginfo(f"🧭 正在处理目标点: x={goal[0]:.2f}, y={goal[1]:.2f}")
                
                if turn == 1:
                    self.yaw = 0.75 * math.pi 
                elif turn == 2:
                    self.yaw = -0.75 * math.pi 
                elif turn == 3:
                    self.yaw = 0.25 * math.pi 
                goal_pose = self.create_pose_stamped(goal[0], goal[1], self.yaw)

                # ✅ 发布目标点
                rospy.loginfo(f"📍 发布目标点 {goal}")
                self.goal_pub.publish(goal_pose)

                # ✅ 等待实际接近目标点
                skip_current_target = False
                start_time = rospy.Time.now()

                while not rospy.is_shutdown():
                    # if self.move_base_status:
                    #     latest_status = self.move_base_status[-1].status
                    #     status_map = {
                    #         0: "🕓 PENDING - 任务等待中",
                    #         1: "🚀 ACTIVE - 正在执行中",
                    #         2: "⛔ PREEMPTED - 被抢占中止",
                    #         3: "✅ SUCCEEDED - 成功完成",
                    #         4: "❌ ABORTED - 执行失败",
                    #         5: "🚫 REJECTED - 被拒绝执行",
                    #         6: "🚧 PREEMPTING - 正在处理中断",
                    #         7: "🔄 RECALLING - 正在回滚或取消",
                    #         8: "✅ RECALLED - 成功取消",
                    #         9: "🌀 LOST - 状态丢失/通信异常"
                    #     }
                    #     rospy.loginfo(f"📣 move_base 当前状态码: {latest_status} - {status_map.get(latest_status, '未知状态')}")
                    if self.current_pose:
                        dist = self.distance(
                            (self.current_pose.position.x, self.current_pose.position.y), goal
                        )
                        rospy.loginfo(f"📏 当前距离目标点: {dist:.2f} 米")

                        if dist < 1.0:
                            rospy.loginfo("✅ 成功到达目标点")
                            break  # 成功

                    duration = (rospy.Time.now() - start_time).to_sec()
                    movebase_failed = False
                    if self.move_base_status and self.move_base_status[-1].status in [4, 5]:
                        movebase_failed = True

                    if duration > T or movebase_failed:
                        reason = "⏱️ 超时" if duration > T else "❌ move_base 规划失败"
                        rospy.logwarn(f"{reason}，跳过目标点")
                        T += 2  # 增加容错时间
                        skip_current_target = True
                        break

                    rospy.sleep(0.5)

                if skip_current_target:
                    continue
                
            current_x -= self.step_x
            self.direction *= -1
            turn += 1

        # 最终目标点
        rospy.loginfo("🎯 Navigating to final goal.")
        self.goal_pub.publish(self.create_pose_stamped(self.end_goal[0], self.end_goal[1], 0))
        # 等待小车到达 end_goal
        while not rospy.is_shutdown():
            if self.current_pose:
                dist = self.distance((self.current_pose.position.x, self.current_pose.position.y), self.end_goal)
                # rospy.loginfo(f"🧠dist:{dist}")
                if dist < 0.5:
                    rospy.loginfo("✅ Reached end point.")
                    break
        rospy.sleep(1.0)  # 稍微等一下，确保目标发布生效
        # 修改 local costmap 的 inflation_radius
        inflation_radius = 0.15
        cost_scaling_factor = 10
        local_client = Client("/move_base/local_costmap/inflation_layer", timeout=5)
        local_client.update_configuration({"inflation_radius": inflation_radius})
        local_client.update_configuration({"cost_scaling_factor": cost_scaling_factor})
        rospy.loginfo(f"✅ local_costmap inflation_radius 设置为 {inflation_radius}")

        # 修改 global costmap 的 inflation_radius
        global_client = Client("/move_base/global_costmap/inflation_layer", timeout=5)
        global_client.update_configuration({"inflation_radius": inflation_radius})
        global_client.update_configuration({"cost_scaling_factor": cost_scaling_factor})
        rospy.loginfo(f"✅ global_costmap inflation_radius 设置为 {inflation_radius}")

        # 创建 dynamic_reconfigure client 连接 GlobalPlanner（带异常处理）
        try:
            planner_client = Client("/move_base/GlobalPlanner", timeout=5)
            planner_client.update_configuration({"cost_factor": 0.55})
            rospy.loginfo("✅ GlobalPlanner cost_factor 设置为 0.55")
        except Exception as e:
            rospy.logwarn(f"⚠️ GlobalPlanner 参数设置失败，跳过：{e}")
        
        rospy.sleep(1.0)

        self.count_over_pub.publish(Bool(data=True))
        rospy.loginfo("✅ /count_over published with data=True")

if __name__ == "__main__":
    try:
        patroller = ZigzagPatroller()
        patroller.patrol()
    except rospy.ROSInterruptException:
        pass