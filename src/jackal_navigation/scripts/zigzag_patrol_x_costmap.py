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
        self.step_x = 3.0
        self.start_x = 19.8
        self.end_x = 10
        self.top_y = -4
        self.bottom_y = -21.5

        self.start_goal = (21.5, -21.5)
        self.end_goal = (11.0, -3.0)

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
    
    def wait_until_move_base_idle(self, timeout=10):
        start_time = rospy.Time.now()
        cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)

        # 等待小车到达 start_goal
        while not rospy.is_shutdown():
            if self.current_pose:
                dist = self.distance((self.current_pose.position.x, self.current_pose.position.y), self.start_goal)
                if dist < 0.5:
                    rospy.loginfo("✅ Reached start point. Begin zigzag patrol.")
                    break
            rospy.sleep(0.5)

        while not rospy.is_shutdown():
            if self.move_base_status is None:
                rospy.sleep(0.2)
                continue

            # 检查是否空闲
            if len(self.move_base_status) == 0 or all(s.status in [3, 4, 5] for s in self.move_base_status):
                rospy.loginfo("✅ move_base 已空闲")
                return True

            # 超时：强制取消目标
            if (rospy.Time.now() - start_time).to_sec() > timeout:
                rospy.logwarn("⏱ 等待 move_base 空闲超时，正在取消当前目标...")
                cancel_pub.publish(GoalID())  # 强制取消当前目标
                return False

            rospy.sleep(0.2)


    def is_goal_in_obstacle(self, x, y):
        if self.costmap is None:
            rospy.logwarn("⚠️ 代价地图尚未接收到，无法判断")
            return False

        try:
            # 将目标点从 map 坐标系转换到 costmap 坐标系
            time = rospy.Time(0)
            self.tf_listener.waitForTransform(
                self.costmap.header.frame_id, "map", time, rospy.Duration(1.0)
            )

            point = tf.transformations.translation_matrix([x, y, 0.0])
            point_stamped = PoseStamped()
            point_stamped.header.frame_id = "map"
            point_stamped.header.stamp = time
            point_stamped.pose.position.x = x
            point_stamped.pose.position.y = y
            point_stamped.pose.position.z = 0.0
            point_stamped.pose.orientation.w = 1.0

            transformed = self.tf_listener.transformPose(
                self.costmap.header.frame_id, point_stamped
            )
            tx = transformed.pose.position.x
            ty = transformed.pose.position.y

            origin = self.costmap.info.origin.position
            resolution = self.costmap.info.resolution
            width = self.costmap.info.width
            height = self.costmap.info.height

            # 地图边界（世界坐标）
            map_x_min = origin.x
            map_y_min = origin.y
            map_x_max = origin.x + width * resolution
            map_y_max = origin.y + height * resolution

            epsilon = 1e-6  # 容差，避免浮点数精度误判

            if not (map_x_min <= tx < map_x_max - epsilon and map_y_min <= ty < map_y_max - epsilon):
                rospy.logwarn("❌ 目标点超出代价地图边界")
                return True 

            i = int((tx - origin.x) / resolution)
            j = int((ty - origin.y) / resolution)

            if not (0 <= i < self.costmap.info.width and 0 <= j < self.costmap.info.height):
                rospy.logwarn("⚠️ Grid 索引越界（理论上不应发生），跳过该点")
                return True

            index = j * self.costmap.info.width + i
            cost = self.costmap.data[index]

            # 调整判断阈值
            if cost >= 100:
                rospy.logwarn(f"🚧 目标点代价为 {cost}，视为障碍")
                return True

        except (tf.Exception, tf.LookupException, tf.ConnectivityException) as e:
            rospy.logwarn("⚠️ TF 坐标转换失败：" + str(e))
            return False



    def patrol(self):

        rospy.loginfo(f"🚩 Navigating to start point: {self.start_goal}")
        self.goal_pub.publish(self.create_pose_stamped(self.start_goal[0], self.start_goal[1]))
        # # 等待小车到达 start_goal
        # while not rospy.is_shutdown():
        #     if self.current_pose:
        #         dist = self.distance((self.current_pose.position.x, self.current_pose.position.y), self.start_goal)
        #         # rospy.loginfo(f"🧠dist:{dist}")
        #         if dist < 0.5:
        #             rospy.loginfo("✅ Reached start point. Begin zigzag patrol.")
        #             break
        #     rospy.sleep(0.5)

        self.wait_until_move_base_idle(timeout=15)

        # 修改 local costmap 的 inflation_radius
        inflation_radius = 0.6
        cost_scaling_factor = 2.5
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
            "max_vel_x": 1,
            "max_vel_theta": 0.3
        })
        
        current_x = self.start_x
        T = 7   # 到达目标的时间限制
        
        while current_x >= self.end_x:
            if self.direction == 1:
                rospy.loginfo("right to left")
                ys = list(np.linspace(self.bottom_y, self.top_y, num=8))  # 插入 5 个点（含首尾）
            else :
                rospy.loginfo("left to right")
                ys = list(np.linspace(self.top_y, self.bottom_y, num=8))

            for y in ys:
                goal = (current_x, y)
                rospy.loginfo(f"🧭 正在处理目标点: x={goal[0]:.2f}, y={goal[1]:.2f}")
                if self.is_goal_in_obstacle(goal[0], goal[1]):
                    rospy.logwarn("🚧 目标点在障碍或未知区域，跳过")
                    continue
                
                self.yaw = 0.5 * math.pi * self.direction
                goal_pose = self.create_pose_stamped(goal[0], goal[1], self.yaw)

                # ✅ 发布目标点
                rospy.loginfo(f"📍 发布目标点 {goal}")
                self.goal_pub.publish(goal_pose)

                # ✅ 等待实际接近目标点
                skip_current_target = False
                start_time = rospy.Time.now()

                while not rospy.is_shutdown():
                    if self.current_pose:
                        dist = self.distance(
                            (self.current_pose.position.x, self.current_pose.position.y), goal
                        )
                        rospy.loginfo(f"📏 当前距离目标点: {dist:.2f} 米")

                        if dist < 1.0:
                            rospy.loginfo("✅ 成功到达目标点")
                            break  # 成功
                    
                    # 🚧 检查目标点是否变成了障碍
                    if self.is_goal_in_obstacle(goal[0], goal[1]):
                        rospy.logwarn("🚧 目标点导航途中变为障碍，取消当前目标")
                        skip_current_target = True
                        break

                    if (rospy.Time.now() - start_time).to_sec() > T:
                        T += 3
                        rospy.logwarn(f"❌ 超时未到达目标点，跳过，还剩{T}秒到达下一目标点")
                        skip_current_target = True
                        break

                    rospy.sleep(0.5)

                if skip_current_target:
                    continue
                
            current_x -= self.step_x
            self.direction *= -1

        # 最终目标点
        rospy.loginfo("🎯 Navigating to final goal.")
        self.goal_pub.publish(self.create_pose_stamped(self.end_goal[0], self.end_goal[1]))
        rospy.sleep(1.0)  # 稍微等一下，确保目标发布生效
        self.count_over_pub.publish(Bool(data=True))
        rospy.loginfo("✅ /count_over published with data=True")

if __name__ == "__main__":
    try:
        patroller = ZigzagPatroller()
        patroller.patrol()
    except rospy.ROSInterruptException:
        pass