#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
import math

def publish_goal():
    rospy.init_node('auto_goal_publisher')
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.sleep(1.0)  # wait for publisher to register
    rospy.wait_for_message("/clock", rospy.AnyMsg)  # 仿真时钟准备好

    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.header.stamp = rospy.Time.now()

    # 修改以下位置为你目标点的位置
    goal.pose.position.x = 19.7
    goal.pose.position.y = -22.2
    goal.pose.position.z = 0.0

    # 设置朝向（四元数），此处为面向 x 方向
    goal.pose.orientation.x = 0.0
    goal.pose.orientation.y = 0.0
    goal.pose.orientation.z = 0.0
    goal.pose.orientation.w = 1.0

    rospy.loginfo("Publishing goal to /move_base_simple/goal")
    for _ in range(3):  # 发布多次防止丢包
        pub.publish(goal)
        rospy.sleep(0.5)

if __name__ == '__main__':
    try:
        publish_goal()
    except rospy.ROSInterruptException:
        pass
