#!/usr/bin/env python3
"""
实验运行器 - 自动发送目标点并监控实验进度
支持四类实验: straight, obstacles, dynamic, narrow
"""

import rospy
import math
import time
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class ExperimentRunner:
    def __init__(self):
        rospy.init_node('experiment_runner')

        self.experiment_type = rospy.get_param('~experiment_type', 'straight')
        self.goal_x = rospy.get_param('~goal_x', 40.0)
        self.goal_y = rospy.get_param('~goal_y', 0.0)
        self.timeout = rospy.get_param('~timeout', 300)  # 秒

        self.current_pose = (0.0, 0.0, 0.0)
        self.start_time = None
        self.goal_reached = False
        self.collision_detected = False

        # 发布器
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.result_pub = rospy.Publisher('/experiment_result', String, queue_size=1)

        # 订阅器
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        rospy.loginfo(f"ExperimentRunner: type={self.experiment_type}, "
                      f"goal=({self.goal_x},{self.goal_y}), timeout={self.timeout}s")

        # 等待仿真就绪
        rospy.sleep(3.0)

        # 发送目标
        self.send_goal()

        # 监控循环
        self.monitor_loop()

    def odom_callback(self, msg):
        self.current_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            0.0
        )

    def send_goal(self):
        """发送目标点"""
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = self.goal_x
        goal.pose.position.y = self.goal_y
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0

        self.goal_pub.publish(goal)
        self.start_time = rospy.Time.now()
        rospy.loginfo(f"Goal sent: ({self.goal_x}, {self.goal_y})")

    def monitor_loop(self):
        """监控实验进度"""
        rate = rospy.Rate(10)  # 10Hz
        last_progress = 0.0

        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - self.start_time).to_sec()

            # 检查是否到达目标
            dist = math.hypot(
                self.current_pose[0] - self.goal_x,
                self.current_pose[1] - self.goal_y
            )

            if dist < 1.0:
                self.goal_reached = True
                rospy.loginfo(f"GOAL REACHED! Time: {elapsed:.1f}s")
                self.publish_result(elapsed, "success")
                break

            # 检查超时
            if elapsed > self.timeout:
                rospy.logwarn(f"TIMEOUT! Elapsed: {elapsed:.1f}s, "
                              f"remaining distance: {dist:.1f}m")
                self.publish_result(elapsed, "timeout")
                break

            # 进度日志
            progress = self.goal_x - dist if self.goal_x > 0 else dist
            if progress - last_progress > 5.0:
                rospy.loginfo(f"Progress: {progress:.1f}m / {self.goal_x:.1f}m "
                              f"({elapsed:.1f}s)")
                last_progress = progress

            rate.sleep()

        rospy.signal_shutdown("Experiment complete")

    def publish_result(self, elapsed, status):
        """发布实验结果"""
        result = (f"experiment: {self.experiment_type}, "
                  f"status: {status}, "
                  f"time: {elapsed:.1f}s, "
                  f"goal: ({self.goal_x}, {self.goal_y})")
        self.result_pub.publish(result)
        rospy.loginfo(f"Result: {result}")


if __name__ == '__main__':
    try:
        runner = ExperimentRunner()
    except rospy.ROSInterruptException:
        pass