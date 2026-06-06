#!/usr/bin/env python3
"""
DWA局部避障节点 - 动态窗口法
订阅: /scan (sensor_msgs/LaserScan), /global_plan (nav_msgs/Path), /odom (nav_msgs/Odometry)
发布: /cmd_vel (geometry_msgs/Twist)
"""

import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist


class DWAPlanner:
    def __init__(self):
        rospy.init_node('dwa_planner_node')

        # 船舶运动学约束
        self.max_linear_vel = rospy.get_param('~max_linear_vel', 2.0)       # m/s
        self.min_linear_vel = rospy.get_param('~min_linear_vel', -0.5)
        self.max_angular_vel = rospy.get_param('~max_angular_vel', 0.8)     # rad/s
        self.max_linear_accel = rospy.get_param('~max_linear_accel', 0.5)   # m/s^2
        self.max_angular_accel = rospy.get_param('~max_angular_accel', 0.4) # rad/s^2

        # DWA参数
        self.predict_time = rospy.get_param('~predict_time', 2.0)          # 前向仿真时间
        self.dt = rospy.get_param('~dt', 0.1)                               # 仿真步长
        self.linear_samples = rospy.get_param('~linear_samples', 10)        # 线速度采样
        self.angular_samples = rospy.get_param('~angular_samples', 20)      # 角速度采样

        # 评价函数权重
        self.goal_weight = rospy.get_param('~goal_weight', 1.0)             # 目标方向
        self.obstacle_weight = rospy.get_param('~obstacle_weight', 2.0)     # 障碍物距离
        self.speed_weight = rospy.get_param('~speed_weight', 0.3)           # 速度
        self.safety_dist = rospy.get_param('~safety_dist', 2.0)             # 安全距离

        # 状态
        self.current_vel = (0.0, 0.0)  # (v, w)
        self.current_pose = (0.0, 0.0, 0.0)  # (x, y, theta)
        self.obstacles = []  # [(x, y), ...]
        self.global_path = []
        self.goal_point = None

        # 发布器
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # 订阅器
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/global_plan', Path, self.path_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # 控制循环
        self.control_timer = rospy.Timer(rospy.Duration(0.1), self.control_loop)

        rospy.loginfo("DWA Planner started")

    def scan_callback(self, msg):
        """激光雷达数据回调 - 提取障碍物位置"""
        self.obstacles = []
        angle = msg.angle_min
        for i, r in enumerate(msg.ranges):
            if msg.range_min < r < msg.range_max:
                # 转换到世界坐标
                wx = self.current_pose[0] + r * math.cos(self.current_pose[2] + angle)
                wy = self.current_pose[1] + r * math.sin(self.current_pose[2] + angle)
                self.obstacles.append((wx, wy))
            angle += msg.angle_increment

    def path_callback(self, msg):
        """全局路径回调"""
        self.global_path = []
        for pose in msg.poses:
            self.global_path.append((pose.pose.position.x, pose.pose.position.y))
        if self.global_path:
            self.goal_point = self.global_path[-1]

    def odom_callback(self, msg):
        """里程计回调"""
        self.current_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            self.quaternion_to_yaw(msg.pose.pose.orientation)
        )
        self.current_vel = (
            msg.twist.twist.linear.x,
            msg.twist.twist.angular.z
        )

    def quaternion_to_yaw(self, q):
        """四元数转偏航角"""
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    def control_loop(self, event):
        """DWA控制循环"""
        if not self.goal_point:
            return

        # 检查是否到达目标
        dist_to_goal = math.hypot(
            self.current_pose[0] - self.goal_point[0],
            self.current_pose[1] - self.goal_point[1]
        )
        if dist_to_goal < 1.0:
            self.stop()
            return

        # 动态窗口
        dw = self.calc_dynamic_window()

        # 评估所有候选速度
        best_score = -float('inf')
        best_cmd = (0.0, 0.0)

        for v in np.linspace(dw[0], dw[1], self.linear_samples):
            for w in np.linspace(dw[2], dw[3], self.angular_samples):
                # 轨迹预测
                traj = self.predict_trajectory(v, w)
                # 评价
                score = self.evaluate(v, w, traj)
                if score > best_score:
                    best_score = score
                    best_cmd = (v, w)

        # 发布速度指令
        self.publish_cmd(best_cmd[0], best_cmd[1])

    def calc_dynamic_window(self):
        """计算动态窗口: 速度限制 + 加速度限制"""
        v_min = max(self.min_linear_vel,
                    self.current_vel[0] - self.max_linear_accel * self.dt)
        v_max = min(self.max_linear_vel,
                    self.current_vel[0] + self.max_linear_accel * self.dt)
        w_min = max(-self.max_angular_vel,
                    self.current_vel[1] - self.max_angular_accel * self.dt)
        w_max = min(self.max_angular_vel,
                    self.current_vel[1] + self.max_angular_accel * self.dt)
        return (v_min, v_max, w_min, w_max)

    def predict_trajectory(self, v, w):
        """预测轨迹"""
        traj = []
        x, y, theta = self.current_pose
        t = 0.0
        while t < self.predict_time:
            x += v * math.cos(theta) * self.dt
            y += v * math.sin(theta) * self.dt
            theta += w * self.dt
            traj.append((x, y, theta))
            t += self.dt
        return traj

    def evaluate(self, v, w, traj):
        """评价函数"""
        if not traj:
            return -float('inf')

        # 1. 目标方向得分
        goal_score = self.goal_score(traj[-1])

        # 2. 障碍物距离得分
        obs_score = self.obstacle_score(traj)

        # 3. 速度得分
        speed_score = self.speed_score(v)

        return (self.goal_weight * goal_score +
                self.obstacle_weight * obs_score +
                self.speed_weight * speed_score)

    def goal_score(self, final_pose):
        """目标方向得分: 终点离目标越近越好"""
        if not self.goal_point:
            return 0.0
        dist = math.hypot(final_pose[0] - self.goal_point[0],
                          final_pose[1] - self.goal_point[1])
        return 1.0 / (1.0 + dist)

    def obstacle_score(self, traj):
        """障碍物距离得分: 轨迹离障碍物越远越好"""
        if not self.obstacles:
            return 1.0

        min_dist = float('inf')
        for pose in traj:
            for obs in self.obstacles:
                dist = math.hypot(pose[0] - obs[0], pose[1] - obs[1])
                if dist < min_dist:
                    min_dist = dist

        # 安全距离内扣分
        if min_dist < self.safety_dist:
            return min_dist / self.safety_dist
        return 1.0

    def speed_score(self, v):
        """速度得分: 鼓励高速"""
        if self.max_linear_vel == 0:
            return 0.0
        return v / self.max_linear_vel

    def publish_cmd(self, v, w):
        """发布速度指令"""
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)

    def stop(self):
        """停止"""
        self.publish_cmd(0.0, 0.0)


if __name__ == '__main__':
    try:
        planner = DWAPlanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass