#!/usr/bin/env python3
"""
纯追踪轨迹跟踪节点 - Pure Pursuit
订阅: /global_plan (nav_msgs/Path), /odom (nav_msgs/Odometry)
发布: /cmd_vel (geometry_msgs/Twist)
"""

import rospy
import math
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist


class PurePursuitController:
    def __init__(self):
        rospy.init_node('pure_pursuit_node')

        # 参数
        self.fixed_lookahead = rospy.get_param('~fixed_lookahead', 5.0)       # 固定前视距离(m)
        self.adaptive_lookahead = rospy.get_param('~adaptive_lookahead', True) # 是否自适应
        self.min_lookahead = rospy.get_param('~min_lookahead', 3.0)
        self.max_lookahead = rospy.get_param('~max_lookahead', 10.0)
        self.lookahead_gain = rospy.get_param('~lookahead_gain', 2.0)          # 速度增益
        self.wheelbase = rospy.get_param('~wheelbase', 1.2)                    # 轴距
        self.max_linear_vel = rospy.get_param('~max_linear_vel', 2.0)
        self.max_angular_vel = rospy.get_param('~max_angular_vel', 0.8)
        self.linear_kp = rospy.get_param('~linear_kp', 0.5)                    # 线速度比例系数
        self.goal_tolerance = rospy.get_param('~goal_tolerance', 1.0)          # 目标容忍距离

        # 状态
        self.path = []
        self.current_pose = (0.0, 0.0, 0.0)
        self.current_vel = (0.0, 0.0)

        # 发布器
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # 订阅器
        rospy.Subscriber('/global_plan', Path, self.path_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # 控制循环 (20Hz)
        self.control_timer = rospy.Timer(rospy.Duration(0.05), self.control_loop)

        rospy.loginfo("PurePursuit Controller started")

    def path_callback(self, msg):
        """路径回调"""
        self.path = []
        for pose in msg.poses:
            self.path.append((pose.pose.position.x, pose.pose.position.y))

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
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    def control_loop(self, event):
        """纯追踪控制循环"""
        if not self.path:
            return

        x, y, theta = self.current_pose
        v = self.current_vel[0]

        # 1. 计算前视距离
        if self.adaptive_lookahead:
            lookahead = self.min_lookahead + self.lookahead_gain * abs(v)
            lookahead = min(max(lookahead, self.min_lookahead), self.max_lookahead)
        else:
            lookahead = self.fixed_lookahead

        # 2. 找到最近路径点
        nearest_idx = self.find_nearest_point(x, y)
        if nearest_idx is None:
            return

        # 3. 找到前瞻点
        lookahead_point = self.find_lookahead_point(x, y, lookahead, nearest_idx)
        if lookahead_point is None:
            # 无前瞻点，检查是否到达终点
            end_dist = math.hypot(x - self.path[-1][0], y - self.path[-1][1])
            if end_dist < self.goal_tolerance:
                self.stop()
            return

        # 4. 计算曲率
        curvature = self.calc_curvature(x, y, theta, lookahead_point, lookahead)

        # 5. 计算控制指令
        angular_vel = curvature * v  # w = kappa * v
        angular_vel = max(-self.max_angular_vel, min(self.max_angular_vel, angular_vel))

        # 线速度: 根据路径曲率调节
        target_speed = self.max_linear_vel
        if abs(curvature) > 0.1:
            # 弯道减速
            target_speed = self.max_linear_vel / (1.0 + abs(curvature) * 10)

        # 接近终点时减速
        end_dist = math.hypot(x - self.path[-1][0], y - self.path[-1][1])
        if end_dist < lookahead * 2:
            target_speed = min(target_speed, end_dist * self.linear_kp)

        # 发布
        self.publish_cmd(target_speed, angular_vel)

    def find_nearest_point(self, x, y):
        """找到最近路径点"""
        if not self.path:
            return None
        min_dist = float('inf')
        nearest = 0
        for i, (px, py) in enumerate(self.path):
            dist = math.hypot(x - px, y - py)
            if dist < min_dist:
                min_dist = dist
                nearest = i
        return nearest

    def find_lookahead_point(self, x, y, lookahead, start_idx):
        """找到前瞻点: 路径上距离当前位置等于前视距离的点"""
        for i in range(start_idx, len(self.path)):
            px, py = self.path[i]
            dist = math.hypot(x - px, y - py)
            if dist >= lookahead:
                # 线性插值以获得更精确的前瞻点
                if i == start_idx:
                    return (px, py)
                prev_x, prev_y = self.path[i - 1]
                # 线段插值
                return self.interpolate_on_segment(x, y, prev_x, prev_y, px, py, lookahead)

        # 如果前视距离超出路径终点，返回终点
        return self.path[-1]

    def interpolate_on_segment(self, cx, cy, x1, y1, x2, y2, radius):
        """在(x1,y1)-(x2,y2)线段上找到距离(cx,cy)为radius的点"""
        # 线段方向向量
        dx = x2 - x1
        dy = y2 - y1
        seg_len = math.hypot(dx, dy)
        if seg_len < 1e-6:
            return (x2, y2)

        # 二分查找
        lo, hi = 0.0, 1.0
        for _ in range(20):
            mid = (lo + hi) / 2.0
            px = x1 + mid * dx
            py = y1 + mid * dy
            dist = math.hypot(cx - px, cy - py)
            if dist < radius:
                lo = mid
            else:
                hi = mid

        t = (lo + hi) / 2.0
        return (x1 + t * dx, y1 + t * dy)

    def calc_curvature(self, x, y, theta, lookahead_point, lookahead):
        """计算纯追踪曲率: kappa = 2 * sin(alpha) / L"""
        lx, ly = lookahead_point
        # 前瞻点在船舶坐标系中的位置
        dx = lx - x
        dy = ly - y
        # 转换到船体坐标系
        local_x = dx * math.cos(-theta) - dy * math.sin(-theta)
        local_y = dx * math.sin(-theta) + dy * math.cos(-theta)

        # 纯追踪公式
        if lookahead < 1e-6:
            return 0.0
        curvature = 2.0 * local_y / (lookahead * lookahead)
        return curvature

    def publish_cmd(self, v, w):
        """发布速度指令"""
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)

    def stop(self):
        self.publish_cmd(0.0, 0.0)


if __name__ == '__main__':
    try:
        controller = PurePursuitController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass