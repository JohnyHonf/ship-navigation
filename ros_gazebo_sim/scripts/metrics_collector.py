#!/usr/bin/env python3
"""
实验指标采集器 - 记录路径长度、扩展节点数、规划耗时、航行时间、
到达成功率、横向误差、航向变化、安全距离和碰撞次数
"""

import rospy
import math
import json
import os
from datetime import datetime
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32, Float32, String
from geometry_msgs.msg import PoseStamped


class MetricsCollector:
    def __init__(self):
        rospy.init_node('metrics_collector')

        self.output_dir = rospy.get_param('~output_dir', '/tmp/ship_nav_metrics')
        os.makedirs(self.output_dir, exist_ok=True)

        # 指标
        self.metrics = {
            'experiment_id': '',
            'timestamp': '',
            'path_length': 0.0,           # 全局路径长度(m)
            'nodes_expanded': 0,          # 扩展节点数
            'planning_time': 0.0,         # 规划耗时(s)
            'navigation_time': 0.0,       # 航行时间(s)
            'reached_goal': False,        # 是否到达目标
            'lateral_errors': [],         # 横向误差序列
            'mean_lateral_error': 0.0,    # 平均横向误差
            'max_lateral_error': 0.0,     # 最大横向误差
            'heading_changes': [],         # 航向变化序列
            'heading_std': 0.0,           # 航向变化标准差
            'min_safety_dist': float('inf'), # 最小安全距离
            'collision_count': 0,         # 碰撞次数
            'avg_speed': 0.0,             # 平均速度
            'speeds': [],                 # 速度序列
            'path_poses': []              # 实际路径
        }

        self.start_time = None
        self.goal_pose = None
        self.collision_threshold = 0.5  # 碰撞距离阈值

        # 订阅器
        rospy.Subscriber('/global_plan', Path, self.path_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/planner_nodes_expanded', Int32, self.nodes_callback)
        rospy.Subscriber('/planner_time', Float32, self.time_callback)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)

        # 保存定时器
        rospy.Timer(rospy.Duration(5.0), self.periodic_save)

        rospy.on_shutdown(self.on_shutdown)

        rospy.loginfo(f"MetricsCollector started. Output: {self.output_dir}")

    def path_callback(self, msg):
        if msg.poses:
            self.metrics['path_length'] = self._calc_path_length(msg)
            # 记录路径点数
            self.metrics['path_waypoints'] = len(msg.poses)

    def odom_callback(self, msg):
        if self.start_time is None:
            self.start_time = rospy.Time.now()

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        v = msg.twist.twist.linear.x

        self.metrics['speeds'].append(v)
        self.metrics['path_poses'].append((x, y, rospy.Time.now().to_sec()))

        # 计算横向误差
        if self.goal_pose:
            lateral_err = self._calc_lateral_error(x, y)
            self.metrics['lateral_errors'].append(lateral_err)

        # 检查是否到达目标
        if self.goal_pose and not self.metrics['reached_goal']:
            dist = math.hypot(x - self.goal_pose.position.x,
                              y - self.goal_pose.position.y)
            if dist < 1.0:
                self.metrics['reached_goal'] = True
                self.metrics['navigation_time'] = (
                    rospy.Time.now() - self.start_time).to_sec()

    def scan_callback(self, msg):
        """从激光雷达数据提取最小安全距离"""
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                if r < self.metrics['min_safety_dist']:
                    self.metrics['min_safety_dist'] = r
                if r < self.collision_threshold:
                    self.metrics['collision_count'] += 1

    def nodes_callback(self, msg):
        self.metrics['nodes_expanded'] = msg.data

    def time_callback(self, msg):
        self.metrics['planning_time'] = msg.data

    def goal_callback(self, msg):
        self.goal_pose = msg

    def _calc_path_length(self, path_msg):
        length = 0.0
        for i in range(1, len(path_msg.poses)):
            dx = path_msg.poses[i].pose.position.x - path_msg.poses[i-1].pose.position.x
            dy = path_msg.poses[i].pose.position.y - path_msg.poses[i-1].pose.position.y
            length += math.sqrt(dx*dx + dy*dy)
        return length

    def _calc_lateral_error(self, x, y):
        """简化横向误差计算: 到全局路径的最近距离"""
        return abs(y)  # 简化: 假设参考路径沿x轴

    def periodic_save(self, event):
        self._compute_summary()
        self._save_metrics()

    def _compute_summary(self):
        """计算汇总指标"""
        if self.metrics['lateral_errors']:
            self.metrics['mean_lateral_error'] = (
                sum(self.metrics['lateral_errors']) / len(self.metrics['lateral_errors']))
            self.metrics['max_lateral_error'] = max(self.metrics['lateral_errors'])

        if self.metrics['speeds']:
            self.metrics['avg_speed'] = (
                sum(self.metrics['speeds']) / len(self.metrics['speeds']))

        # 航向变化标准差
        if len(self.metrics['path_poses']) > 1:
            headings = []
            for i in range(1, len(self.metrics['path_poses'])):
                dx = self.metrics['path_poses'][i][0] - self.metrics['path_poses'][i-1][0]
                dy = self.metrics['path_poses'][i][1] - self.metrics['path_poses'][i-1][1]
                if dx or dy:
                    headings.append(math.atan2(dy, dx))
            if headings:
                mean_h = sum(headings) / len(headings)
                var = sum((h - mean_h)**2 for h in headings) / len(headings)
                self.metrics['heading_std'] = math.sqrt(var)

        if self.metrics['min_safety_dist'] == float('inf'):
            self.metrics['min_safety_dist'] = 0.0

    def _save_metrics(self):
        """保存指标到JSON"""
        self.metrics['timestamp'] = datetime.now().isoformat()

        filename = f"metrics_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        filepath = os.path.join(self.output_dir, filename)

        # 移除大列表减少文件大小
        save_data = {k: v for k, v in self.metrics.items()
                     if k not in ['path_poses', 'speeds', 'lateral_errors',
                                  'heading_changes']}

        with open(filepath, 'w') as f:
            json.dump(save_data, f, indent=2)

        rospy.loginfo(f"Metrics saved: {filepath}")

    def on_shutdown(self):
        self._compute_summary()
        self._save_metrics()
        self._print_summary()

    def _print_summary(self):
        """打印实验摘要"""
        rospy.loginfo("=" * 50)
        rospy.loginfo("  实验评估摘要")
        rospy.loginfo("=" * 50)
        rospy.loginfo(f"  路径长度:       {self.metrics['path_length']:.2f} m")
        rospy.loginfo(f"  扩展节点数:     {self.metrics['nodes_expanded']}")
        rospy.loginfo(f"  规划耗时:       {self.metrics['planning_time']*1000:.1f} ms")
        rospy.loginfo(f"  航行时间:       {self.metrics['navigation_time']:.1f} s")
        rospy.loginfo(f"  到达目标:       {'是' if self.metrics['reached_goal'] else '否'}")
        rospy.loginfo(f"  平均横向误差:   {self.metrics['mean_lateral_error']:.2f} m")
        rospy.loginfo(f"  最大横向误差:   {self.metrics['max_lateral_error']:.2f} m")
        rospy.loginfo(f"  航向变化标准差: {self.metrics['heading_std']:.3f} rad")
        rospy.loginfo(f"  最小安全距离:   {self.metrics['min_safety_dist']:.2f} m")
        rospy.loginfo(f"  碰撞次数:       {self.metrics['collision_count']}")
        rospy.loginfo(f"  平均速度:       {self.metrics['avg_speed']:.2f} m/s")
        rospy.loginfo("=" * 50)


if __name__ == '__main__':
    try:
        collector = MetricsCollector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass