#!/usr/bin/env python3
"""
全局路径规划节点 - Dijkstra & A* 算法
订阅: /map (occupancy grid), /move_base_simple/goal (geometry_msgs/PoseStamped)
发布: /global_plan (nav_msgs/Path)
"""

import rospy
import math
import heapq
import time
from collections import deque
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Int32, Float32, String


class GlobalPlanner:
    def __init__(self):
        rospy.init_node('global_planner_node')

        # 参数
        self.algorithm = rospy.get_param('~algorithm', 'astar')  # 'dijkstra' or 'astar'
        self.heuristic_weight = rospy.get_param('~heuristic_weight', 1.2)
        self.obstacle_threshold = rospy.get_param('~obstacle_threshold', 50)
        self.inflation_radius = rospy.get_param('~inflation_radius', 2)  # 膨胀半径(格)

        # 地图数据
        self.map_data = None
        self.map_info = None
        self.width = 0
        self.height = 0
        self.resolution = 1.0
        self.origin_x = 0.0
        self.origin_y = 0.0

        # 起点和目标
        self.start = None
        self.goal = None

        # 统计
        self.nodes_expanded = 0
        self.planning_time = 0.0
        self.path_length = 0.0

        # 发布器
        self.path_pub = rospy.Publisher('/global_plan', Path, queue_size=1)
        self.stats_pub = rospy.Publisher('/planner_stats', String, queue_size=1)
        self.nodes_pub = rospy.Publisher('/planner_nodes_expanded', Int32, queue_size=1)
        self.time_pub = rospy.Publisher('/planner_time', Float32, queue_size=1)

        # 订阅器
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)

        # 服务: 切换算法
        rospy.Subscriber('/planner/set_algorithm', String, self.set_algorithm_callback)

        rospy.loginfo(f"GlobalPlanner started. Algorithm: {self.algorithm}")

    def set_algorithm_callback(self, msg):
        """切换规划算法"""
        if msg.data in ['dijkstra', 'astar']:
            self.algorithm = msg.data
            rospy.loginfo(f"Algorithm switched to: {self.algorithm}")

    def map_callback(self, msg):
        """接收地图数据"""
        self.map_data = msg.data
        self.map_info = msg.info
        self.width = msg.info.width
        self.height = msg.info.height
        self.resolution = msg.info.resolution
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y

        rospy.loginfo(f"Map received: {self.width}x{self.height}, resolution={self.resolution}")

    def goal_callback(self, msg):
        """接收目标点"""
        if self.map_data is None:
            rospy.logwarn("No map data received yet")
            return

        # 设置起点 (地图原点附近)
        self.start = self.world_to_grid(0, 0)
        self.goal = (
            int((msg.pose.position.x - self.origin_x) / self.resolution),
            int((msg.pose.position.y - self.origin_y) / self.resolution)
        )

        if not self.is_valid(self.goal[0], self.goal[1]):
            rospy.logwarn(f"Invalid goal: {self.goal}")
            return

        rospy.loginfo(f"Planning from {self.start} to {self.goal} using {self.algorithm}")

        # 执行规划
        t0 = time.time()
        if self.algorithm == 'dijkstra':
            path = self.dijkstra(self.start, self.goal)
        else:
            path = self.astar(self.start, self.goal)
        self.planning_time = time.time() - t0

        # 发布结果
        if path:
            self.path_length = self.compute_path_length(path)
            self.publish_path(path)
            self.publish_stats()
            rospy.loginfo(f"Path found: {len(path)} waypoints, "
                          f"length={self.path_length:.2f}m, "
                          f"time={self.planning_time*1000:.1f}ms, "
                          f"nodes={self.nodes_expanded}")
        else:
            rospy.logwarn("No path found!")

    def world_to_grid(self, wx, wy):
        """世界坐标转网格坐标"""
        gx = int((wx - self.origin_x) / self.resolution)
        gy = int((wy - self.origin_y) / self.resolution)
        return (gx, gy)

    def grid_to_world(self, gx, gy):
        """网格坐标转世界坐标"""
        wx = gx * self.resolution + self.origin_x
        wy = gy * self.resolution + self.origin_y
        return (wx, wy)

    def is_valid(self, gx, gy):
        """检查网格是否可通行"""
        if gx < 0 or gx >= self.width or gy < 0 or gy >= self.height:
            return False
        idx = gy * self.width + gx
        return self.map_data[idx] < self.obstacle_threshold

    def get_neighbors(self, node):
        """获取8邻域可通行节点"""
        gx, gy = node
        neighbors = []
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1),
                       (-1, -1), (-1, 1), (1, -1), (1, 1)]:
            nx, ny = gx + dx, gy + dy
            if self.is_valid(nx, ny):
                cost = math.sqrt(dx*dx + dy*dy)  # 对角线距离
                neighbors.append(((nx, ny), cost))
        return neighbors

    def heuristic(self, a, b):
        """启发式距离 (欧几里得)"""
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    def dijkstra(self, start, goal):
        """Dijkstra算法 - 最短路径基准"""
        self.nodes_expanded = 0
        pq = [(0, start)]
        came_from = {}
        cost_so_far = {start: 0}

        while pq:
            current_cost, current = heapq.heappop(pq)
            self.nodes_expanded += 1

            if current == goal:
                break

            if current_cost > cost_so_far.get(current, float('inf')):
                continue

            for neighbor, move_cost in self.get_neighbors(current):
                new_cost = current_cost + move_cost
                if new_cost < cost_so_far.get(neighbor, float('inf')):
                    cost_so_far[neighbor] = new_cost
                    heapq.heappush(pq, (new_cost, neighbor))
                    came_from[neighbor] = current

        return self.reconstruct_path(came_from, start, goal)

    def astar(self, start, goal):
        """A*算法 - 启发式搜索"""
        self.nodes_expanded = 0
        pq = [(0, start)]
        came_from = {}
        g_score = {start: 0}

        while pq:
            _, current = heapq.heappop(pq)
            self.nodes_expanded += 1

            if current == goal:
                break

            for neighbor, move_cost in self.get_neighbors(current):
                tentative_g = g_score[current] + move_cost
                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + self.heuristic_weight * self.heuristic(neighbor, goal)
                    heapq.heappush(pq, (f_score, neighbor))

        return self.reconstruct_path(came_from, start, goal)

    def reconstruct_path(self, came_from, start, goal):
        """从came_from表重建路径"""
        if goal not in came_from and start != goal:
            return None

        path = []
        current = goal
        while current != start:
            path.append(current)
            if current not in came_from:
                return None
            current = came_from[current]
        path.append(start)
        path.reverse()

        # 平滑路径 (移除冗余点)
        return self.smooth_path(path)

    def smooth_path(self, path):
        """路径平滑: 移除共线冗余点"""
        if len(path) <= 2:
            return path

        smoothed = [path[0]]
        for i in range(1, len(path) - 1):
            prev = smoothed[-1]
            curr = path[i]
            next_ = path[i + 1]
            # 如果三点共线，跳过中间点
            if not self.is_collinear(prev, curr, next_):
                smoothed.append(curr)
        smoothed.append(path[-1])
        return smoothed

    def is_collinear(self, a, b, c):
        """检查三点是否共线"""
        return (b[0] - a[0]) * (c[1] - a[1]) == (c[0] - a[0]) * (b[1] - a[1])

    def compute_path_length(self, path):
        """计算路径长度(米)"""
        length = 0.0
        for i in range(1, len(path)):
            dx = (path[i][0] - path[i-1][0]) * self.resolution
            dy = (path[i][1] - path[i-1][1]) * self.resolution
            length += math.sqrt(dx*dx + dy*dy)
        return length

    def publish_path(self, path):
        """发布路径消息"""
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = rospy.Time.now()

        for gx, gy in path:
            wx, wy = self.grid_to_world(gx, gy)
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

    def publish_stats(self):
        """发布规划统计"""
        stats = (f"Algorithm: {self.algorithm}, "
                 f"Nodes: {self.nodes_expanded}, "
                 f"Time: {self.planning_time*1000:.1f}ms, "
                 f"Length: {self.path_length:.2f}m")
        self.stats_pub.publish(stats)
        self.nodes_pub.publish(self.nodes_expanded)
        self.time_pub.publish(self.planning_time)


if __name__ == '__main__':
    try:
        planner = GlobalPlanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass