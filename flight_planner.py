import math
from typing import List, Dict, Optional

class FlightPlanner:
    """航线规划器"""
    
    def __init__(self, obstacles: List[List[List[float]]], safe_radius: float):
        self.obstacles = obstacles
        self.safe_radius = safe_radius
    
    def calculate_distance(self, point1: List[float], point2: List[float]) -> float:
        """计算两点间距离（米）"""
        lat1, lon1 = math.radians(point1[0]), math.radians(point1[1])
        lat2, lon2 = math.radians(point2[0]), math.radians(point2[1])
        
        R = 6371000  # 地球半径
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        
        a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        
        return R * c
    
    def is_point_in_polygon(self, point: List[float], polygon: List[List[float]]) -> bool:
        """射线法判断点是否在多边形内"""
        x, y = point[1], point[0]  # 转换为 (lon, lat)
        inside = False
        n = len(polygon)
        
        for i in range(n):
            x1, y1 = polygon[i][1], polygon[i][0]
            x2, y2 = polygon[(i + 1) % n][1], polygon[(i + 1) % n][0]
            
            if ((y1 > y) != (y2 > y)) and (x < (x2 - x1) * (y - y1) / (y2 - y1) + x1):
                inside = not inside
        
        return inside
    
    def is_point_safe(self, point: List[float]) -> bool:
        """检查点是否安全"""
        for obstacle in self.obstacles:
            if self.is_point_in_polygon(point, obstacle):
                return False
        return True
    
    def is_line_safe(self, start: List[float], end: List[float]) -> bool:
        """检查线段是否安全（采样检查）"""
        num_samples = 10
        for i in range(num_samples + 1):
            t = i / num_samples
            lat = start[0] + (end[0] - start[0]) * t
            lon = start[1] + (end[1] - start[1]) * t
            if not self.is_point_safe([lat, lon]):
                return False
        return True
    
    def plan_route(self, start: List[float], end: List[float]) -> Optional[Dict]:
        """规划航线"""
        if not self.is_point_safe(start) or not self.is_point_safe(end):
            return None
        
        # 尝试直线
        if self.is_line_safe(start, end):
            total_distance = self.calculate_distance(start, end)
            return {
                'waypoints': [start, end],
                'total_distance': total_distance,
                'estimated_time': total_distance / 15,
                'is_safe': True,
                'path_type': '直线路径',
                'start_point': start,
                'end_point': end,
                'num_waypoints': 2
            }
        
        # 尝试绕行（简单偏移）
        mid_lat = (start[0] + end[0]) / 2
        mid_lon = (start[1] + end[1]) / 2
        
        # 尝试不同的偏移量
        offsets = [0.002, 0.005, 0.008, -0.002, -0.005, -0.008]
        
        for offset_lat in offsets:
            for offset_lon in offsets:
                mid_point = [mid_lat + offset_lat, mid_lon + offset_lon]
                if self.is_point_safe(mid_point):
                    if self.is_line_safe(start, mid_point) and self.is_line_safe(mid_point, end):
                        waypoints = [start, mid_point, end]
                        total_distance = (self.calculate_distance(start, mid_point) +
                                        self.calculate_distance(mid_point, end))
                        return {
                            'waypoints': waypoints,
                            'total_distance': total_distance,
                            'estimated_time': total_distance / 15,
                            'is_safe': True,
                            'path_type': '绕行路径',
                            'start_point': start,
                            'end_point': end,
                            'num_waypoints': 3
                        }
        
        return None


class DroneSimulator:
    """无人机飞行模拟器"""
    
    def __init__(self, waypoints: List[List[float]], speed: float = 15):
        self.waypoints = waypoints
        self.speed = speed
        self.current_waypoint_index = 0
        self.current_position = waypoints[0].copy() if waypoints else [0, 0]
        self.completed_distance = 0.0
        self.total_distance = self._calculate_total_distance()
        self.is_flying = True
    
    def _calculate_distance(self, p1: List[float], p2: List[float]) -> float:
        """计算两点距离"""
        lat1, lon1 = math.radians(p1[0]), math.radians(p1[1])
        lat2, lon2 = math.radians(p2[0]), math.radians(p2[1])
        R = 6371000
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
        return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    
    def _calculate_total_distance(self) -> float:
        """计算总航程"""
        total = 0
        for i in range(len(self.waypoints) - 1):
            total += self._calculate_distance(self.waypoints[i], self.waypoints[i+1])
        return total
    
    def update(self, delta_time: float = 0.1) -> bool:
        """更新位置"""
        if not self.is_flying:
            return False
        
        if self.current_waypoint_index >= len(self.waypoints) - 1:
            self.is_flying = False
            return False
        
        target = self.waypoints[self.current_waypoint_index + 1]
        distance_to_target = self._calculate_distance(self.current_position, target)
        step_distance = self.speed * delta_time
        
        if distance_to_target <= step_distance:
            # 到达航点
            self.current_position = target
            self.completed_distance += distance_to_target
            self.current_waypoint_index += 1
        else:
            # 向目标移动
            ratio = step_distance / distance_to_target
            new_lat = self.current_position[0] + (target[0] - self.current_position[0]) * ratio
            new_lon = self.current_position[1] + (target[1] - self.current_position[1]) * ratio
            self.current_position = [new_lat, new_lon]
            self.completed_distance += step_distance
        
        return self.current_waypoint_index < len(self.waypoints) - 1
    
    def get_status(self) -> Dict:
        """获取状态"""
        if self.total_distance > 0:
            progress = (self.completed_distance / self.total_distance) * 100
        else:
            progress = 0
        
        return {
            'position': self.current_position,
            'current_waypoint': self.current_waypoint_index + 1,
            'total_waypoints': len(self.waypoints),
            'remaining_distance': self.total_distance - self.completed_distance,
            'completed_distance': self.completed_distance,
            'progress': progress,
            'segment_progress': 0,
            'is_flying': self.is_flying
        }
