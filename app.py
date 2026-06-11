import streamlit as st
import folium
from streamlit_folium import st_folium
from folium import plugins
import pandas as pd
import numpy as np
from datetime import datetime
import time
import plotly.graph_objects as go
import plotly.express as px
import math
import json
import os
from typing import List, Dict, Optional

# ==================== 坐标系转换模块 ====================
class CoordConverter:
    """坐标系转换工具（WGS-84 ↔ GCJ-02）"""
    
    a = 6378245.0
    ee = 0.00669342162296594323
    
    @staticmethod
    def _transform_lat(lon, lat):
        ret = -100.0 + 2.0 * lon + 3.0 * lat + 0.2 * lat * lat + \
              0.1 * lon * lat + 0.2 * math.sqrt(abs(lon))
        ret += (20.0 * math.sin(6.0 * lon * math.pi) + 20.0 *
                math.sin(2.0 * lon * math.pi)) * 2.0 / 3.0
        ret += (20.0 * math.sin(lat * math.pi) + 40.0 *
                math.sin(lat / 3.0 * math.pi)) * 2.0 / 3.0
        ret += (160.0 * math.sin(lat / 12.0 * math.pi) + 320 *
                math.sin(lat * math.pi / 30.0)) * 2.0 / 3.0
        return ret
    
    @staticmethod
    def _transform_lon(lon, lat):
        ret = 300.0 + lon + 2.0 * lat + 0.1 * lon * lon + \
              0.1 * lon * lat + 0.1 * math.sqrt(abs(lon))
        ret += (20.0 * math.sin(6.0 * lon * math.pi) + 20.0 *
                math.sin(2.0 * lon * math.pi)) * 2.0 / 3.0
        ret += (20.0 * math.sin(lon * math.pi) + 40.0 *
                math.sin(lon / 3.0 * math.pi)) * 2.0 / 3.0
        ret += (150.0 * math.sin(lon / 12.0 * math.pi) + 300.0 *
                math.sin(lon / 30.0 * math.pi)) * 2.0 / 3.0
        return ret
    
    @staticmethod
    def _out_of_china(lon, lat):
        if lon < 72.004 or lon > 137.8347:
            return True
        if lat < 0.8293 or lat > 55.8271:
            return True
        return False
    
    @classmethod
    def gcj02_to_wgs84(cls, lat, lon):
        if cls._out_of_china(lon, lat):
            return lat, lon
        
        dlat = cls._transform_lat(lon - 105.0, lat - 35.0)
        dlon = cls._transform_lon(lon - 105.0, lat - 35.0)
        
        radlat = lat / 180.0 * math.pi
        magic = math.sin(radlat)
        magic = 1 - cls.ee * magic * magic
        sqrtmagic = math.sqrt(magic)
        
        dlat = (dlat * 180.0) / ((cls.a * (1 - cls.ee)) / (magic * sqrtmagic) * math.pi)
        dlon = (dlon * 180.0) / (cls.a / sqrtmagic * math.cos(radlat) * math.pi)
        
        mg_lat = lat + dlat
        mg_lon = lon + dlon
        return mg_lat, mg_lon
    
    @classmethod
    def wgs84_to_gcj02(cls, lat, lon):
        if cls._out_of_china(lon, lat):
            return lat, lon
        
        dlat = cls._transform_lat(lon - 105.0, lat - 35.0)
        dlon = cls._transform_lon(lon - 105.0, lat - 35.0)
        
        radlat = lat / 180.0 * math.pi
        magic = math.sin(radlat)
        magic = 1 - cls.ee * magic * magic
        sqrtmagic = math.sqrt(magic)
        
        dlat = (dlat * 180.0) / ((cls.a * (1 - cls.ee)) / (magic * sqrtmagic) * math.pi)
        dlon = (dlon * 180.0) / (cls.a / sqrtmagic * math.cos(radlat) * math.pi)
        
        mg_lat = lat + dlat
        mg_lon = lon + dlon
        return mg_lat, mg_lon


# ==================== 障碍物持久化管理 ====================
class ObstaclePersistence:
    """障碍物配置持久化管理"""
    
    CONFIG_FILE = "obstacle_config.json"
    VERSION = "v12.2"
    
    @classmethod
    def save_obstacles(cls, obstacles: List):
        """保存障碍物配置到文件"""
        config = {
            'version': cls.VERSION,
            'save_time': datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            'obstacles': obstacles,
            'count': len(obstacles)
        }
        try:
            with open(cls.CONFIG_FILE, 'w', encoding='utf-8') as f:
                json.dump(config, f, ensure_ascii=False, indent=2)
            return True, config
        except Exception as e:
            return False, str(e)
    
    @classmethod
    def load_obstacles(cls):
        """从文件加载障碍物配置"""
        if not os.path.exists(cls.CONFIG_FILE):
            return [], None
        
        try:
            with open(cls.CONFIG_FILE, 'r', encoding='utf-8') as f:
                config = json.load(f)
            return config.get('obstacles', []), config
        except Exception as e:
            return [], None
    
    @classmethod
    def get_config_path(cls):
        """获取配置文件绝对路径"""
        return os.path.abspath(cls.CONFIG_FILE)
    
    @classmethod
    def get_config_status(cls):
        """获取配置文件状态"""
        if os.path.exists(cls.CONFIG_FILE):
            try:
                with open(cls.CONFIG_FILE, 'r', encoding='utf-8') as f:
                    config = json.load(f)
                return {
                    'exists': True,
                    'count': config.get('count', 0),
                    'save_time': config.get('save_time', '未知'),
                    'version': config.get('version', '未知'),
                    'path': cls.get_config_path()
                }
            except:
                return {'exists': False, 'error': '读取失败'}
        return {'exists': False, 'count': 0}


# ==================== 航线规划模块 ====================
class FlightPlanner:
    def __init__(self, obstacles: List[List[List[float]]], safe_radius: float):
        self.obstacles = obstacles
        self.safe_radius = safe_radius
    
    def calculate_distance(self, point1: List[float], point2: List[float]) -> float:
        lat1, lon1 = math.radians(point1[0]), math.radians(point1[1])
        lat2, lon2 = math.radians(point2[0]), math.radians(point2[1])
        R = 6371000
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
        return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    
    def is_point_in_polygon(self, point: List[float], polygon: List[List[float]]) -> bool:
        x, y = point[1], point[0]
        inside = False
        n = len(polygon)
        for i in range(n):
            x1, y1 = polygon[i][1], polygon[i][0]
            x2, y2 = polygon[(i + 1) % n][1], polygon[(i + 1) % n][0]
            if ((y1 > y) != (y2 > y)) and (x < (x2 - x1) * (y - y1) / (y2 - y1) + x1):
                inside = not inside
        return inside
    
    def is_point_safe(self, point: List[float]) -> bool:
        for obstacle in self.obstacles:
            if self.is_point_in_polygon(point, obstacle):
                return False
        return True
    
    def is_line_safe(self, start: List[float], end: List[float]) -> bool:
        num_samples = 20
        for i in range(num_samples + 1):
            t = i / num_samples
            lat = start[0] + (end[0] - start[0]) * t
            lon = start[1] + (end[1] - start[1]) * t
            if not self.is_point_safe([lat, lon]):
                return False
        return True
    
    def plan_route(self, start: List[float], end: List[float]) -> Optional[Dict]:
        if not self.is_point_safe(start) or not self.is_point_safe(end):
            return None
        
        if self.is_line_safe(start, end):
            total_distance = self.calculate_distance(start, end)
            return {
                'waypoints': [start, end],
                'total_distance': total_distance,
                'estimated_time': total_distance / 15,
                'is_safe': True,
                'path_type': '直线路径',
                'num_waypoints': 2
            }
        
        mid_lat = (start[0] + end[0]) / 2
        mid_lon = (start[1] + end[1]) / 2
        offsets = [0.001, 0.002, 0.003, 0.004, 0.005, -0.001, -0.002, -0.003, -0.004, -0.005]
        
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
                            'num_waypoints': 3
                        }
        return None


# ==================== 无人机模拟器模块 ====================
class DroneSimulator:
    def __init__(self, waypoints: List[List[float]], speed: float = 15):
        self.waypoints = waypoints
        self.speed = speed
        self.current_waypoint_index = 0
        self.current_position = waypoints[0].copy() if waypoints else [0, 0]
        self.completed_distance = 0.0
        self.total_distance = self._calculate_total_distance()
        self.is_flying = True
    
    def _calculate_distance(self, p1, p2):
        lat1, lon1 = math.radians(p1[0]), math.radians(p1[1])
        lat2, lon2 = math.radians(p2[0]), math.radians(p2[1])
        R = 6371000
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
        return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    
    def _calculate_total_distance(self):
        total = 0
        for i in range(len(self.waypoints) - 1):
            total += self._calculate_distance(self.waypoints[i], self.waypoints[i+1])
        return total
    
    def update(self, delta_time=0.1):
        if not self.is_flying or self.current_waypoint_index >= len(self.waypoints) - 1:
            return False
        
        target = self.waypoints[self.current_waypoint_index + 1]
        distance_to_target = self._calculate_distance(self.current_position, target)
        step_distance = self.speed * delta_time
        
        if distance_to_target <= step_distance:
            self.current_position = target
            self.completed_distance += distance_to_target
            self.current_waypoint_index += 1
        else:
            ratio = step_distance / distance_to_target
            new_lat = self.current_position[0] + (target[0] - self.current_position[0]) * ratio
            new_lon = self.current_position[1] + (target[1] - self.current_position[1]) * ratio
            self.current_position = [new_lat, new_lon]
            self.completed_distance += step_distance
        
        return self.current_waypoint_index < len(self.waypoints) - 1
    
    def get_status(self):
        progress = (self.completed_distance / self.total_distance * 100) if self.total_distance > 0 else 0
        return {
            'position': self.current_position,
            'current_waypoint': self.current_waypoint_index + 1,
            'total_waypoints': len(self.waypoints),
            'remaining_distance': self.total_distance - self.completed_distance,
            'progress': progress,
            'completed_distance': self.completed_distance
        }


# ==================== 心跳监控模块 ====================
class HeartbeatMonitor:
    def __init__(self):
        self.sequence_number = 0
        self.send_log = []
        self.receive_log = []
        self.timeout_log = []
        self.last_heartbeat_time = None
        self.is_connected = False
    
    def send_heartbeat(self):
        self.sequence_number += 1
        send_time = datetime.now()
        heartbeat = {
            'seq': self.sequence_number,
            'send_time': send_time,
        }
        self.send_log.append(heartbeat)
        
        # 模拟网络延迟（0-50ms）
        import random
        delay_ms = random.uniform(5, 50)
        receive_time = send_time + timedelta(milliseconds=delay_ms)
        heartbeat['receive_time'] = receive_time
        heartbeat['delay_ms'] = round(delay_ms, 2)
        self.receive_log.append(heartbeat)
        self.last_heartbeat_time = receive_time
        self.is_connected = True
        return heartbeat
    
    def check_timeout(self):
        """检查超时"""
        current_time = datetime.now()
        if self.last_heartbeat_time:
            elapsed = (current_time - self.last_heartbeat_time).total_seconds()
            if elapsed > 3:
                self.is_connected = False
                self.timeout_log.append({
                    'time': current_time,
                    'elapsed': elapsed,
                    'message': f'连接超时: {elapsed:.1f}秒未收到心跳'
                })
        return self.is_connected
    
    def get_status(self):
        self.check_timeout()
        total_sent = len(self.send_log)
        total_received = len(self.receive_log)
        
        return {
            'heartbeat_rate': 60 if self.is_connected else 0,
            'last_heartbeat_time': self.last_heartbeat_time,
            'total_sent': total_sent,
            'total_received': total_received,
            'is_connected': self.is_connected,
            'success_rate': (total_received / total_sent * 100) if total_sent > 0 else 0,
            'timeout_count': len(self.timeout_log)
        }
    
    def get_recent_heartbeats(self, n=10):
        recent = []
        for h in self.receive_log[-n:]:
            recent.append({
                'seq': h['seq'],
                'send_time': h['send_time'].strftime("%H:%M:%S.%f")[:-3],
                'receive_time': h['receive_time'].strftime("%H:%M:%S.%f")[:-3],
                'delay_ms': h['delay_ms']
            })
        return recent
    
    def get_delay_data(self):
        return [(h['seq'], h['delay_ms']) for h in self.receive_log]


# ==================== 页面配置 ====================
st.set_page_config(
    page_title="南京科技职业学院 - 无人机智能监控系统",
    page_icon="🛰️",
    layout="wide"
)

# 南京科技职业学院坐标（GCJ-02）
CAMPUS_CENTER = [32.234097, 118.749413]

# 初始化session state
if 'page' not in st.session_state:
    st.session_state.page = "航线规划"
if 'obstacles' not in st.session_state:
    # 尝试加载保存的障碍物
    saved_obstacles, config = ObstaclePersistence.load_obstacles()
    st.session_state.obstacles = saved_obstacles if saved_obstacles else []
    st.session_state.obstacle_config = config
if 'waypoints' not in st.session_state:
    st.session_state.waypoints = []
if 'flight_plan' not in st.session_state:
    st.session_state.flight_plan = None
if 'coord_type' not in st.session_state:
    st.session_state.coord_type = "GCJ-02"
if 'point_a' not in st.session_state:
    st.session_state.point_a = [32.2323, 118.749]
if 'point_b' not in st.session_state:
    st.session_state.point_b = [32.2344, 118.749]
if 'heartbeat_monitor' not in st.session_state:
    st.session_state.heartbeat_monitor = HeartbeatMonitor()
if 'is_flying' not in st.session_state:
    st.session_state.is_flying = False
if 'simulator' not in st.session_state:
    st.session_state.simulator = None
if 'start_time' not in st.session_state:
    st.session_state.start_time = None
if 'altitude_data' not in st.session_state:
    st.session_state.altitude_data = []

from datetime import timedelta

# ==================== 侧边栏 ====================
with st.sidebar:
    st.title("🛰️ 无人机系统")
    st.caption("南京科技职业学院 · 智能监控平台")
    st.markdown("---")
    
    # 页面导航
    st.subheader("📱 功能页面")
    page = st.radio("", ["🗺️ 航线规划", "📡 飞行监控"], label_visibility="collapsed")
    st.session_state.page = page
    
    st.markdown("---")
    
    # 坐标系设置
    st.subheader("🌐 坐标系设置")
    coord_type = st.selectbox(
        "输入坐标系",
        ["WGS-84", "GCJ-02 (高德/百度)"],
        help="GCJ-02是中国国测局坐标，用于高德、百度地图"
    )
    st.session_state.coord_type = "WGS-84" if coord_type == "WGS-84" else "GCJ-02"
    
    if st.session_state.coord_type == "GCJ-02":
        st.info("📍 当前使用 GCJ-02 坐标系\n(高德/百度地图)")
    else:
        st.info("🌍 当前使用 WGS-84 坐标系\n(GPS/国际标准)")
    
    st.markdown("---")
    
    # 系统状态
    st.subheader("📊 系统状态")
    st.success("✅ 系统运行正常")
    
    # 障碍物持久化状态
    config_status = ObstaclePersistence.get_config_status()
    if config_status['exists']:
        st.info(f"💾 障碍物配置\n共 {config_status['count']} 个 | {config_status['save_time']}")
    else:
        st.warning("⚠️ 暂无保存的障碍物配置")


# ==================== 页面1: 航线规划 ====================
if st.session_state.page == "🗺️ 航线规划":
    st.title("🛰️ 航线规划")
    st.markdown("设置起降点、障碍区，规划安全航线 — **南京科技职业学院**")
    st.markdown("---")
    
    col_left, col_right = st.columns([1.5, 1])
    
    with col_left:
        st.subheader("🛰️ 卫星地图")
        st.caption("📍 南京科技职业学院 | 坐标: 32.2341°N, 118.7494°E | 地图: OpenStreetMap")
        
        # 创建地图 - 使用 OpenStreetMap
        m = folium.Map(
            location=CAMPUS_CENTER,
            zoom_start=18,
            control_scale=True
        )
        
        # 添加高德卫星图（更清晰）
        folium.TileLayer(
            tiles='https://webst0{s}.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}',
            attr='高德卫星地图',
            subdomains=['1', '2', '3', '4'],
            name='卫星地图',
            overlay=False,
            control=True
        ).add_to(m)
        
        # 添加OpenStreetMap作为备选
        folium.TileLayer(
            tiles='https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',
            attr='OpenStreetMap',
            name='街道地图'
        ).add_to(m)
        
        # 添加标注：南京科技职业学院
        folium.Marker(
            CAMPUS_CENTER,
            popup=folium.Popup(
                '<b>🏫 南京科技职业学院</b><br>'
                'Nanjing Polytechnic Institute<br>'
                '地址：南京市江北新区欣乐路188号<br>'
                '📍 坐标：32.234097, 118.749413',
                max_width=250
            ),
            icon=folium.Icon(color='red', icon='university', prefix='fa')
        ).add_to(m)
        
        # 绘制已保存的障碍区
        for i, obstacle in enumerate(st.session_state.obstacles):
            if len(obstacle) >= 3:
                display_obs = obstacle
                if st.session_state.coord_type == "GCJ-02":
                    display_obs = []
                    for p in obstacle:
                        wgs_lat, wgs_lon = CoordConverter.gcj02_to_wgs84(p[0], p[1])
                        display_obs.append([wgs_lat, wgs_lon])
                
                folium.Polygon(
                    locations=[[p[0], p[1]] for p in display_obs],
                    color='red',
                    weight=2,
                    fill=True,
                    fill_color='red',
                    fill_opacity=0.35,
                    popup=f'🚧 障碍区 {i+1}'
                ).add_to(m)
        
        # 绘制A点和B点
        if st.session_state.point_a:
            display_a = st.session_state.point_a
            if st.session_state.coord_type == "GCJ-02":
                display_a = CoordConverter.gcj02_to_wgs84(
                    st.session_state.point_a[0], 
                    st.session_state.point_a[1]
                )
            folium.Marker(
                [display_a[0], display_a[1]],
                popup='🚁 起飞点 A',
                icon=folium.Icon(color='green', icon='play', prefix='fa')
            ).add_to(m)
        
        if st.session_state.point_b:
            display_b = st.session_state.point_b
            if st.session_state.coord_type == "GCJ-02":
                display_b = CoordConverter.gcj02_to_wgs84(
                    st.session_state.point_b[0], 
                    st.session_state.point_b[1]
                )
            folium.Marker(
                [display_b[0], display_b[1]],
                popup='🎯 目标点 B',
                icon=folium.Icon(color='red', icon='flag-checkered', prefix='fa')
            ).add_to(m)
        
        # 绘制规划航线
        if st.session_state.flight_plan:
            waypoints = st.session_state.flight_plan['waypoints']
            display_wps = []
            for wp in waypoints:
                if st.session_state.coord_type == "GCJ-02":
                    display_wp = CoordConverter.gcj02_to_wgs84(wp[0], wp[1])
                else:
                    display_wp = wp
                display_wps.append([display_wp[0], display_wp[1]])
            
            folium.PolyLine(
                display_wps,
                color='cyan',
                weight=3,
                opacity=0.9,
                popup='✈️ 规划航线'
            ).add_to(m)
            
            for i, wp in enumerate(display_wps[1:-1], 1):
                folium.Marker(
                    wp,
                    popup=f'📍 航点 {i}',
                    icon=folium.Icon(color='orange', icon='info-sign', prefix='fa')
                ).add_to(m)
        
        # 添加绘图工具
        draw = plugins.Draw(
            draw_options={
                'polyline': False,
                'rectangle': False,
                'circle': False,
                'marker': True,
                'polygon': {'allowIntersection': False},
                'circlemarker': False
            },
            edit_options={'edit': True}
        )
        draw.add_to(m)
        
        # 添加测量工具
        plugins.MeasureControl(
            position='topleft',
            active_color='red',
            completed_color='green'
        ).add_to(m)
        
        # 添加全屏按钮
        plugins.Fullscreen().add_to(m)
        
        # 添加图层控制
        folium.LayerControl().add_to(m)
        
        # 显示地图
        output = st_folium(m, width=750, height=550, key="planning_map")
        
        # 处理地图绘图
        if output and 'last_active_drawing' in output:
            drawing = output['last_active_drawing']
            if drawing and drawing['geometry']['type'] == 'Polygon':
                coords = drawing['geometry']['coordinates'][0]
                points = [[c[1], c[0]] for c in coords]
                st.session_state['temp_obstacle'] = points
                st.success(f"✅ 已绘制 {len(points)} 个点的障碍区")
    
    with col_right:
        st.subheader("🎯 控制面板")
        
        # 校园快速定位
        st.markdown("### 🏫 校园快速定位")
        if st.button("📍 定位南京科技职业学院", use_container_width=True):
            st.success("已定位到学院中心")
            st.rerun()
        
        st.markdown("---")
        
        # A点设置
        st.markdown("### 🚁 起点 A")
        st.caption(f"输入坐标系: {st.session_state.coord_type}")
        col1, col2 = st.columns(2)
        with col1:
            lat_a = st.number_input("纬度", value=st.session_state.point_a[0], format="%.6f", key="lat_a")
        with col2:
            lon_a = st.number_input("经度", value=st.session_state.point_a[1], format="%.6f", key="lon_a")
        
        if st.button("📍 设置 A 点", use_container_width=True):
            st.session_state.point_a = [lat_a, lon_a]
            st.success(f"起点已设置: ({lat_a}, {lon_a})")
            st.rerun()
        
        # B点设置
        st.markdown("### 🎯 终点 B")
        col1, col2 = st.columns(2)
        with col1:
            lat_b = st.number_input("纬度", value=st.session_state.point_b[0], format="%.6f", key="lat_b")
        with col2:
            lon_b = st.number_input("经度", value=st.session_state.point_b[1], format="%.6f", key="lon_b")
        
        if st.button("🏁 设置 B 点", use_container_width=True):
            st.session_state.point_b = [lat_b, lon_b]
            st.success(f"终点已设置: ({lat_b}, {lon_b})")
            st.rerun()
        
        st.markdown("---")
        
        # 飞行参数
        st.markdown("### ⚙️ 飞行参数")
        flight_height = st.slider("设定飞行高度 (m)", 10, 200, 50)
        safe_radius = st.slider("安全半径 (m)", 10, 100, 30)
        
        st.markdown("---")
        
        # ========== 障碍物配置持久化 ==========
        st.markdown("### 🚧 障碍物配置持久化")
        st.caption(f"📁 配置文件: {ObstaclePersistence.get_config_path()}")
        st.caption(f"📌 版本: {ObstaclePersistence.VERSION}")
        
        # 显示当前障碍物状态
        if st.session_state.obstacles:
            st.info(f"📦 当前共 {len(st.session_state.obstacles)} 个障碍物")
        
        # 临时障碍物处理
        if 'temp_obstacle' in st.session_state:
            col1, col2 = st.columns(2)
            with col1:
                if st.button("✅ 保存障碍区", use_container_width=True, type="primary"):
                    st.session_state.obstacles.append(st.session_state.temp_obstacle)
                    del st.session_state.temp_obstacle
                    st.success("障碍区已添加")
                    st.rerun()
            with col2:
                if st.button("🗑️ 取消", use_container_width=True):
                    del st.session_state.temp_obstacle
                    st.rerun()
        
        # 障碍物列表
        if st.session_state.obstacles:
            with st.expander(f"📋 障碍物列表 ({len(st.session_state.obstacles)}个)"):
                for i, obs in enumerate(st.session_state.obstacles):
                    st.text(f"障碍区 {i+1}: {len(obs)} 个点")
                    if st.button(f"🗑️ 删除 {i+1}", key=f"del_{i}"):
                        st.session_state.obstacles.pop(i)
                        st.rerun()
        
        # 持久化操作按钮
        col1, col2, col3 = st.columns(3)
        with col1:
            if st.button("💾 保存配置", use_container_width=True):
                success, result = ObstaclePersistence.save_obstacles(st.session_state.obstacles)
                if success:
                    st.success(f"✅ 已保存 {len(st.session_state.obstacles)} 个障碍物")
                    config_status = ObstaclePersistence.get_config_status()
                    st.info(f"保存时间: {config_status['save_time']}")
                else:
                    st.error(f"保存失败: {result}")
        
        with col2:
            if st.button("📂 加载配置", use_container_width=True):
                loaded, config = ObstaclePersistence.load_obstacles()
                if loaded:
                    st.session_state.obstacles = loaded
                    st.success(f"✅ 已加载 {len(loaded)} 个障碍物")
                    if config:
                        st.info(f"保存时间: {config.get('save_time', '未知')}")
                    st.rerun()
                else:
                    st.warning("没有找到保存的配置")
        
        with col3:
            if st.button("🗑️ 清除全部", use_container_width=True):
                st.session_state.obstacles = []
                st.success("已清除所有障碍物")
                st.rerun()
        
        # 下载配置文件
        st.markdown("---")
        st.markdown("### 📥 下载配置文件")
        
        config_status = ObstaclePersistence.get_config_status()
        if config_status['exists']:
            st.caption(f"文件状态: 共 {config_status['count']} 个障碍物")
            st.caption(f"保存时间: {config_status['save_time']}")
            st.caption(f"版本: {config_status['version']}")
            
            with open(ObstaclePersistence.CONFIG_FILE, 'r', encoding='utf-8') as f:
                config_content = f.read()
            st.download_button(
                label="📥 下载 obstacle_config.json",
                data=config_content,
                file_name="obstacle_config.json",
                mime="application/json",
                use_container_width=True
            )
        else:
            st.info("暂无配置文件，保存后即可下载")
        
        st.markdown("---")
        
        # 航线规划按钮
        col1, col2 = st.columns(2)
        with col1:
            if st.button("🚀 规划航线", use_container_width=True, type="primary"):
                if st.session_state.point_a and st.session_state.point_b:
                    start = st.session_state.point_a
                    end = st.session_state.point_b
                    
                    if st.session_state.coord_type == "GCJ-02":
                        start = CoordConverter.gcj02_to_wgs84(start[0], start[1])
                        end = CoordConverter.gcj02_to_wgs84(end[0], end[1])
                    
                    planner = FlightPlanner(st.session_state.obstacles, safe_radius)
                    flight_plan = planner.plan_route(start, end)
                    
                    if flight_plan:
                        st.session_state.flight_plan = flight_plan
                        st.success("✅ 航线规划成功！")
                        st.rerun()
                    else:
                        st.error("❌ 无法规划安全航线，请调整障碍区或航点")
                else:
                    st.warning("请先设置 A 点和 B 点")
        
        with col2:
            if st.button("📊 航线信息", use_container_width=True):
                if st.session_state.flight_plan:
                    info = st.session_state.flight_plan
                    st.info(f"""
                    📏 总航程: {info['total_distance']:.2f} m
                    ⏱️ 预计时间: {info['estimated_time']:.2f} s
                    🛡️ 路径类型: {info['path_type']}
                    📍 航点数量: {info['num_waypoints']}
                    """)
                else:
                    st.warning("请先规划航线")


# ==================== 页面2: 飞行监控 ====================
else:
    st.title("📡 飞行监控")
    st.markdown("实时监控无人机飞行状态和心跳信号")
    st.markdown("---")
    
    col_left, col_right = st.columns([1, 1])
    
    with col_left:
        st.subheader("🚁 飞行状态")
        
        if st.session_state.flight_plan:
            col1, col2, col3 = st.columns(3)
            with col1:
                if st.button("▶️ 开始飞行", use_container_width=True, type="primary") and not st.session_state.is_flying:
                    st.session_state.is_flying = True
                    st.session_state.simulator = DroneSimulator(
                        st.session_state.flight_plan['waypoints'], 15
                    )
                    st.session_state.start_time = datetime.now()
                    st.session_state.altitude_data = []
                    st.rerun()
            
            with col2:
                if st.button("⏸️ 暂停", use_container_width=True):
                    st.session_state.is_flying = False
            
            with col3:
                if st.button("🛑 终止", use_container_width=True):
                    st.session_state.is_flying = False
                    st.session_state.simulator = None
                    st.rerun()
        
        # 飞行仪表盘
        if st.session_state.get('is_flying') and st.session_state.get('simulator'):
            status = st.session_state.simulator.get_status()
            elapsed = (datetime.now() - st.session_state.start_time).total_seconds()
            
            # 发送心跳
            heartbeat = st.session_state.heartbeat_monitor.send_heartbeat()
            
            # 仪表盘
            col1, col2, col3, col4 = st.columns(4)
            with col1:
                st.metric("📍 当前航点", f"{status['current_waypoint']}/{status['total_waypoints']}")
            with col2:
                st.metric("⚡ 飞行速度", "15 m/s")
            with col3:
                st.metric("⏱️ 已用时间", f"{elapsed:.1f}s")
            with col4:
                st.metric("📏 剩余距离", f"{status['remaining_distance']:.0f}m")
            
            col1, col2, col3, col4 = st.columns(4)
            with col1:
                st.metric("📊 完成进度", f"{status['progress']:.1f}%")
            with col2:
                battery = max(0, 100 - elapsed / 6)
                st.metric("🔋 电量", f"{battery:.0f}%")
            with col3:
                hb_status = st.session_state.heartbeat_monitor.get_status()
                st.metric("💓 心跳", f"{hb_status['heartbeat_rate']}/min")
            with col4:
                st.metric("📡 延迟", f"{heartbeat['delay_ms']} ms")
            
            # 进度条
            st.progress(int(status['progress']))
            
            # 模拟高度数据
            current_altitude = 50 + math.sin(elapsed * 2) * 5
            st.session_state.altitude_data.append({
                'time': elapsed,
                'altitude': current_altitude,
                'delay': heartbeat['delay_ms']
            })
            if len(st.session_state.altitude_data) > 50:
                st.session_state.altitude_data = st.session_state.altitude_data[-50:]
            
            # 实时数据图表
            if len(st.session_state.altitude_data) > 1:
                df = pd.DataFrame(st.session_state.altitude_data)
                fig = go.Figure()
                fig.add_trace(go.Scatter(x=df['time'], y=df['altitude'],
                                         mode='lines', name='飞行高度 (m)',
                                         line=dict(color='green', width=2)))
                fig.add_trace(go.Scatter(x=df['time'], y=df['delay'],
                                         mode='lines', name='心跳延迟 (ms)',
                                         line=dict(color='orange', width=2, dash='dash')))
                fig.update_layout(title="实时飞行数据",
                                 xaxis_title="时间 (秒)",
                                 yaxis_title="数值")
                st.plotly_chart(fig, use_container_width=True)
            
            if status['progress'] >= 100:
                st.success("✅ 飞行完成！")
                st.session_state.is_flying = False
                st.balloons()
            else:
                st.session_state.simulator.update(0.1)
                time.sleep(0.1)
                st.rerun()
        else:
            st.info("点击「开始飞行」启动监控")
    
    with col_right:
        st.subheader("💓 心跳信号监控")
        
        hb_status = st.session_state.heartbeat_monitor.get_status()
        
        # 心跳统计
        col1, col2 = st.columns(2)
        with col1:
            st.metric("📤 发送总数", hb_status['total_sent'])
        with col2:
            st.metric("📥 接收总数", hb_status['total_received'])
        
        col1, col2 = st.columns(2)
        with col1:
            st.metric("✅ 成功率", f"{hb_status['success_rate']:.1f}%")
        with col2:
            st.metric("⚠️ 超时次数", hb_status['timeout_count'])
        
        col1, col2 = st.columns(2)
        with col1:
            status_text = "🟢 正常" if hb_status['is_connected'] else "🔴 超时"
            st.metric("🔗 连接状态", status_text)
        with col2:
            st.metric("💓 心跳频率", f"{hb_status['heartbeat_rate']}/min")
        
        st.markdown("---")
        st.markdown("### 📋 最新心跳记录")
        
        recent = st.session_state.heartbeat_monitor.get_recent_heartbeats(8)
        if recent:
            df = pd.DataFrame(recent)
            st.dataframe(df, use_container_width=True, hide_index=True)
        else:
            st.info("等待心跳信号...")
        
        # 延迟图表
        if len(st.session_state.heartbeat_monitor.receive_log) > 0:
            st.markdown("---")
            st.markdown("### 📈 心跳延迟趋势")
            
            df_delay = pd.DataFrame(st.session_state.heartbeat_monitor.receive_log[-50:])
            fig = px.line(df_delay, x='seq', y='delay_ms',
                         title="心跳延迟实时监控",
                         labels={'seq': '心跳序号', 'delay_ms': '延迟(ms)'})
            fig.add_hline(y=sum([h['delay_ms'] for h in df_delay.to_dict('records')]) / len(df_delay) if len(df_delay) > 0 else 0,
                         line_dash="dash", line_color="red",
                         annotation_text="平均延迟")
            st.plotly_chart(fig, use_container_width=True)
