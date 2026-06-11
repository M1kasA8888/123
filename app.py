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
from typing import List, Dict, Optional

# ==================== 坐标系转换模块 ====================
class CoordConverter:
    """坐标系转换工具（WGS-84 ↔ GCJ-02）"""
    
    a = 6378245.0  # 长半轴
    ee = 0.00669342162296594323  # 偏心率平方
    
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
        """GCJ-02 转 WGS-84"""
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
        """WGS-84 转 GCJ-02"""
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
        num_samples = 10
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
        self.last_heartbeat_time = None
    
    def send_heartbeat(self):
        self.sequence_number += 1
        send_time = datetime.now()
        heartbeat = {
            'seq': self.sequence_number,
            'send_time': send_time,
            'status': 'sent'
        }
        self.send_log.append(heartbeat)
        # 模拟接收（延迟很小）
        receive_time = datetime.now()
        heartbeat['receive_time'] = receive_time
        heartbeat['delay'] = round((receive_time - send_time).total_seconds() * 1000, 2)
        self.receive_log.append(heartbeat)
        self.last_heartbeat_time = receive_time
        return heartbeat
    
    def get_status(self):
        current_time = datetime.now()
        last_time = self.last_heartbeat_time
        is_connected = last_time and (current_time - last_time).total_seconds() < 3
        
        total_sent = len(self.send_log)
        total_received = len(self.receive_log)
        
        return {
            'heartbeat_rate': 60 if is_connected else 0,
            'last_heartbeat_time': last_time,
            'total_sent': total_sent,
            'total_received': total_received,
            'is_connected': is_connected,
            'success_rate': (total_received / total_sent * 100) if total_sent > 0 else 0
        }
    
    def get_recent_heartbeats(self, n=10):
        recent = []
        for h in self.receive_log[-n:]:
            recent.append({
                'seq': h['seq'],
                'send_time': h['send_time'].strftime("%H:%M:%S.%f")[:-3],
                'receive_time': h['receive_time'].strftime("%H:%M:%S.%f")[:-3],
                'delay_ms': h['delay']
            })
        return recent


# ==================== 页面配置 ====================
st.set_page_config(
    page_title="南京科技职业学院 - 无人机智能监控系统",
    page_icon="🚁",
    layout="wide"
)

# 南京科技职业学院坐标（GCJ-02）
CAMPUS_CENTER = [32.234097, 118.749413]

# 初始化session state
if 'page' not in st.session_state:
    st.session_state.page = "航线规划"
if 'obstacles' not in st.session_state:
    st.session_state.obstacles = []
if 'waypoints' not in st.session_state:
    st.session_state.waypoints = []
if 'flight_plan' not in st.session_state:
    st.session_state.flight_plan = None
if 'coord_type' not in st.session_state:
    st.session_state.coord_type = "GCJ-02"
if 'point_a' not in st.session_state:
    st.session_state.point_a = [32.2322, 118.749]
if 'point_b' not in st.session_state:
    st.session_state.point_b = [32.2343, 118.754]
if 'heartbeat_monitor' not in st.session_state:
    st.session_state.heartbeat_monitor = HeartbeatMonitor()
if 'is_flying' not in st.session_state:
    st.session_state.is_flying = False
if 'simulator' not in st.session_state:
    st.session_state.simulator = None
if 'start_time' not in st.session_state:
    st.session_state.start_time = None


# ==================== 侧边栏 ====================
with st.sidebar:
    st.title("🚁 无人机系统")
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
    st.info(f"当前坐标系: {st.session_state.coord_type}")
    
    st.markdown("---")
    
    # 系统状态
    st.subheader("📊 系统状态")
    st.success("✅ 系统运行正常")
    st.info(f"📍 障碍区数量: {len(st.session_state.obstacles)}")
    
    if st.session_state.flight_plan:
        st.success("✈️ 航线已规划")
    else:
        st.warning("⚠️ 未规划航线")


# ==================== 页面1: 航线规划 ====================
if st.session_state.page == "🗺️ 航线规划":
    st.title("🗺️ 航线规划")
    st.markdown("设置起降点、障碍区，规划安全航线 — **南京科技职业学院**")
    st.markdown("---")
    
    col_left, col_right = st.columns([1.5, 1])
    
    with col_left:
        st.subheader("🗺️ 校园全景地图")
        st.caption("📍 南京科技职业学院 | 坐标: 32.2341°N, 118.7494°E")
        
        # 创建地图 - 使用高德卫星图
        m = folium.Map(
            location=CAMPUS_CENTER,
            zoom_start=17,
            control_scale=True,
            tiles=None
        )
        
        # 添加高德卫星图（最清晰，显示校园建筑细节）
        folium.TileLayer(
            tiles='https://webst0{s}.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}',
            attr='高德卫星图',
            subdomains=['1', '2', '3', '4'],
            name='📷 卫星地图'
        ).add_to(m)
        
        # 添加高德街道图
        folium.TileLayer(
            tiles='https://webrd0{s}.is.autonavi.com/appmaptile?lang=zh_cn&size=1&scale=1&style=8&x={x}&y={y}&z={z}',
            attr='高德地图',
            subdomains=['1', '2', '3', '4'],
            name='🗺️ 街道地图'
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
        
        # 添加校园范围示意
        folium.Circle(
            CAMPUS_CENTER,
            radius=300,
            color='blue',
            fill=True,
            fill_opacity=0.1,
            popup='校园范围 (300m)'
        ).add_to(m)
        
        # 绘制已保存的障碍区
        for i, obstacle in enumerate(st.session_state.obstacles):
            if len(obstacle) >= 3:
                display_obs = obstacle
                if st.session_state.coord_type == "GCJ-02":
                    # 转换坐标显示
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
                    fill_opacity=0.3,
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
                color='blue',
                weight=3,
                opacity=0.8,
                popup='✈️ 规划航线'
            ).add_to(m)
            
            for i, wp in enumerate(display_wps[1:-1], 1):
                folium.Marker(
                    wp,
                    popup=f'📍 航点 {i}',
                    icon=folium.Icon(color='orange', icon='info-sign')
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
        
        # 添加图层控制
        folium.LayerControl().add_to(m)
        
        # 添加测量工具
        plugins.MeasureControl().add_to(m)
        
        # 显示地图
        output = st_folium(m, width=700, height=500, key="planning_map")
        
        # 处理地图绘图
        if output and 'last_active_drawing' in output:
            drawing = output['last_active_drawing']
            if drawing and drawing['geometry']['type'] == 'Polygon':
                coords = drawing['geometry']['coordinates'][0]
                points = [[c[1], c[0]] for c in coords]
                st.session_state['temp_obstacle'] = points
                st.success(f"已绘制 {len(points)} 个点的障碍区，点击'保存障碍区'确认")
    
    with col_right:
        st.subheader("🎯 控制面板")
        
        # 校园快速定位
        st.markdown("### 🏫 校园快速定位")
        if st.button("📍 定位南京科技职业学院", use_container_width=True):
            st.session_state.map_center = CAMPUS_CENTER
            st.success("已定位到学院中心")
            st.rerun()
        
        st.markdown("---")
        
        # A点设置
        st.markdown("### 🚁 起点 A")
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
        flight_height = st.slider("设定飞行高度 (m)", 20, 200, 50)
        safe_radius = st.slider("安全半径 (m)", 10, 100, 30)
        
        st.markdown("---")
        
        # 障碍区管理
        st.markdown("### 🚧 障碍区管理")
        
        if 'temp_obstacle' in st.session_state:
            col1, col2 = st.columns(2)
            with col1:
                if st.button("✅ 保存障碍区", use_container_width=True):
                    st.session_state.obstacles.append(st.session_state.temp_obstacle)
                    del st.session_state.temp_obstacle
                    st.success("障碍区已保存")
                    st.rerun()
            with col2:
                if st.button("🗑️ 取消", use_container_width=True):
                    del st.session_state.temp_obstacle
                    st.rerun()
        
        if st.session_state.obstacles:
            for i, obs in enumerate(st.session_state.obstacles):
                with st.expander(f"障碍区 {i+1} ({len(obs)} 个点)"):
                    if st.button(f"删除", key=f"del_{i}"):
                        st.session_state.obstacles.pop(i)
                        st.rerun()
        
        if st.button("🗑️ 清除所有障碍区", use_container_width=True):
            st.session_state.obstacles = []
            st.rerun()
        
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
                        st.success("航线规划成功！")
                        st.rerun()
                    else:
                        st.error("无法规划安全航线，请调整障碍区或航点")
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
                st.metric("📡 延迟", f"{heartbeat['delay']} ms")
            
            st.progress(int(status['progress']))
            
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
        
        col1, col2 = st.columns(2)
        with col1:
            st.metric("📤 发送总数", hb_status['total_sent'])
        with col2:
            st.metric("📥 接收总数", hb_status['total_received'])
        
        col1, col2 = st.columns(2)
        with col1:
            st.metric("✅ 成功率", f"{hb_status['success_rate']:.1f}%")
        with col2:
            status_text = "🟢 正常" if hb_status['is_connected'] else "🔴 超时"
            st.metric("🔗 连接状态", status_text)
        
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
            
            df_delay = pd.DataFrame(st.session_state.heartbeat_monitor.receive_log[-30:])
            fig = px.line(df_delay, x='seq', y='delay',
                         title="心跳延迟实时监控",
                         labels={'seq': '心跳序号', 'delay': '延迟(ms)'})
            st.plotly_chart(fig, use_container_width=True)
