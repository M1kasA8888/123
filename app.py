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

# 导入自定义模块
from coord_converter import CoordConverter
from flight_planner import FlightPlanner, DroneSimulator
from heartbeat_monitor import HeartbeatMonitor

st.set_page_config(
    page_title="无人机智能监控系统",
    page_icon="🚁",
    layout="wide"
)

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
    st.session_state.coord_type = "GCJ-02"  # 默认高德/百度坐标
if 'point_a' not in st.session_state:
    st.session_state.point_a = None
if 'point_b' not in st.session_state:
    st.session_state.point_b = None
if 'map_center' not in st.session_state:
    # 南京工业大学示例坐标（GCJ-02）
    st.session_state.map_center = [32.2322, 118.749]

# 侧边栏导航
with st.sidebar:
    st.title("🚁 无人机系统")
    st.markdown("---")
    
    # 页面导航
    st.subheader("📱 功能页面")
    page = st.radio(
        "",
        ["🗺️ 航线规划", "📡 飞行监控"],
        label_visibility="collapsed"
    )
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
    
    # 显示当前坐标系统状态
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
    st.markdown("设置起降点、障碍区，规划安全航线")
    st.markdown("---")
    
    # 左右两列布局
    col_left, col_right = st.columns([1.5, 1])
    
    with col_left:
        st.subheader("🗺️ 3D地图")
        
        # 创建地图
        m = folium.Map(
            location=st.session_state.map_center,
            zoom_start=17,
            control_scale=True
        )
        
        # 添加3D效果（使用带高度的标记）
        # 绘制已保存的障碍区
        for i, obstacle in enumerate(st.session_state.obstacles):
            if len(obstacle) >= 3:
                # 转换坐标用于显示
                display_obs = obstacle
                if st.session_state.coord_type == "GCJ-02":
                    display_obs = [CoordConverter.gcj02_to_wgs84(p[0], p[1]) for p in obstacle]
                
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
            
            # 添加航点标记
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
        
        # A点设置
        st.markdown("### 🚁 起点 A")
        col1, col2 = st.columns(2)
        with col1:
            lat_a = st.number_input("纬度", value=32.2322, format="%.6f")
        with col2:
            lon_a = st.number_input("经度", value=118.749, format="%.6f")
        
        if st.button("📍 设置 A 点", use_container_width=True):
            st.session_state.point_a = [lat_a, lon_a]
            st.success(f"起点已设置: ({lat_a}, {lon_a})")
            st.rerun()
        
        # B点设置
        st.markdown("### 🎯 终点 B")
        col1, col2 = st.columns(2)
        with col1:
            lat_b = st.number_input("纬度", value=32.2343, key="lat_b", format="%.6f")
        with col2:
            lon_b = st.number_input("经度", value=118.754, key="lon_b", format="%.6f")
        
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
        
        # 显示已保存的障碍区
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
                    # 转换坐标（如果需要）
                    start = st.session_state.point_a
                    end = st.session_state.point_b
                    
                    # 如果输入是GCJ-02，先转换为WGS-84用于计算
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

# ==================== 页面2: 飞行监控（含心跳包） ====================
else:
    st.title("📡 飞行监控")
    st.markdown("实时监控无人机飞行状态和心跳信号")
    st.markdown("---")
    
    # 初始化心跳监控器
    if 'heartbeat_monitor' not in st.session_state:
        st.session_state.heartbeat_monitor = HeartbeatMonitor()
    
    # 左右布局
    col_left, col_right = st.columns([1, 1])
    
    with col_left:
        st.subheader("🚁 飞行状态")
        
        # 飞行控制
        if st.session_state.flight_plan:
            col1, col2, col3 = st.columns(3)
            with col1:
                if st.button("▶️ 开始飞行", use_container_width=True, type="primary"):
                    st.session_state.is_flying = True
                    st.session_state.simulator = DroneSimulator(
                        st.session_state.flight_plan['waypoints'],
                        15
                    )
                    st.session_state.start_time = datetime.now()
                    st.session_state.heartbeat_monitor.start()
                    st.rerun()
            
            with col2:
                if st.button("⏸️ 暂停", use_container_width=True):
                    st.session_state.is_flying = False
            
            with col3:
                if st.button("🛑 终止", use_container_width=True):
                    st.session_state.is_flying = False
                    st.session_state.simulator = None
                    st.session_state.heartbeat_monitor.stop()
                    st.rerun()
        
        # 飞行仪表盘
        if st.session_state.get('is_flying') and st.session_state.get('simulator'):
            status = st.session_state.simulator.get_status()
            elapsed = (datetime.now() - st.session_state.start_time).total_seconds()
            
            # 更新心跳
            st.session_state.heartbeat_monitor.update()
            heartbeat_status = st.session_state.heartbeat_monitor.get_status()
            
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
                st.metric("🔋 电量", f"{max(0, 100 - elapsed/6):.0f}%")
            with col3:
                st.metric("💓 心跳", f"{heartbeat_status['heartbeat_rate']}/min")
            with col4:
                last_time = heartbeat_status['last_heartbeat_time']
                if last_time:
                    last_sec = (datetime.now() - last_time).total_seconds()
                    st.metric("⏰ 最后心跳", f"{last_sec:.1f}s前")
                else:
                    st.metric("⏰ 最后心跳", "无")
            
            # 进度条
            st.progress(int(status['progress']))
            
            # 更新无人机位置
            if status['position']:
                st.session_state.drone_pos = status['position']
            
            # 自动刷新
            if status['progress'] >= 100:
                st.success("✅ 飞行完成！")
                st.session_state.is_flying = False
                st.session_state.heartbeat_monitor.stop()
            else:
                # 更新模拟器
                st.session_state.simulator.update(0.1)
                time.sleep(0.1)
                st.rerun()
        else:
            st.info("点击「开始飞行」启动监控")
    
    with col_right:
        st.subheader("💓 心跳信号监控")
        
        # 心跳数据显示
        if st.session_state.heartbeat_monitor:
            # 获取心跳数据
            heartbeat_data = st.session_state.heartbeat_monitor.get_data_for_display()
            
            # 心跳统计
            col1, col2 = st.columns(2)
            with col1:
                st.metric("📤 发送总数", heartbeat_data['stats']['total_sent'])
            with col2:
                st.metric("📥 接收总数", heartbeat_data['stats']['total_received'])
            
            col1, col2 = st.columns(2)
            with col1:
                st.metric("⚠️ 超时次数", heartbeat_data['stats']['timeouts'])
            with col2:
                success_rate = heartbeat_data['stats']['success_rate']
                st.metric("✅ 成功率", f"{success_rate:.1f}%")
            
            # 最新心跳记录
            st.markdown("---")
            st.markdown("### 📋 最新心跳记录")
            
            recent = heartbeat_data['recent_heartbeats']
            if recent:
                df = pd.DataFrame(recent)
                df_display = df[['seq', 'send_time', 'receive_time', 'delay']]
                df_display.columns = ['序号', '发送时间', '接收时间', '延迟(秒)']
                st.dataframe(df_display, use_container_width=True, height=300)
            else:
                st.info("等待心跳信号...")
            
            # 心跳延迟图表
            if len(heartbeat_data['receive_log']) > 0:
                st.markdown("---")
                st.markdown("### 📈 心跳延迟趋势")
                
                df_delay = pd.DataFrame(heartbeat_data['receive_log'][-30:])
                fig = px.line(df_delay, x='seq', y='delay',
                             title="心跳延迟实时监控",
                             labels={'seq': '心跳序号', 'delay': '延迟(秒)'})
                fig.add_hline(y=heartbeat_data['stats']['avg_delay'],
                             line_dash="dash", line_color="red",
                             annotation_text=f"平均: {heartbeat_data['stats']['avg_delay']:.3f}s")
                st.plotly_chart(fig, use_container_width=True)
