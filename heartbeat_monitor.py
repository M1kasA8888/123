import time
import datetime
import threading
import queue
from typing import Dict, List, Optional

class HeartbeatMonitor:
    """心跳信号监控器"""
    
    def __init__(self):
        self.sequence_number = 0
        self.send_log = []
        self.receive_log = []
        self.timeout_log = []
        self.send_queue = queue.Queue()
        self.running = False
        self.send_thread = None
        self.receive_thread = None
        self.last_heartbeat_time = None
        self.heartbeat_count = 0
        
    def start(self):
        """启动心跳监控"""
        if not self.running:
            self.running = True
            self.send_thread = threading.Thread(target=self._send_loop, daemon=True)
            self.receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
            self.send_thread.start()
            self.receive_thread.start()
    
    def stop(self):
        """停止心跳监控"""
        self.running = False
        if self.send_thread:
            self.send_thread.join(timeout=1)
        if self.receive_thread:
            self.receive_thread.join(timeout=1)
    
    def _send_loop(self):
        """发送心跳循环"""
        while self.running:
            self._send_heartbeat()
            time.sleep(1)
    
    def _receive_loop(self):
        """接收心跳循环"""
        while self.running:
            try:
                received = self.send_queue.get(timeout=3)
                receive_time = datetime.datetime.now()
                received['receive_time'] = receive_time
                received['delay'] = round((receive_time - received['send_time']).total_seconds(), 3)
                self.receive_log.append(received)
                self.last_heartbeat_time = receive_time
                self.heartbeat_count += 1
            except queue.Empty:
                # 超时
                timeout_time = datetime.datetime.now()
                self.timeout_log.append({
                    'timeout_time': timeout_time,
                    'message': '连接超时: 3秒内未收到心跳'
                })
    
    def _send_heartbeat(self):
        """发送单个心跳"""
        self.sequence_number += 1
        send_time = datetime.datetime.now()
        heartbeat = {
            'seq': self.sequence_number,
            'send_time': send_time,
            'status': 'sent'
        }
        self.send_log.append(heartbeat)
        self.send_queue.put(heartbeat.copy())
    
    def update(self):
        """手动更新（用于非线程模式）"""
        self._send_heartbeat()
        try:
            received = self.send_queue.get(timeout=0.1)
            receive_time = datetime.datetime.now()
            received['receive_time'] = receive_time
            received['delay'] = round((receive_time - received['send_time']).total_seconds(), 3)
            self.receive_log.append(received)
            self.last_heartbeat_time = receive_time
            self.heartbeat_count += 1
        except queue.Empty:
            pass
    
    def get_status(self) -> Dict:
        """获取心跳状态"""
        current_time = datetime.datetime.now()
        last_time = self.last_heartbeat_time
        
        # 计算心跳频率（次/分钟）
        if last_time and len(self.receive_log) > 1:
            time_diff = (current_time - last_time).total_seconds()
            if time_diff > 0:
                heartbeat_rate = 60 / time_diff
            else:
                heartbeat_rate = 60
        else:
            heartbeat_rate = 0
        
        return {
            'heartbeat_rate': round(heartbeat_rate, 1),
            'last_heartbeat_time': last_time,
            'total_sent': len(self.send_log),
            'total_received': len(self.receive_log),
            'timeouts': len(self.timeout_log),
            'is_connected': last_time and (current_time - last_time).total_seconds() < 3
        }
    
    def get_data_for_display(self) -> Dict:
        """获取用于显示的数据"""
        # 计算统计信息
        total_sent = len(self.send_log)
        total_received = len(self.receive_log)
        timeouts = len(self.timeout_log)
        
        avg_delay = 0
        if self.receive_log:
            avg_delay = sum([r['delay'] for r in self.receive_log]) / len(self.receive_log)
        
        # 获取最近10条记录
        recent = []
        for r in self.receive_log[-10:]:
            recent.append({
                'seq': r['seq'],
                'send_time': r['send_time'].strftime("%H:%M:%S"),
                'receive_time': r['receive_time'].strftime("%H:%M:%S"),
                'delay': r['delay']
            })
        
        return {
            'stats': {
                'total_sent': total_sent,
                'total_received': total_received,
                'timeouts': timeouts,
                'success_rate': (total_received / total_sent * 100) if total_sent > 0 else 0,
                'avg_delay': round(avg_delay, 3)
            },
            'receive_log': self.receive_log,
            'recent_heartbeats': recent
        }
