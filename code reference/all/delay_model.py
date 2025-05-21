import numpy as np

class DelayModel:
    def __init__(self, comm_delay_mean=0.025, comm_delay_std=0.012):
        """初始化延迟模型"""
        self.comm_delay_mean = comm_delay_mean  # 25ms均值
        self.comm_delay_std = comm_delay_std    # 12ms标准差
        self.vehicle_timestamps = {}  # 记录车辆时间戳
        self.estimated_delays = {}    # 估计的通信延迟

    def update_delay_estimation(self, vehicles, time_step, dt, zone, history):
        """更新通信延迟估计
        vehicles: 当前所有车辆列表
        time_step: 当前时间步
        dt: 步长
        zone: (start, end) 延迟估计区域
        history: 历史记录字典
        """
        current_time = time_step * dt

        for vehicle in vehicles:
            vehicle_id = id(vehicle)
            # 车辆在延迟估计区域
            if self._is_in_zone(vehicle, zone):
                # 首次进入区域，记录时间戳
                if vehicle_id not in self.vehicle_timestamps:
                    self.vehicle_timestamps[vehicle_id] = []
                # 模拟车辆发送带时间戳的消息
                vehicle_timestamp = current_time
                # 模拟通信延迟 (车辆到控制器)
                v2c_delay = max(0, np.random.normal(self.comm_delay_mean, self.comm_delay_std))
                # 模拟接收时间
                receive_time = current_time + v2c_delay
                # 存储时间戳数据
                self.vehicle_timestamps[vehicle_id].append({
                    'sent_time': vehicle_timestamp,
                    'receive_time': receive_time,
                    'v2c_delay': v2c_delay
                })
                # 保留最近的N个时间戳进行统计
                if len(self.vehicle_timestamps[vehicle_id]) > 10:
                    self.vehicle_timestamps[vehicle_id].pop(0)
                # 估计总通信延迟 (车辆到控制器 + 控制器到车辆)
                if len(self.vehicle_timestamps[vehicle_id]) >= 3:
                    delays = [data['v2c_delay'] for data in self.vehicle_timestamps[vehicle_id]]
                    mean_delay = np.mean(delays)
                    total_delay = 2 * mean_delay  # 控制器到车辆延迟假设与车辆到控制器相同
                    self.estimated_delays[vehicle_id] = total_delay
                    # 更新历史
                    if vehicle_id not in history['estimated_delays']:
                        history['estimated_delays'][vehicle_id] = []
                    history['estimated_delays'][vehicle_id].append((current_time, total_delay))

    def _is_in_zone(self, vehicle, zone):
        """判断车辆是否在指定区域"""
        return zone[0] <= vehicle.position[0] <= zone[1]

    def get_delay(self, vehicle_id):
        """获取车辆的估计通信延迟，默认0.05s"""
        return self.estimated_delays.get(vehicle_id, 0.05)