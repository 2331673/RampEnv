import numpy as np

class DelayModel:
    def __init__(self, comm_delay=0.1):
        """初始化延迟模型，所有车辆延迟相同"""
        self.comm_delay = comm_delay  # 所有车辆延迟，单位秒

    def update_delay_estimation(self, vehicles, time_step, dt, zone, history):
        """不再动态估计延迟，直接使用固定延迟"""
        pass  # 保留接口，不做任何操作

    def _is_in_zone(self, vehicle, zone):
        """判断车辆是否在指定区域"""
        return zone[0] <= vehicle.position[0] <= zone[1]

    def get_delay(self, vehicle_id):
        """获取车辆的固定通信延迟"""
        return self.comm_delay