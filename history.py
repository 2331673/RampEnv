import logging
import os
from datetime import datetime
class History:
    def __init__(self, log_dir="logs"):
        self.history = {
            'time': [],
            'planned_speeds': {},
            'actual_speeds': {},
            'planned_gaps': {},
            'actual_gaps': {},
            'speed_errors': [],
            'gap_errors': [],
            'estimated_delays': {}
        }
        self.speed_errors = []
        self.gap_errors = []

        # 设置日志
        self._setup_logger(log_dir)

    def _setup_logger(self, log_dir):
        """设置日志记录器"""
        # 创建日志目录
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
        
        # 创建带时间戳的日志文件名
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_file = os.path.join(log_dir, f"simulation_{timestamp}.log")
        
        # 配置日志记录器
        self.logger = logging.getLogger("VehicleCoordination")
        self.logger.setLevel(logging.INFO)
        
        # 文件处理器
        file_handler = logging.FileHandler(log_file)
        file_handler.setLevel(logging.INFO)
        
        # 格式化器
        formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
        file_handler.setFormatter(formatter)
        
        # 添加处理器
        self.logger.addHandler(file_handler)
        
        self.logger.info("Simulation started")
        self.logger.info("History logger initialized")
    
    def record(self, current_time, planned_speeds, actual_speeds, planned_gaps, actual_gaps):
        self.logger.info(f"Recording data at time: {current_time}")
        self.history['time'].append(current_time)

        # 记录速度误差
        total_speed_error = 0
        count_speed = 0
        for vehicle_id, planned_speed in planned_speeds.items():
            if vehicle_id in actual_speeds:
                actual_speed = actual_speeds[vehicle_id]
                error = abs(actual_speed - planned_speed) / max(0.1, planned_speed)
                total_speed_error += error
                count_speed += 1
                self.logger.debug(f"Vehicle {vehicle_id}: Planned speed={planned_speed:.2f}, Actual speed={actual_speed:.2f}, Error={error*100:.2f}%")

        if count_speed > 0:
            avg_speed_error = total_speed_error / count_speed * 100  # 百分比
            avg_speed_error = min(avg_speed_error, 200) 
            self.speed_errors.append(avg_speed_error)
            self.history['speed_errors'].append((current_time, avg_speed_error))
            self.logger.info(f"Average speed error at time {current_time}: {avg_speed_error:.2f}%")

        # 记录间距误差
        total_gap_error = 0
        count_gap = 0
        for vehicle_pair, planned_gap in planned_gaps.items():
            if vehicle_pair in actual_gaps:
                actual_gap = actual_gaps[vehicle_pair]
                if planned_gap < 0.5 or actual_gap < 0.5:  # 间距太小，可能是异常值
                    continue
                error = abs(actual_gap - planned_gap) / max(0.1, planned_gap)
                total_gap_error += error
                count_gap += 1
        if count_gap > 0:
            avg_gap_error = total_gap_error / count_gap * 100  # 百分比
            avg_gap_error = min(avg_gap_error, 200) 
            self.gap_errors.append(avg_gap_error)
            self.history['gap_errors'].append((current_time, avg_gap_error))
            self.logger.info(f"Average gap error at time {current_time}: {avg_gap_error:.2f}%")
            
        # 记录速度和间距计划/实际值
        for vehicle_id, planned_speed in planned_speeds.items():
            if vehicle_id not in self.history['planned_speeds']:
                self.history['planned_speeds'][vehicle_id] = []
            self.history['planned_speeds'][vehicle_id].append((current_time, planned_speed))
            if vehicle_id in actual_speeds:
                if vehicle_id not in self.history['actual_speeds']:
                    self.history['actual_speeds'][vehicle_id] = []
                self.history['actual_speeds'][vehicle_id].append((current_time, actual_speeds[vehicle_id]))
        for vehicle_pair, planned_gap in planned_gaps.items():
            # Convert tuple to string key
            pair_key = f"{vehicle_pair[0]}_{vehicle_pair[1]}"
            if pair_key not in self.history['planned_gaps']:
                self.history['planned_gaps'][pair_key] = []
            self.history['planned_gaps'][pair_key].append((current_time, planned_gap))
        
        for vehicle_pair, actual_gap in actual_gaps.items():
            # Convert tuple to string key
            pair_key = f"{vehicle_pair[0]}_{vehicle_pair[1]}"
            if pair_key not in self.history['actual_gaps']:
                self.history['actual_gaps'][pair_key] = []
            self.history['actual_gaps'][pair_key].append((current_time, actual_gap))

    def calculate_errors(self, planned_speeds, actual_speeds, planned_gaps, actual_gaps):
        # 速度误差
        speed_errors = []
        for vehicle_id, planned_speed in planned_speeds.items():
            if vehicle_id in actual_speeds and planned_speed > 0:
                actual_speed = actual_speeds[vehicle_id]
                relative_error = abs(planned_speed - actual_speed) / planned_speed
                speed_errors.append(relative_error)
        # 间距误差
        gap_errors = []
        for vehicle_pair, planned_gap in planned_gaps.items():
            if vehicle_pair in actual_gaps and planned_gap > 0:
                actual_gap = actual_gaps[vehicle_pair]
                relative_error = abs(planned_gap - actual_gap) / planned_gap
                gap_errors.append(relative_error)
        avg_speed_error = sum(speed_errors) / len(speed_errors) if speed_errors else 0
        avg_gap_error = sum(gap_errors) / len(gap_errors) if gap_errors else 0
        self.speed_errors.append(avg_speed_error)
        self.gap_errors.append(avg_gap_error)
        return avg_speed_error, avg_gap_error

    def get_average_errors(self):
        avg_speed_error = sum(self.speed_errors) / len(self.speed_errors) if self.speed_errors else 0
        avg_gap_error = sum(self.gap_errors) / len(self.gap_errors) if self.gap_errors else 0
        self.logger.info(f"Final average speed error: {avg_speed_error:.2f}%, gap error: {avg_gap_error:.2f}%")
        return avg_speed_error, avg_gap_error
    
    def log_simulation_end(self, total_frames):
        """记录模拟结束的信息"""
        self.logger.info(f"Simulation completed with {total_frames} frames")
        avg_speed_error, avg_gap_error = self.get_average_errors()
        self.logger.info(f"Final results - Average Speed Error: {avg_speed_error:.2f}%, Average Gap Error: {avg_gap_error:.2f}%")