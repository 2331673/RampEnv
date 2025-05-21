class History:
    def __init__(self):
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

    def record(self, current_time, planned_speeds, actual_speeds, planned_gaps, actual_gaps):
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
        if count_speed > 0:
            avg_speed_error = total_speed_error / count_speed * 100  # 百分比
            self.speed_errors.append(avg_speed_error)
            self.history['speed_errors'].append((current_time, avg_speed_error))

        # 记录间距误差
        total_gap_error = 0
        count_gap = 0
        for vehicle_pair, planned_gap in planned_gaps.items():
            if vehicle_pair in actual_gaps:
                actual_gap = actual_gaps[vehicle_pair]
                error = abs(actual_gap - planned_gap) / max(0.1, planned_gap)
                total_gap_error += error
                count_gap += 1
        if count_gap > 0:
            avg_gap_error = total_gap_error / count_gap * 100  # 百分比
            self.gap_errors.append(avg_gap_error)
            self.history['gap_errors'].append((current_time, avg_gap_error))

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
            pair_key = f"{vehicle_pair[0]}_{vehicle_pair[1]}"
            if pair_key not in self.history['planned_gaps']:
                self.history['planned_gaps'][pair_key] = []
            self.history['planned_gaps'][pair_key].append((current_time, planned_gap))
            if vehicle_pair in actual_gaps:
                if pair_key not in self.history['actual_gaps']:
                    self.history['actual_gaps'][pair_key] = []
                self.history['actual_gaps'][pair_key].append((current_time, actual_gaps[vehicle_pair]))

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
        return avg_speed_error, avg_gap_error