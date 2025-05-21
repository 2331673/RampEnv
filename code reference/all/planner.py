from highway_env.vehicle.behavior import IDMVehicle

class Planner:
    def __init__(self, delay_model):
        self.delay_model = delay_model
        self.delay_estimation_zone = (50, 80)
        self.control_zone = (80, 130)
        self.merging_zone = (130, 150)

    def correct_vehicle_state(self, vehicle, env):
        """根据通信延迟修正车辆状态"""
        vehicle_id = id(vehicle)
        position = vehicle.position[0]
        speed = vehicle.speed
        # 调用acceleration
        if isinstance(vehicle, IDMVehicle):
            front_vehicle, rear_vehicle = vehicle.road.neighbour_vehicles(vehicle, vehicle.lane_index)
            acceleration = vehicle.acceleration(ego_vehicle=vehicle, front_vehicle=front_vehicle, rear_vehicle=rear_vehicle)
        else:
            acceleration = getattr(vehicle, '_last_acceleration', 0.0)
        delay = self.delay_model.get_delay(vehicle_id)
        corrected_position = position + speed * delay + 0.5 * acceleration * delay**2
        corrected_speed = speed + acceleration * delay
        return {
            'position': corrected_position,
            'speed': corrected_speed,
            'acceleration': acceleration
        }

    def calculate_safe_distance(self, follower_speed, leader_speed):
        """计算安全跟车距离"""
        min_gap = 8.0
        time_gap = 1.8
        speed_term = follower_speed * time_gap # 跟车需要的反应距离
        rel_speed_term = max(0, follower_speed - leader_speed) * 1.0
        safe_distance = min_gap + speed_term + rel_speed_term
        return safe_distance

    def plan_trajectory(self, merging_sequence, env):
        """规划所有车辆的速度和间距，考虑通信延迟
        返回: planned_speeds, planned_gaps
        """
        planned_speeds = {}
        planned_gaps = {}
        for i, vehicle in enumerate(merging_sequence):
            vehicle_id = id(vehicle)
            corrected_state = self.correct_vehicle_state(vehicle, env)
            target_speed = 30.0  # 默认目标速度
            leader = None
            if i > 0:
                leader = merging_sequence[i-1]
            if leader:
                leader_id = id(leader)
                leader_state = self.correct_vehicle_state(leader, env)
                safe_distance = self.calculate_safe_distance(
                    corrected_state['speed'], 
                    leader_state['speed']
                )
                actual_distance = leader_state['position'] - corrected_state['position'] - leader.LENGTH
                if actual_distance < safe_distance * 0.8:
                    target_speed = leader_state['speed'] * 0.95
                elif actual_distance > safe_distance * 1.2:
                    target_speed = min(leader_state['speed'] * 1.05, 35.0)
                else:
                    target_speed = leader_state['speed']
                planned_gaps[(leader_id, vehicle_id)] = safe_distance
            target_speed = max(25.0, min(35.0, target_speed))
            planned_speeds[vehicle_id] = target_speed
        return planned_speeds, planned_gaps

    def calculate_merging_acceleration(self, vehicle, leader, follower, distance_to_merge):
        """计算汇入加速度"""
        time_to_merge = distance_to_merge / max(0.1, vehicle.speed)
        leader_future_pos = leader.position[0] + leader.speed * time_to_merge
        follower_future_pos = follower.position[0] + follower.speed * time_to_merge
        ideal_position = follower_future_pos + follower.LENGTH + self.calculate_safe_distance(follower.speed, vehicle.speed) + vehicle.LENGTH * 0.5
        max_allowed_position = leader_future_pos - leader.LENGTH - self.calculate_safe_distance(vehicle.speed, leader.speed)
        ideal_position = min(ideal_position, max_allowed_position)
        required_end_speed = (ideal_position - vehicle.position[0]) / time_to_merge # 插入的时候的理想末速度
        return (required_end_speed - vehicle.speed) / time_to_merge