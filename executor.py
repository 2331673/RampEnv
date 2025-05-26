class Executor:
    def __init__(self, env, planner, vehicle_manager, dt, merging_zone):
        self.env = env
        self.planner = planner
        self.vehicle_manager = vehicle_manager
        self.dt = dt
        self.merging_zone = merging_zone
        self.planned_speeds_history = []  # 新增：用于接收历史
        self.planned_gaps_history = []    # 新增：用于接收间距历史

    def apply_control(self, planned_speeds, actual_speeds):
        for vehicle in self.env.road.vehicles:
            vehicle_id = id(vehicle)
            # 获取延迟步数
            delay = self.planner.delay_model.get_delay(vehicle_id)
            delay_steps = int(delay / self.dt)
            # 取历史planned_speeds
            history_idx = len(self.planned_speeds_history) - 1 - delay_steps
            target_speed = getattr(vehicle, "target_speed", vehicle.speed)
            if self.planned_speeds_history and 0 <= history_idx < len(self.planned_speeds_history):
                history_dict = self.planned_speeds_history[history_idx]
                if vehicle_id in history_dict:
                    target_speed = history_dict[vehicle_id]
            vehicle.target_speed = target_speed
            actual_speeds[vehicle_id] = vehicle.speed

        actual_gaps = {}
        vehicles = sorted(self.env.road.vehicles, key=lambda v: v.position[0], reverse=True)
        
        for i in range(len(vehicles) - 1):
            v1 = vehicles[i]
            for j in range(i + 1, len(vehicles)):
                v2 = vehicles[j]
                # Only calculate gaps for vehicles in the same lane
                if v1.lane_index[0] == v2.lane_index[0] and v1.lane_index[2] == v2.lane_index[2]:
                    gap = abs(v1.position[0] - v2.position[0])
                    if gap < 100:  # Only record gaps smaller than 100m
                        pair_key = (id(v1), id(v2))
                        actual_gaps[pair_key] = gap
                    break  # Only consider the closest following vehicle
        
        # Store the actual gaps
        self.actual_gaps = actual_gaps
        
        # Return both actual speeds and gaps
        return actual_speeds, actual_gaps