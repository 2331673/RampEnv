class Executor:
    def __init__(self, env, planner, vehicle_manager, dt, merging_zone):
        self.env = env
        self.planner = planner
        self.vehicle_manager = vehicle_manager
        self.dt = dt
        self.merging_zone = merging_zone

    def apply_control(self, planned_speeds,actual_speeds):
        for vehicle in self.env.road.vehicles:
            vehicle_id = id(vehicle)
            # 检查前方车辆，实施紧急避撞
            leader = self.vehicle_manager.find_leader(vehicle)
            if leader:
                distance = leader.position[0] - vehicle.position[0] - leader.LENGTH
                # 动态安全时距调整
                in_merging_zone = vehicle.position[0] >= self.merging_zone[0]
                time_gap = 1.5 if in_merging_zone else 1.5
                min_safe_distance = self.planner.calculate_safe_distance(vehicle.speed, leader.speed, time_gap)
                if distance < min_safe_distance * 0.7:
                    deceleration = -3.5
                    new_speed = max(0, vehicle.speed + deceleration * self.dt)
                    # 最低速度约束
                    if vehicle.lane_index[0] == "a":
                        min_speed = self.planner.MAINLINE_MIN_SPEED
                    elif vehicle.lane_index[0] == "j":
                        min_speed = self.planner.RAMP_MIN_SPEED
                    else:
                        min_speed = 0
                    vehicle.target_speed = max(new_speed, min_speed)
                    continue
            if vehicle_id in planned_speeds:
                target_speed = planned_speeds[vehicle_id]
                # 最低速度约束
                if vehicle.lane_index[0] == "a":
                    min_speed = self.planner.MAINLINE_MIN_SPEED
                elif vehicle.lane_index[0] == "j":
                    min_speed = self.planner.RAMP_MIN_SPEED
                else:
                    min_speed = 0
                target_speed = max(target_speed, min_speed)
                speed_diff = target_speed - vehicle.speed
                if abs(speed_diff) > 0.1:
                    if speed_diff > 0:
                        acceleration = min(0.8, speed_diff / 2.0)
                    else:
                        acceleration = max(-2.5, speed_diff / 0.5)
                    new_speed = vehicle.speed + acceleration * self.dt
                    new_speed = max(min_speed, min(35, new_speed))
                    vehicle.target_speed = new_speed
                else:
                    vehicle.target_speed = target_speed
                actual_speeds[vehicle_id] = vehicle.speed
