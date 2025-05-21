class Executor:
    def __init__(self, env, planner, vehicle_manager, dt, merging_zone):
        self.env = env
        self.planner = planner
        self.vehicle_manager = vehicle_manager
        self.dt = dt
        self.merging_zone = merging_zone

    def apply_control(self, planned_speeds, planned_gaps, actual_speeds):
        for vehicle in self.env.road.vehicles:
            vehicle_id = id(vehicle)
            # 检查前方车辆，实施紧急避撞
            leader = self.vehicle_manager.find_leader(vehicle)
            if leader:
                distance = leader.position[0] - vehicle.position[0] - leader.LENGTH
                min_safe_distance = self.planner.calculate_safe_distance(vehicle.speed, leader.speed)
                if distance < min_safe_distance * 0.7:
                    deceleration = -3.5
                    new_speed = max(0, vehicle.speed + deceleration * self.dt)
                    vehicle.target_speed = new_speed
                    continue
            if vehicle_id in planned_speeds:
                target_speed = planned_speeds[vehicle_id]
                speed_diff = target_speed - vehicle.speed
                if abs(speed_diff) > 0.1:
                    if speed_diff > 0:
                        acceleration = min(0.8, speed_diff / 2.0)
                    else:
                        acceleration = max(-2.5, speed_diff / 0.5)
                    new_speed = vehicle.speed + acceleration * self.dt
                    new_speed = max(0, min(35, new_speed))
                    vehicle.target_speed = new_speed
                else:
                    vehicle.target_speed = target_speed
                actual_speeds[vehicle_id] = vehicle.speed

    def ramp_vehicle_control(self, vehicle):
        if vehicle.lane_index[0] != "j":
            return
        distance_to_merge = self.merging_zone[0] - vehicle.position[0]
        mainline_vehicles = self.vehicle_manager.get_relevant_mainline_vehicles(self.merging_zone)
        leader, follower = self.vehicle_manager.find_gap_vehicles(mainline_vehicles, self.merging_zone, distance_to_merge)
        if leader and follower:
            gap_size = leader.position[0] - follower.position[0] - leader.LENGTH
            required_gap = (vehicle.LENGTH +
                            self.planner.calculate_safe_distance(follower.speed, vehicle.speed) +
                            self.planner.calculate_safe_distance(vehicle.speed, leader.speed))
            if gap_size > required_gap:
                target_speed = follower.speed * 1.05
                if distance_to_merge < 30:
                    acceleration = self.planner.calculate_merging_acceleration(vehicle, leader, follower, distance_to_merge)
                    new_speed = vehicle.speed + acceleration * self.dt
                    vehicle.target_speed = max(15, min(new_speed, 35))
                else:
                    vehicle.target_speed = min(target_speed, 35)
            else:
                deceleration = -1.5
                if distance_to_merge < 15:
                    deceleration = -3.0
                new_speed = max(5, vehicle.speed + deceleration * self.dt)
                vehicle.target_speed = new_speed

    def warn_mainline_vehicles(self, ramp_vehicle):
        for vehicle in self.env.road.vehicles:
            if vehicle.lane_index[0] == "a":
                distance_to_merge = self.merging_zone[0] - vehicle.position[0]
                if 0 < distance_to_merge < 30:
                    vehicle.target_speed = max(vehicle.target_speed * 0.95, 25.0)