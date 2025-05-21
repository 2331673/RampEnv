class Executor:
    def __init__(self, env, planner, vehicle_manager, dt, merging_zone):
        self.env = env
        self.planner = planner
        self.vehicle_manager = vehicle_manager
        self.dt = dt
        self.merging_zone = merging_zone

    def apply_control(self, planned_speeds, actual_speeds):
        for vehicle in self.env.road.vehicles:
            vehicle_id = id(vehicle)
            if vehicle_id in planned_speeds:
                target_speed = planned_speeds[vehicle_id]
                # 最低速度约束
                if vehicle.lane_index[0] == "a":
                    min_speed = self.planner.MAINLINE_MIN_SPEED
                elif vehicle.lane_index[0] == "j":
                    min_speed = self.planner.RAMP_MIN_SPEED
                else:
                    min_speed = 0
                target_speed = max(min_speed, min(35.0, target_speed))
                vehicle.target_speed = target_speed
                actual_speeds[vehicle_id] = vehicle.speed
