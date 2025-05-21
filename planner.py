from highway_env.vehicle.behavior import IDMVehicle

class Planner:
    def __init__(self, delay_model):
        self.delay_model = delay_model
        self.delay_estimation_zone = (50, 80)
        self.control_zone = (80, 130)
        # merging_zone的定义：第一个点为jk交汇点，第二个点为kb交汇点
        # 对应merge_env.py中 ends = [150, 80, 80, 150]
        # jk交汇点x = ends[0] = 150，kb交汇点x = ends[0] + ends[1] = 230
        self.merging_zone = (150, 230)
        self.MAINLINE_MIN_SPEED = 50.0/3.6  # 50km/h
        self.RAMP_MIN_SPEED = 20/3.6    # 20km/h

    def correct_vehicle_state(self, vehicle, env):
        vehicle_id = id(vehicle)
        position = vehicle.position[0]
        speed = vehicle.speed
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

    def predict_arrival_time(self, s, v0, v_target, a_max):
        """
        预测车辆从当前位置以最大加速度/减速度加速到目标速度后到达汇入点的时间
        s: 距离汇入点距离
        v0: 当前速度
        v_target: 目标速度
        a_max: 最大加速度（正为加速，负为减速）
        """
        if abs(v_target - v0) < 1e-2 or a_max == 0:
            # 速度变化很小或无加速度，直接匀速
            return s / max(v0, 0.1)
        t_acc = abs((v_target - v0) / a_max)
        s_acc = v0 * t_acc + 0.5 * a_max * t_acc ** 2
        if s_acc >= s:
            # 在到达汇入点前就能加到目标速度
            # 解一元二次方程：0.5*a*t^2 + v0*t - s = 0
            a = 0.5 * a_max
            b = v0
            c = -s
            delta = b ** 2 - 4 * a * c
            if delta < 0:
                return s / max(v0, 0.1)
            t = (-b + delta ** 0.5) / (2 * a)
            return max(t, 0.1)
        else:
            # 先加速到目标速度，再匀速
            t_const = (s - s_acc) / max(v_target, 0.1)
            return t_acc + t_const

    def coordinate_merge(self, mainline_vehicles, ramp_vehicles, env):
        planned_speeds = {}
        if not ramp_vehicles or not mainline_vehicles:
            return planned_speeds
        ramp_vehicle = ramp_vehicles[0]
        ramp_state = self.correct_vehicle_state(ramp_vehicle, env)
        merging_point = self.merging_zone[1]
        mainline_sorted = sorted(
            [v for v in mainline_vehicles if v.position[0] < merging_point],
            key=lambda v: merging_point - v.position[0]
        )
        if not mainline_sorted:
            return planned_speeds
        # 假设最大加速度/减速度
        if isinstance(ramp_vehicle, IDMVehicle):
            RAMP_A_MAX = min(ramp_vehicle.ACC_MAX, ramp_vehicle.COMFORT_ACC_MAX)
            RAMP_A_MIN = max(-ramp_vehicle.ACC_MAX, ramp_vehicle.COMFORT_ACC_MIN)
        else:
            RAMP_A_MAX = 2.0
            RAMP_A_MIN = -2.0
        if isinstance(mainline_sorted[0], IDMVehicle):
            MAIN_A_MAX = min(mainline_sorted[0].ACC_MAX, mainline_sorted[0].COMFORT_ACC_MAX)
            MAIN_A_MIN = max(-mainline_sorted[0].ACC_MAX, mainline_sorted[0].COMFORT_ACC_MIN)
        else:
            MAIN_A_MAX = 2.0
            MAIN_A_MIN = -2.0
        for idx in range(min(5, len(mainline_sorted))):
            main_vehicle = mainline_sorted[idx]
            main_state = self.correct_vehicle_state(main_vehicle, env)
            ramp_dist = merging_point - ramp_state['position']
            main_dist = merging_point - main_state['position']
            # 预测两车到达汇入点的时间（考虑加速度过程）
            ramp_time = self.predict_arrival_time(ramp_dist, ramp_state['speed'], self.RAMP_MIN_SPEED, RAMP_A_MAX)   # 匝道加速
            main_time = self.predict_arrival_time(main_dist, main_state['speed'], self.MAINLINE_MIN_SPEED, MAIN_A_MIN)  # 主路减速
            time_gap = 1.0 if ramp_state['position'] >= self.merging_zone[0] else 1.0
            if (main_time - ramp_time) > time_gap:
                # 匝道车辆以最大加速度提速，但不超过限速
                ramp_target_speed = ramp_state['speed'] + RAMP_A_MAX * ramp_time
                ramp_target_speed = max(self.RAMP_MIN_SPEED, ramp_target_speed)
                # 主干道车辆以最大减速度减速，但不低于限速
                main_target_speed = main_state['speed'] + MAIN_A_MIN * main_time
                main_target_speed = min(self.MAINLINE_MIN_SPEED, main_target_speed)
                planned_speeds[id(ramp_vehicle)] = ramp_target_speed
                planned_speeds[id(main_vehicle)] = main_target_speed
                
                dt = 0.1 # 获取仿真步长，默认0.1s
                planned_speeds[id(ramp_vehicle)] = ramp_state['speed'] + RAMP_A_MAX * dt
                planned_speeds[id(main_vehicle)] = main_state['speed'] + MAIN_A_MIN * dt
                
                break
        return planned_speeds

    def plan_trajectory(self, merging_sequence, env):
        # 车辆分组
        mainline_vehicles = [v for v in merging_sequence if v.lane_index[0] in ["a", "b", "c"]]
        ramp_vehicles = [v for v in merging_sequence if v.lane_index[0] in ["j", "k"]]

        planned_speeds = {}
        dt = 0.1  # 仿真步长

        for vehicle in merging_sequence:
            vehicle_id = id(vehicle)
            # 主干道车辆
            if vehicle.lane_index[0] in ["a", "b", "c"]:
                speed = vehicle.speed
                acc_max = min(getattr(vehicle, "ACC_MAX", 2.0), getattr(vehicle, "COMFORT_ACC_MAX", 2.0))
                acc_min = max(-getattr(vehicle, "ACC_MAX", 2.0), getattr(vehicle, "COMFORT_ACC_MIN", -2.0))
                if speed < 70/3.6:
                    target_speed = speed + acc_max * dt
                else:
                    target_speed = speed + acc_min * dt
                min_speed = self.MAINLINE_MIN_SPEED
            # 匝道车辆
            elif vehicle.lane_index[0] in ["j", "k"]:
                speed = vehicle.speed
                acc_max = min(getattr(vehicle, "ACC_MAX", 2.0), getattr(vehicle, "COMFORT_ACC_MAX", 2.0))
                acc_min = max(-getattr(vehicle, "ACC_MAX", 2.0), getattr(vehicle, "COMFORT_ACC_MIN", -2.0))
                if speed < 30/3.6:
                    target_speed = speed + acc_max * dt
                else:
                    target_speed = speed + acc_min * dt
                min_speed = self.RAMP_MIN_SPEED
            else:
                target_speed = 30.0
                min_speed = 0
            planned_speeds[vehicle_id] = max(min_speed, min(35.0, target_speed))

        # 2. 协调控制，只和主干道上序号为1的车辆交互
        merging_point = self.merging_zone[1]
        mainline_sorted = sorted(
            [v for v in mainline_vehicles if v.position[0] < merging_point],
            key=lambda v: merging_point - v.position[0]
        )
        mainline_for_coord = [mainline_sorted[0]] if mainline_sorted else []
        coord_speeds = self.coordinate_merge(mainline_for_coord, ramp_vehicles, env)
        planned_speeds.update(coord_speeds)

        return planned_speeds