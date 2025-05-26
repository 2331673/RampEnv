from delay_model import DelayModel
from vehicle_manager import VehicleManager
from planner import Planner
from executor import Executor
from history import History

class RoadsideController:
    def __init__(self, env, communication_delay_mean=0.1, communication_delay_std=0.0):
        self.env = env.unwrapped
        self.dt = 1.0 / self.env.config["simulation_frequency"]
        self.time_step = 0

        # 实例化各功能模块
        self.delay_model = DelayModel(communication_delay_mean)  # 只传递一个参数
        self.vehicle_manager = VehicleManager(self.env)
        self.planner = Planner(self.delay_model)
        self.executor = Executor(self.env, self.planner, self.vehicle_manager, self.dt, self.planner.merging_zone)
        self.history = History()

        # 运行时数据
        self.merging_sequence = []
        self.planned_speeds = {}
        self.planned_gaps = {}
        self.actual_speeds = {}
        self.actual_gaps = {}

    def update(self):
        self.time_step += 1
        current_time = self.time_step * self.dt

        # 1. 更新通信延迟估计
        self.delay_model.update_delay_estimation(
            self.env.road.vehicles, self.time_step, self.dt, self.planner.delay_estimation_zone, self.history.history
        )

        # 2. 检测车辆
        mainline_vehicles, ramp_vehicles, ego_vehicle = self.vehicle_manager.detect_vehicles()

        # 3. 按预计到达汇入点时间排序
        all_vehicles = mainline_vehicles + ramp_vehicles + [ego_vehicle]
        merging_zone_start = self.planner.merging_zone[0]
        self.merging_sequence = sorted(
            all_vehicles,
            key=lambda v: (merging_zone_start - v.position[0]) / max(0.1, v.speed)
        )

        # 4. 轨迹/速度/间距规划
        self.planned_speeds = self.planner.plan_trajectory(self.merging_sequence, self.env)
        # 新增：同步历史给executor
        self.executor.planned_speeds_history = self.planner.planned_speeds_history

        # 5. 控制执行
        self.executor.apply_control(self.planned_speeds, self.actual_speeds)
        self.actual_gaps = getattr(self.executor, 'actual_gaps', {})

        # 6. 历史记录与误差分析
        self.history.record(
            current_time,
            self.planned_speeds,
            self.actual_speeds,
            self.planner.planned_gaps_history,
            self.actual_gaps
        )

        # 输出ego车辆每步的车道、目标速度、真实速度
        ego_vehicle = self.env.vehicle
        lane = ego_vehicle.lane_index[0] if hasattr(ego_vehicle, "lane_index") else None
        target_speed = getattr(ego_vehicle, "target_speed", None)
        speed = getattr(ego_vehicle, "speed", None)
        print(f"Step {self.time_step}: Ego lane={lane}, target_speed={target_speed:.2f}, actual_speed={speed:.2f}")

    def verify_vehicle_states(self):
        # 调试用，调用 vehicle_manager 的方法打印车辆状态
        mainline_vehicles, ramp_vehicles, ego_vehicle = self.vehicle_manager.detect_vehicles()
        print(f"Mainline: {len(mainline_vehicles)}, Ramp: {len(ramp_vehicles)}, Ego: {id(ego_vehicle) if ego_vehicle else None}")