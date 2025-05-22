import random
from highway_env.envs.merge_env import MergeEnv
from highway_env.vehicle.behavior import IDMVehicle
from highway_env import utils

class MergeEnvV1(MergeEnv):
    def _make_vehicles(self) -> None:
        road = self.road
        other_vehicles_type = utils.class_from_path(self.config["other_vehicles_type"])
        # 匝道ego车，速度范围30~40km/h
        ego_speed = self.np_random.uniform(28, 35) / 3.6
        ego_vehicle = other_vehicles_type(
            road, road.network.get_lane(("j", "k", 0)).position(145, 0), speed=ego_speed
        )
        road.vehicles.append(ego_vehicle)
        ego_vehicle.color = (0, 255, 0)
        # 主干道车辆，速度范围70~90km/h
        main_positions = [390, 350, 300, 260, 220, 190, 150, 120, 90, 50, 30, 5]  # 可根据需要调整数量和分布
        i = 0
        for position in main_positions:
            # 原有逻辑：如果i是基数，那么lane为0，否则lane为1
            # if i % 2 == 0:
            #     lane = road.network.get_lane(("a", "b", 0))
            # else:
            #     lane = road.network.get_lane(("a", "b", 1))
            # 新增：随机选择lane为0或1
            # 新增：随机选择lane为0或1
            lane_index = self.np_random.integers(0, 2)
            lane = road.network.get_lane(("a", "b", lane_index))
            pos = lane.position(position + self.np_random.uniform(-5, 5), 0)
            speed = self.np_random.uniform(65, 75) / 3.6  # 70~90 km/h
            road.vehicles.append(other_vehicles_type(road, pos, speed=speed))
            i += 1
        self.vehicle = ego_vehicle

    def step(self, action):
        # 调用父类step
        obs, reward, terminated, truncated, info = super().step(action)

        # 步数计数器
        if not hasattr(self, "frame_count"):
            self.frame_count = 0
        self.frame_count += 1

        # # 第帧时让ego车target_speed为0
        # if self.frame_count == 1:
        #     candidates = [v for v in self.road.vehicles if v is self.vehicle and hasattr(v, "target_speed")]
        #     if candidates:
        #         candidates[0].target_speed = 0

        return obs, reward, terminated, truncated, info