import numpy as np
from highway_env.vehicle.behavior import IDMVehicle
from highway_env import utils

class VehicleManager:
    def __init__(self, env):
        self.env = env

    def detect_vehicles(self):
        """检测主线、匝道和ego车辆"""
        mainline_vehicles = []
        ramp_vehicles = []
        ego_vehicle = self.env.vehicle
        for vehicle in self.env.road.vehicles:
            if vehicle.lane_index[0] == "a":
                mainline_vehicles.append(vehicle)
            elif vehicle.lane_index[0] == "j":
                ramp_vehicles.append(vehicle)
        return mainline_vehicles, ramp_vehicles, ego_vehicle

    def find_leader(self, vehicle):
        """找到给定车辆前方最近的车"""
        min_distance = float('inf')
        leader = None
        for other in self.env.road.vehicles:
            if (other.lane_index[0] == vehicle.lane_index[0] and 
                other.lane_index[1] == vehicle.lane_index[1]) or (
                vehicle.lane_index[0] == "j" and other.lane_index[0] == "a"):
                distance = other.position[0] - vehicle.position[0]
                if distance > 0 and distance < min_distance:
                    min_distance = distance
                    leader = other
        return leader

    def get_relevant_mainline_vehicles(self, merging_zone):
        """获取主路上与汇入相关的车辆"""
        relevant_vehicles = []
        for vehicle in self.env.road.vehicles:
            if vehicle.lane_index[0] == "a":
                distance_to_merge = merging_zone[0] - vehicle.position[0]
                if -50 < distance_to_merge < 50:
                    relevant_vehicles.append(vehicle)
        return relevant_vehicles

    def find_gap_vehicles(self, mainline_vehicles, merging_zone, ramp_vehicle_distance):
        """找到主路上形成间隙的前后车辆"""
        sorted_vehicles = sorted(mainline_vehicles, key=lambda v: merging_zone[0] - v.position[0])
        leader = None
        follower = None
        for vehicle in sorted_vehicles:
            vehicle_distance = merging_zone[0] - vehicle.position[0]
            if vehicle_distance < 0 and abs(vehicle_distance) < abs(ramp_vehicle_distance):
                if not leader or abs(vehicle_distance) < abs(merging_zone[0] - leader.position[0]):
                    leader = vehicle
            if vehicle_distance > 0 and vehicle_distance > ramp_vehicle_distance:
                if not follower or vehicle_distance < merging_zone[0] - follower.position[0]:
                    follower = vehicle
        return leader, follower