import gymnasium
import highway_env
from controller import RoadsideController
import matplotlib.pyplot as plt
import imageio
from highway_env.envs.merge_v1 import MergeEnvV1
import os

def run_simulation(delay = 0.1):
    env = gymnasium.make(
        "merge-v0",
        render_mode="rgb_array",
        config={
            "observation": {"type": "Kinematics"},
            "action": {"type": "DiscreteMetaAction"},
            "simulation_frequency": 30,
            "policy_frequency": 10,
            "other_vehicles_type": "highway_env.vehicle.behavior.IDMVehicle",
            "screen_width": 800,
            "screen_height": 200,
            "centering_position": [0.3, 0.5],
            "scaling": 5.5,
            "vehicles_count": 8,
            "initial_spacing": 2,
            "controlled_vehicles": 1,
            "ego_spacing": 2,
            "show_trajectories": True,
            "render_agent": True,
            "collision_reward": -5.0,
            "offscreen_rendering": True,
            "normalize_reward": False
        }
    )

    video_folder = os.path.join(os.path.dirname(__file__), "videos")
    env.unwrapped.__class__ = MergeEnvV1
    env = gymnasium.wrappers.RecordVideo(
        env,
        video_folder = video_folder,
        episode_trigger=lambda episode_id: True
    )
    obs, info = env.reset()

    ego_vehicle = env.unwrapped.vehicle
    print(f"Ego vehicle ID: {id(ego_vehicle)}")
    print(f"Ego lane index: {ego_vehicle.lane_index}")
    print(f"Ego position: {ego_vehicle.position}")

    controller = RoadsideController(env, delay)

    # 检查车辆
    mainline_vehicles, ramp_vehicles, controller_ego = controller.vehicle_manager.detect_vehicles()
    print(f"Controller found {len(ramp_vehicles)} ramp vehicles")
    print(f"Controller ego ID: {id(controller_ego) if controller_ego else None}")

    frames = []   
    
    fig, axes = plt.subplots(5, 1, figsize=(12, 20))

    for step in range(400):
        controller.update()
        ego_vehicle = env.unwrapped.vehicle
        # 判断是否接近汇入点
        merging_zone = controller.planner.merging_zone
        if ego_vehicle.lane_index[0] == "j" and ego_vehicle.position[0] > merging_zone[0] - 5:
            action = 2  # LANE_RIGHT
        else:
            action = 0  # IDLE
        obs, reward, done, truncated, info = env.step(action)

        frame = env.render()
        if frame is not None:
            frames.append(frame)

        collision = any(v1.crashed for v1 in env.unwrapped.road.vehicles)
        if collision:
            print(f"WARNING: Collision detected at step {step}")

        if done:
            print(f"Simulation completed at step {step}")
            break

    if frames:
        imageio.mimsave("merge_process.gif", frames, fps=15)

    # 1. 仿真场景
    axes[0].set_title("Last Frame of Merge Simulation")
    if frames:
        axes[0].imshow(frames[-1])

    # 2. 速度对比
    axes[1].set_title("Speed Comparison (Actual vs Delayed Planned)")
    axes[1].set_xlabel("Time Step")
    axes[1].set_ylabel("Speed (m/s)")
    history = controller.history.history

    # 新增：第三个子图，展示规划速度和延迟规划速度
    axes[2].set_title("Planned Speed vs Delayed Planned Speed")
    axes[2].set_xlabel("Time Step")
    axes[2].set_ylabel("Speed (m/s)")

    dt = controller.dt
    planned_speeds_history = controller.planner.planned_speeds_history

    coordinated_ids = controller.planner.coordinated_vehicle_ids
    
    # 跟踪是否已经添加了带有标签的图元
    has_label_axis1 = False
    has_label_axis2 = False

    for vehicle_id in history['actual_speeds']:
        if vehicle_id not in coordinated_ids:
            continue  # 只画被协调控制过的车辆

        # 真实速度
        actual = [v[1] if isinstance(v, tuple) else v for v in history['actual_speeds'][vehicle_id]]
        t_actual = list(range(len(actual)))

        # 规划速度
        planned = history['planned_speeds'].get(vehicle_id, [])
        if not planned:  # 如果没有规划速度数据，跳过
            continue
            
        planned_vals = [v[1] if isinstance(v, tuple) else v for v in planned]
        t_planned = list(range(len(planned_vals)))

        # 延迟规划速度
        delay = controller.delay_model.get_delay(vehicle_id)
        delay_steps = int(delay / dt)
        delayed_planned = []
        for i in range(len(actual)):
            idx = i - delay_steps
            if idx >= 0 and idx < len(planned_vals):
                delayed_planned.append(planned_vals[idx])
            else:
                delayed_planned.append(None)

        # 筛选有效数据点
        valid_delayed_indices = [i for i, v in enumerate(delayed_planned) if v is not None]
        valid_delayed_values = [v for v in delayed_planned if v is not None]

        # 绘制实际速度和延迟规划速度
        if len(actual) > 0:
            axes[1].plot(t_actual, actual, '-', label=f"Actual {vehicle_id % 10000}")
            has_label_axis1 = True
        
        if len(valid_delayed_values) > 0:
            axes[1].plot(valid_delayed_indices, valid_delayed_values, '--', 
                       label=f"DelayedPlan {vehicle_id % 10000}")
            has_label_axis1 = True

        # 绘制规划速度和延迟规划速度
        if len(planned_vals) > 0:
            axes[2].plot(t_planned, planned_vals, '-', label=f"Planned {vehicle_id % 10000}")
            has_label_axis2 = True
        
        if len(valid_delayed_values) > 0:
            axes[2].plot(valid_delayed_indices, valid_delayed_values, '--', 
                       label=f"DelayedPlan {vehicle_id % 10000}")
            has_label_axis2 = True

    # 只在有带标签图元时添加图例
    if has_label_axis1:
        axes[1].legend(loc='upper right', fontsize=8)
    if has_label_axis2:
        axes[2].legend(loc='upper right', fontsize=8)

        # Plot the gaps between vehicles
    for pair_key in history.get('actual_gaps', {}):
        # 首先提取车辆ID
        if isinstance(pair_key, str):
            pair_parts = pair_key.split('_')
            v1_id = int(pair_parts[0])
            v2_id = int(pair_parts[1])
        else:
            v1_id, v2_id = pair_key
        
        # 然后创建planned_key
        planned_key = f"{v1_id}_{v2_id}"
        
        # 获取实际间距数据
        actual_gaps = history['actual_gaps'][pair_key]
        
        # 检查是否有对应的计划间距数据
        if planned_key in history.get('planned_gaps', {}):
            planned_gaps = history['planned_gaps'][planned_key]
            
            # 计算间距误差，加入有效性检查
            for i in range(min(len(actual_gaps), len(planned_gaps))):
                actual_gap = actual_gaps[i][1] if isinstance(actual_gaps[i], tuple) else actual_gaps[i]
                planned_gap = planned_gaps[i][1] if isinstance(planned_gaps[i], tuple) else planned_gaps[i]
                
                # 关键检查：确保间距有效
                if actual_gap > 0 and actual_gap < 500:  # 使用合理阈值，如500米
                    gap_error = abs(planned_gap - actual_gap) / actual_gap * 100
                    # 存储或使用gap_error...
                else:
                    # 跳过异常值，或记录但不计入平均值
                    continue
    # 3. 误差分析
    axes[3].set_title("Error Analysis")
    axes[3].set_xlabel("Time Step")
    axes[3].set_ylabel("Error Rate (%)")
    speed_errors = [e[1] if isinstance(e, tuple) else e for e in history['speed_errors']]
    gap_errors = [e[1] if isinstance(e, tuple) else e for e in history['gap_errors']]
    time = list(range(len(speed_errors)))
    min_len = min(len(time), len(speed_errors), len(gap_errors))
    axes[3].plot(time[:min_len], speed_errors[:min_len], 'r-', label="Speed Error (%)")
    axes[3].plot(time[:min_len], gap_errors[:min_len], 'b-', label="Gap Error (%)")
    axes[3].axhline(y=15, color='r', linestyle='--', label="15% Threshold")
    axes[3].legend(loc='upper right')

     # 显示平均误差
    avg_speed_error, avg_gap_error = controller.history.get_average_errors()
    axes[3].text(0.05, 0.95, 
              f"Avg Speed Error: {avg_speed_error:.2f}%\nAvg Gap Error: {avg_gap_error:.2f}%",
           transform=axes[3].transAxes, fontsize=10, 
           bbox=dict(facecolor='white', alpha=0.7),
           verticalalignment='top')
    
        # 4. 路侧车辆间距指导
    axes[4].set_title("Roadside Vehicle Spacing Guidance")
    axes[4].set_xlabel("Time Step")
    axes[4].set_ylabel("Gap (meters)")

    # 添加理想安全间距参考线
    safety_gap = 30  # 安全间距值
    axes[4].axhline(y=safety_gap, color='g', linestyle='-.', linewidth=1.5, 
                label="Recommended Safe Gap")

    # 颜色方案
    color_list = ['blue', 'red', 'green', 'orange', 'purple', 'brown', 'pink', 'cyan']
    vehicle_pair_colors = {}
    color_index = 0

    # 记录已添加的图例标签
    has_label_axis4 = False
    added_labels = set()

    # 根据planner中的coordinated_vehicle_ids识别被协调的车辆
    def is_coordinated_vehicle(vehicle_id):
        return vehicle_id in controller.planner.coordinated_vehicle_ids if hasattr(controller, 'planner') else False

    # 循环处理所有车辆对
    for pair_key in history.get('actual_gaps', {}):
        # 提取车辆ID
        if isinstance(pair_key, str):
            pair_parts = pair_key.split('_')
            v1_id = int(pair_parts[0])
            v2_id = int(pair_parts[1])
        else:
            v1_id, v2_id = pair_key
        
        # 判断是否为被协调控制的车辆对
        is_coordinated = is_coordinated_vehicle(v1_id) and is_coordinated_vehicle(v2_id)
        
        # 只处理被协调控制的车辆对，其他的忽略
        if not is_coordinated:
            continue
        
        # 创建计划间距的键
        planned_key = f"{v1_id}_{v2_id}"
        
        # 为车辆对分配一个唯一的颜色
        if pair_key not in vehicle_pair_colors:
            vehicle_pair_colors[pair_key] = color_list[color_index % len(color_list)]
            color_index += 1
        
        pair_color = vehicle_pair_colors[pair_key]
        
        # 获取实际间距数据
        actual_gaps = history['actual_gaps'][pair_key]
        t_gap = list(range(len(actual_gaps)))
        gap_values = [g[1] if isinstance(g, tuple) else g for g in actual_gaps]
        
        # 短ID用于显示
        v1_short_id = v1_id % 10000
        v2_short_id = v2_id % 10000
        pair_label = f"{v1_short_id}-{v2_short_id}"
        
        # 先检查计划间距数据是否存在
        planned_gap_data = []
        if planned_key in history.get('planned_gaps', {}):
            planned_gap_data = history['planned_gaps'][planned_key]

        # 然后再计算max_time
        max_time = max(len(actual_gaps), len(planned_gap_data))
        t_common = list(range(max_time))

        # 绘制实际间距
        if len(gap_values) > 0:
            label = f"Actual Gap {pair_label}"
            if label not in added_labels:
                axes[4].plot(t_common[:len(gap_values)], gap_values, '-', linewidth=2, 
                color=pair_color, label=f"Actual Gap {pair_label}")
                added_labels.add(label)
                has_label_axis4 = True
            else:
                axes[4].plot(t_common[:len(gap_values)], gap_values, '-', linewidth=2, 
                color=pair_color)

        # 绘制计划间距（如果有）
        if planned_key in history.get('planned_gaps', {}):
            planned_values = [g[1] if isinstance(g, tuple) else g for g in planned_gap_data]
            
            if len(planned_values) > 0:
                label = f"Guided Gap {pair_label}"
                if label not in added_labels:
                    axes[4].plot(t_common[:len(planned_values)], planned_values, '--', linewidth=2, 
                    color=pair_color, label=f"Planned Gap {pair_label}")
                    added_labels.add(label)
                    has_label_axis4 = True
                else:
                    axes[4].plot(t_common[:len(planned_values)], planned_values, '--', linewidth=2, 
                    color=pair_color)

    # 只在有带标签图元时添加图例
    if has_label_axis4:
        axes[4].legend(loc='upper right', fontsize=8)

    axes[4].grid(True)

    # 设置合理的y轴范围
    axes[4].set_ylim(0, 100)


    
    plt.tight_layout()
    plt.savefig("merge_analysis.png", dpi=300)
    plt.show()

    env.close()
    
    print(f"Simulation completed with {len(frames)} frames.")
    print(f"Average Speed Error: {avg_speed_error:.2f}%")
    print(f"Average Gap Error: {avg_gap_error:.2f}%")

    return controller, frames

if __name__ == "__main__":
    controller, frames = run_simulation(0.1)
