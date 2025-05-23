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
    
    fig, axes = plt.subplots(4, 1, figsize=(12, 16))

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

    for vehicle_id in history['actual_speeds']:
        if vehicle_id not in coordinated_ids:
            continue  # 只画被协调控制过的车辆

        # 真实速度
        actual = [v[1] if isinstance(v, tuple) else v for v in history['actual_speeds'][vehicle_id]]
        t_actual = list(range(len(actual)))

        # 规划速度
        planned = history['planned_speeds'].get(vehicle_id, [])
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

        # 绘制实际速度和延迟规划速度
        axes[1].plot(t_actual, actual, '-', label=f"Actual {vehicle_id % 10000}")
        axes[1].plot(
            [i for i, v in enumerate(delayed_planned) if v is not None],
            [v for v in delayed_planned if v is not None],
            '--', label=f"DelayedPlan {vehicle_id % 10000}"
        )

        # 绘制规划速度和延迟规划速度
        axes[2].plot(t_planned, planned_vals, '-', label=f"Planned {vehicle_id % 10000}")
        axes[2].plot(
            [i for i, v in enumerate(delayed_planned) if v is not None],
            [v for v in delayed_planned if v is not None],
            '--', label=f"DelayedPlan {vehicle_id % 10000}"
        )

    axes[1].legend(loc='upper right', fontsize=8)
    axes[2].legend(loc='upper right', fontsize=8)

    # 3. 误差分析
    axes[3].set_title("Error Analysis")
    axes[3].set_xlabel("Time Step")
    axes[3].set_ylabel("Error Rate (%)")
    speed_errors = [e[1] * 100 if isinstance(e, tuple) else e * 100 for e in history['speed_errors']]
    gap_errors = [e[1] * 100 if isinstance(e, tuple) else e * 100 for e in history['gap_errors']]
    time = list(range(len(speed_errors)))
    min_len = min(len(time), len(speed_errors), len(gap_errors))
    axes[3].plot(time[:min_len], speed_errors[:min_len], 'r-', label="Speed Error (%)")
    axes[3].plot(time[:min_len], gap_errors[:min_len], 'b-', label="Gap Error (%)")
    axes[3].axhline(y=15, color='r', linestyle='--', label="15% Threshold")
    axes[3].legend(loc='upper right')

    # 显示平均误差
    avg_speed_error, avg_gap_error = controller.history.get_average_errors()
    fig.text(0.5, 0.02, 
            f"Average Speed Error: {avg_speed_error:.2f}%\nAverage Gap Error: {avg_gap_error:.2f}%",
            ha='center', fontsize=12, 
            bbox=dict(facecolor='white', edgecolor='black', boxstyle='round,pad=0.5'))

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