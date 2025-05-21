import gymnasium
import highway_env
from matplotlib import pyplot as plt
import imageio

# 创建 merge-v1 环境
env = gymnasium.make(
    "merge-v1",
    render_mode="rgb_array",
    config ={
        "observation": {"type": "Kinematics"},
        "action": {"type": "DiscreteMetaAction"},
        "simulation_frequency": 5,
        "policy_frequency": 1,
        "other_vehicles_type": "highway_env.vehicle.behavior.IDMVehicle",
        "screen_width": 600,
        "screen_height": 150,
        "centering_position": [0.3, 0.5],
        "scaling": 5.5,
        "show_trajectories": False,
        "render_agent": True,
        "offscreen_rendering": True
    }
)

# 重置环境
obs, info = env.reset()

frames = []

# 仿真循环
for _ in range(20):
    action = env.unwrapped.action_type.actions_indexes["IDLE"]
    obs, reward, done, truncated, info = env.step(action)
    frame = env.render()
    if frame is not None:
        frames.append(frame)


if frames:
    imageio.mimsave("result.gif", frames, fps=20)
    # 可选：显示最后一帧
    plt.imshow(frames[-1])
    plt.title("Ramp Merging Simulation (merge-v1)")
    plt.show()
else:
    print("没有渲染帧可保存。")