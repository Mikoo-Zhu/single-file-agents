import time
import numpy as np
import yaml
import os

# 假设模块可以被正确导入 (可能需要调整 PYTHONPATH 或使用相对导入)
try:
    from core.controller import RobotController, DummyRobotModel
    from simulation.pybullet_env import PybulletEnv
    # from hardware.ethercat_driver import EthercatDriver # 暂不使用硬件
except ImportError as e:
    print(f"Import Error: {e}")
    print("Please ensure the script is run from the project root or adjust PYTHONPATH.")
    print("Example: export PYTHONPATH=$PYTHONPATH:$(pwd)")
    exit(1)


def load_config(config_path="config/robot_config.yaml"):
    """加载 YAML 配置文件。"""
    # 获取脚本所在的目录 (即 industrial_robot 目录)
    script_dir = os.path.dirname(os.path.abspath(__file__))
    full_config_path = os.path.join(script_dir, config_path) # 配置文件相对于脚本目录
    print(f"Loading configuration from: {full_config_path}")
    try:
        with open(full_config_path, 'r') as f:
            config = yaml.safe_load(f)
        print("Configuration loaded successfully.")
        return config
    except FileNotFoundError:
        print(f"Error: Configuration file not found at {full_config_path}")
        return None
    except yaml.YAMLError as e:
        print(f"Error parsing YAML configuration file: {e}")
        return None

def main():
    """主函数，运行 Sim2Real 机器人仿真。"""

    # --- 加载配置 ---
    config = load_config()
    if config is None:
        print("Exiting due to configuration error.")
        return

    # --- 从配置中获取参数 ---
    robot_urdf = config.get('robot_model', {}).get('urdf_path', 'kuka_iiwa/model.urdf')
    control_frequency = config.get('controller', {}).get('frequency_hz', 500)
    simulation_time_step = config.get('simulation', {}).get('time_step', 1./240.)
    simulation_duration = config.get('simulation', {}).get('duration_seconds', 10)
    use_gui = config.get('simulation', {}).get('use_gui', True)
    # 获取控制器增益 (如果需要传递给控制器)
    # kp = config.get('controller', {}).get('kp', 50.0)
    # kd = config.get('controller', {}).get('kd', 10.0)

    print("\n--- Using Configuration ---")
    print(f"Robot URDF: {robot_urdf}")
    print(f"Control Frequency: {control_frequency} Hz")
    print(f"Simulation Time Step: {simulation_time_step} s")
    print(f"Simulation Duration: {simulation_duration} s")
    print(f"Use GUI: {use_gui}")
    print("---------------------------\n")


    # --- 初始化 ---
    print("Initializing simulation environment...")
    try:
        env = PybulletEnv(
            robot_urdf_path=robot_urdf,
            time_step=simulation_time_step,
            use_gui=use_gui
        )
    except ValueError as e:
        print(f"Failed to initialize PyBullet environment: {e}")
        print("Please ensure the URDF path is correct and PyBullet is installed.")
        return
    except Exception as e: # 捕获其他可能的 PyBullet 错误
         print(f"An unexpected error occurred during PyBullet initialization: {e}")
         return


    print("Initializing robot model and controller...")
    # 使用 DummyRobotModel，因为它不需要实际的运动学/动力学计算
    # 实际项目中需要替换为真实的机器人模型类
    # TODO: 可以将配置中的 joint_limits 等传递给 RobotModel
    robot_model = DummyRobotModel(num_joints=env.num_movable_joints)
    # TODO: 可以将 kp, kd 等增益传递给控制器初始化
    controller = RobotController(robot_model=robot_model, control_freq=control_frequency)

    # 获取初始状态并设置控制器
    initial_q, initial_dq = env.get_robot_state()
    controller.update_state(initial_q, initial_dq)
    print(f"Initial robot state: q={initial_q}, dq={initial_dq}")

    # --- 仿真循环 ---
    print(f"Starting simulation loop for {simulation_duration} seconds...")
    start_time = time.time()
    last_control_time = start_time
    control_dt = 1.0 / control_frequency

    num_simulation_steps = int(simulation_duration / simulation_time_step)
    actual_steps = 0

    try:
        for step in range(num_simulation_steps):
            current_time = time.time()

            # --- 控制器更新 (以 control_frequency 运行) ---
            if current_time - last_control_time >= control_dt:
                last_control_time = current_time

                # 1. 获取当前状态
                current_q, current_dq = env.get_robot_state()
                controller.update_state(current_q, current_dq)

                # 2. 规划/设定目标 (示例：保持初始位置)
                # 在实际应用中，这里会根据任务调用 plan_motion
                target_q = initial_q # 保持初始位置
                target_dq = np.zeros_like(target_q)
                target_ddq = np.zeros_like(target_q)

                # 3. 计算控制指令
                torque_command = controller.compute_control_command(
                    target_q, target_dq, target_ddq
                )

                # 4. 应用指令到仿真
                env.apply_joint_torques(torque_command)

            # --- 仿真步进 (以 simulation_time_step 运行) ---
            env.step_simulation()
            actual_steps += 1

            # 如果使用 GUI，稍微延迟以便观察
            if use_gui:
                time.sleep(simulation_time_step / 2.0) # 减慢一点以便观察

            # 检查是否提前退出 (例如，用户关闭 GUI)
            # if use_gui and not p.isConnected(): break

            # 检查是否超时
            if time.time() - start_time > simulation_duration + 1: # 加一点buffer
                 print("Simulation duration exceeded.")
                 break

    except KeyboardInterrupt:
        print("Simulation interrupted by user.")
    except Exception as e:
        print(f"An error occurred during the simulation loop: {e}")
    finally:
        # --- 清理 ---
        print(f"Simulation finished after {actual_steps * simulation_time_step:.2f} simulated seconds.")
        env.close()
        print("Environment closed.")


if __name__ == '__main__':
    main()
