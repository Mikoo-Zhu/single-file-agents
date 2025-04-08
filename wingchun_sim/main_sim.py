import pybullet as p
import pybullet_data
import time
import os
import numpy as np

try:
    from human_model import HumanModel, WoodenDummy
except ImportError:
    print("Error: Could not import HumanModel or WoodenDummy.")
    print("Ensure human_model.py is in the same directory or adjust PYTHONPATH.")
    exit(1)

def main():
    """主函数，运行咏春拳基础仿真。"""

    # --- 配置 ---
    use_gui = True
    simulation_time_step = 1./240.
    simulation_duration = 60 # seconds

    # 获取脚本所在目录，用于查找 URDF 文件
    script_dir = os.path.dirname(os.path.abspath(__file__))

    # --- 初始化 PyBullet ---
    if use_gui:
        physics_client = p.connect(p.GUI)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.resetDebugVisualizerCamera(cameraDistance=2.0, cameraYaw=45, cameraPitch=-30, cameraTargetPosition=[0.5,0,0.8])
    else:
        physics_client = p.connect(p.DIRECT)

    # 连接后才能设置搜索路径
    p.setAdditionalSearchPath(script_dir) # 添加当前目录到搜索路径
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(simulation_time_step)

    # 加载地面
    plane_id = p.loadURDF("plane.urdf")

    # --- 加载模型 ---
    print("Loading models...")
    try:
        # 使用绝对路径加载URDF文件
        human_urdf = os.path.join(script_dir, "human_14j.urdf")
        dummy_urdf = os.path.join(script_dir, "wooden_dummy.urdf")
        
        human = HumanModel(base_pos=[0, 0, 0.9]) # 调整初始高度
        dummy = WoodenDummy(base_pos=[0.6, 0, 0]) # 调整木人桩位置
        print("Models loaded successfully.")
    except p.error as e:
        print(f"Error loading URDF models: {e}")
        print(f"Human URDF path: {human_urdf}")
        print(f"Dummy URDF path: {dummy_urdf}")
        p.disconnect()
        return

    # 设置初始姿态
    human.set_wingchun_stance()
    print("Set initial Wing Chun stance.")

    # --- 仿真循环 ---
    print(f"Starting simulation loop for {simulation_duration} seconds...")
    start_time = time.time()
    actual_steps = 0

    try:
        for step in range(int(simulation_duration / simulation_time_step)):
            # 咏春核心动作序列
            if step < 100:
                # 日字冲拳
                p.setJointMotorControl2(human.body, 1, p.POSITION_CONTROL, targetPosition=np.deg2rad(170))  # 右肘伸直
                p.setJointMotorControl2(human.body, 0, p.POSITION_CONTROL, targetPosition=np.deg2rad(0))    # 右肩前伸
            elif 100 <= step < 200:
                # 摊手
                p.setJointMotorControl2(human.body, 1, p.POSITION_CONTROL, targetPosition=np.deg2rad(45))   # 右肘弯曲
                p.setJointMotorControl2(human.body, 0, p.POSITION_CONTROL, targetPosition=np.deg2rad(-30)) # 右肩外展
            elif 200 <= step < 300:
                # 膀手
                p.setJointMotorControl2(human.body, 1, p.POSITION_CONTROL, targetPosition=np.deg2rad(120)) # 右肘抬高
                p.setJointMotorControl2(human.body, 0, p.POSITION_CONTROL, targetPosition=np.deg2rad(30))   # 右肩内收
            elif 300 <= step < 400:
                # 伏手
                p.setJointMotorControl2(human.body, 1, p.POSITION_CONTROL, targetPosition=np.deg2rad(60))  # 右肘弯曲
                p.setJointMotorControl2(human.body, 0, p.POSITION_CONTROL, targetPosition=np.deg2rad(-15))  # 右肩下沉
            elif 400 <= step < 500:
                # 左摊手
                p.setJointMotorControl2(human.body, 3, p.POSITION_CONTROL, targetPosition=np.deg2rad(45))  # 左肘弯曲
                p.setJointMotorControl2(human.body, 2, p.POSITION_CONTROL, targetPosition=np.deg2rad(-30)) # 左肩外展
            elif 500 <= step < 600:
                # 左右连环冲拳
                if step % 50 < 25:
                    # 右冲拳
                    p.setJointMotorControl2(human.body, 1, p.POSITION_CONTROL, targetPosition=np.deg2rad(170))
                    p.setJointMotorControl2(human.body, 3, p.POSITION_CONTROL, targetPosition=np.deg2rad(45))
                else:
                    # 左冲拳
                    p.setJointMotorControl2(human.body, 3, p.POSITION_CONTROL, targetPosition=np.deg2rad(170))
                    p.setJointMotorControl2(human.body, 1, p.POSITION_CONTROL, targetPosition=np.deg2rad(45))

            p.stepSimulation()
            actual_steps += 1

            if use_gui:
                time.sleep(simulation_time_step)

            # 检查是否超时
            if time.time() - start_time > simulation_duration + 1:
                 print("Simulation duration exceeded.")
                 break

    except KeyboardInterrupt:
        print("Simulation interrupted by user.")
    except p.error as e:
        print(f"An error occurred during the simulation loop: {e}")
    finally:
        # --- 清理 ---
        print(f"Simulation finished after {actual_steps * simulation_time_step:.2f} simulated seconds.")
        p.disconnect()
        print("PyBullet environment closed.")


if __name__ == '__main__':
    main()
