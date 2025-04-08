import pybullet as p
import pybullet_data
import time
import numpy as np

class PybulletEnv:
    """
    基于 PyBullet 的机器人仿真环境。
    负责加载机器人模型、场景，运行物理仿真，并提供状态信息。
    """
    def __init__(self, robot_urdf_path, time_step=1./240., use_gui=True):
        """
        初始化 PyBullet 仿真环境。

        Args:
            robot_urdf_path (str): 机器人 URDF 文件的路径。
            time_step (float): 仿真步长。
            use_gui (bool): 是否启动图形用户界面。
        """
        self.time_step = time_step
        self.robot_urdf_path = robot_urdf_path

        if use_gui:
            self.physics_client = p.connect(p.GUI)
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0) # 禁用默认的GUI控件
            p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=30, cameraPitch=-30, cameraTargetPosition=[0,0,0.5])
        else:
            self.physics_client = p.connect(p.DIRECT)

        p.setAdditionalSearchPath(pybullet_data.getDataPath()) # 用于加载 plane.urdf 等
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(self.time_step)

        # 加载地面
        self.plane_id = p.loadURDF("plane.urdf")

        # 加载机器人
        self.robot_id = self.load_robot()

        if self.robot_id is None:
             raise ValueError("Failed to load robot URDF.")

        self.num_joints = p.getNumJoints(self.robot_id)
        self.joint_indices = [i for i in range(self.num_joints) if p.getJointInfo(self.robot_id, i)[2] != p.JOINT_FIXED]
        self.num_movable_joints = len(self.joint_indices)

        print(f"PyBullet environment initialized. Robot ID: {self.robot_id}, Num Movable Joints: {self.num_movable_joints}")


    def load_robot(self, base_position=[0, 0, 0], base_orientation=[0, 0, 0, 1]):
        """
        加载机器人模型到仿真环境中。

        Args:
            base_position (list): 机器人基座的初始位置。
            base_orientation (list): 机器人基座的初始方向（四元数）。

        Returns:
            int: 加载的机器人模型的 ID，如果失败则返回 None。
        """
        try:
            robot_id = p.loadURDF(
                self.robot_urdf_path,
                basePosition=base_position,
                baseOrientation=p.getQuaternionFromEuler(base_orientation[:3]) # 确保是四元数
            )
            print(f"Robot loaded from: {self.robot_urdf_path}")
            return robot_id
        except p.error as e:
            print(f"Error loading robot URDF: {e}")
            print(f"Please ensure the path is correct: {self.robot_urdf_path}")
            return None


    def step_simulation(self):
        """执行一步仿真。"""
        p.stepSimulation()
        # time.sleep(self.time_step) # 在非GUI模式或需要精确时间控制时可能需要

    def apply_joint_torques(self, torques):
        """
        将计算出的力矩应用到机器人的关节上。

        Args:
            torques (np.ndarray or list): 应用到每个可动关节的力矩。
                                         长度应等于 self.num_movable_joints。
        """
        if len(torques) != self.num_movable_joints:
            print(f"Error: Number of torques ({len(torques)}) does not match number of movable joints ({self.num_movable_joints}).")
            return

        p.setJointMotorControlArray(
            bodyUniqueId=self.robot_id,
            jointIndices=self.joint_indices,
            controlMode=p.TORQUE_CONTROL,
            forces=torques
        )

    def get_robot_state(self):
        """
        获取机器人当前的关节状态。

        Returns:
            tuple: (joint_positions, joint_velocities)
                   joint_positions (np.ndarray): 当前关节角度。
                   joint_velocities (np.ndarray): 当前关节速度。
        """
        joint_states = p.getJointStates(self.robot_id, self.joint_indices)
        joint_positions = np.array([state[0] for state in joint_states])
        joint_velocities = np.array([state[1] for state in joint_states])
        return joint_positions, joint_velocities

    def reset_robot_state(self, joint_positions):
         """
         重置机器人的关节状态。

         Args:
             joint_positions (list or np.ndarray): 要设置的目标关节位置。
         """
         if len(joint_positions) != self.num_movable_joints:
             print(f"Error: Length of joint_positions ({len(joint_positions)}) does not match number of movable joints ({self.num_movable_joints}).")
             return

         for i, joint_index in enumerate(self.joint_indices):
             p.resetJointState(self.robot_id, joint_index, targetValue=joint_positions[i], targetVelocity=0.0)
         print(f"Robot state reset to: {joint_positions}")


    def close(self):
        """关闭仿真环境。"""
        p.disconnect(self.physics_client)
        print("PyBullet environment closed.")

# 示例用法
if __name__ == '__main__':
    # 重要提示：需要一个有效的 URDF 文件路径才能运行此示例
    # 您可以从 pybullet_data 或其他来源获取示例 URDF，例如 'kuka_iiwa/model.urdf'
    # 或者创建一个简单的 URDF 文件
    # urdf_path = "path/to/your/robot.urdf" # <--- 修改这里！
    urdf_path = "r2d2.urdf" # 使用 pybullet_data 中的示例

    try:
        env = PybulletEnv(robot_urdf_path=urdf_path, use_gui=True)

        # 获取初始状态
        q, dq = env.get_robot_state()
        print("Initial state:", q, dq)

        # 运行仿真一段时间
        print("Running simulation...")
        for i in range(1000):
            # 施加零力矩（或计算出的力矩）
            torques = np.zeros(env.num_movable_joints)
            env.apply_joint_torques(torques)
            env.step_simulation()
            if i % 100 == 0:
                 q, dq = env.get_robot_state()
                 print(f"Step {i}, q: {np.round(q, 2)}")
            time.sleep(1./240.) # 保持与仿真步长同步

        # 重置状态示例
        reset_q = np.random.rand(env.num_movable_joints) * 0.5 # 随机小角度
        env.reset_robot_state(reset_q)
        time.sleep(1) # 等待视觉更新

        print("Simulation finished.")

    except ValueError as e:
         print(f"Initialization failed: {e}")
    except p.error as e:
         print(f"PyBullet error during simulation: {e}")
         print("Did you provide a valid URDF path?")
    finally:
        if 'env' in locals() and env.physics_client >= 0: # 检查连接是否仍然有效
             env.close()
