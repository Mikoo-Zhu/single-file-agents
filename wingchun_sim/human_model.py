import pybullet as p
import numpy as np

class HumanModel:
    """简化的人体生物力学模型（14关节）"""
    def __init__(self, base_pos=[0,0,0]):
        self.body = p.loadURDF(
            "human_14j.urdf",  # 需要准备简化的人体URDF文件
            basePosition=base_pos,
            useFixedBase=True
        )
        self.joint_indices = list(range(p.getNumJoints(self.body)))
        self._setup_joint_limits()
        
    def _setup_joint_limits(self):
        """设置咏春拳特有的关节限制"""
        # 肩关节限制（保持沉肩坠肘）
        p.changeDynamics(self.body, 0, jointLowerLimit=-0.5, jointUpperLimit=0.5)  # 右肩
        p.changeDynamics(self.body, 2, jointLowerLimit=-0.5, jointUpperLimit=0.5)  # 左肩
        
        # 肘关节限制（保持曲中有直）
        p.changeDynamics(self.body, 1, jointLowerLimit=np.deg2rad(30), jointUpperLimit=np.deg2rad(150))  # 右肘
        p.changeDynamics(self.body, 3, jointLowerLimit=np.deg2rad(30), jointUpperLimit=np.deg2rad(150))  # 左肘
        
        # 髋关节限制（二字钳羊马）
        p.changeDynamics(self.body, 4, jointLowerLimit=-0.3, jointUpperLimit=0.3)  # 右髋
        p.changeDynamics(self.body, 6, jointLowerLimit=-0.3, jointUpperLimit=0.3)  # 左髋
        
        # 膝关节限制（微屈）
        p.changeDynamics(self.body, 5, jointLowerLimit=np.deg2rad(30), jointUpperLimit=np.deg2rad(90))  # 右膝
        p.changeDynamics(self.body, 7, jointLowerLimit=np.deg2rad(30), jointUpperLimit=np.deg2rad(90))  # 左膝


    def set_wingchun_stance(self):
        """设置二字钳羊马基本姿势"""
        # 下肢关节角度设置
        knee_angle = np.deg2rad(55)
        p.resetJointState(self.body, 5, knee_angle)  # 右膝
        p.resetJointState(self.body, 7, knee_angle)  # 左膝
        
        # 髋关节内扣角度
        hip_angle = np.deg2rad(15)
        p.resetJointState(self.body, 4, -hip_angle)  # 右髋
        p.resetJointState(self.body, 6, hip_angle)   # 左髋
        
        # 脊柱直立
        p.resetJointState(self.body, 8, 0)  # 颈部

class WoodenDummy:
    """传统红木木人桩模型"""
    def __init__(self, base_pos=[0.6,0,0.8]):
        self.dummy = p.loadURDF(
            "wooden_dummy.urdf",
            basePosition=base_pos,
            globalScaling=1.2  # 标准尺寸调整
        )
        # 设置红木材质的物理属性 (摩擦力、恢复系数)
        # 质量和惯性由 URDF 的 <inertial> 标签定义
        # 遍历所有 link (包括 base link，索引为 -1)
        num_joints = p.getNumJoints(self.dummy)
        for link_index in range(-1, num_joints):
            p.changeDynamics(self.dummy, link_index,
                lateralFriction=0.4, # 木材的典型摩擦系数
                restitution=0.2      # 木材较低的恢复系数 (非弹性碰撞)
                # spinningFriction=0.001, # 可选：旋转摩擦
                # rollingFriction=0.001   # 可选：滚动摩擦
            )
