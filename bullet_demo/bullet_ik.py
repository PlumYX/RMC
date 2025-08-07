import time
import math
from datetime import datetime

import pybullet as p
import pybullet_data


# 创建服务端
clid = p.connect(p.SHARED_MEMORY)
if clid < 0:
    # 服务端打开图形GUI做渲染, 需要独显, 性能消耗大
    p.connect(p.GUI)
    # p.connect(p.DIRECT)
    # p.connect(p.SHARED_MEMORY_GUI)  # 不打开图形渲染, 性能消耗小


# 渲染设置(可选)
# p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)      # 是否渲染, 否: 0, 是: 1
# p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)            # 是否打开控件, 否: 0, 是: 1
# p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)  # 是否使用核显渲染, 否: 0, 是: 1


# 在环境变量中添加资源路径, 后面加载机器人就不用写绝对路径了
p.setAdditionalSearchPath(pybullet_data.getDataPath())
print(pybullet_data.getDataPath())


# 设置 x, y, z 轴重力
p.setGravity(0, 0, -9.8)


# 加载 PLANE 场景和 KUKA 机器人
scenceId = p.loadURDF("plane.urdf", [0, 0, -0.65])
# 加载机械臂
kukaId = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0])
p.resetBasePositionAndOrientation(kukaId, [0, 0, 0], [0, 0, 0, 1])
# 加载桌子
tableUid = p.loadURDF("table/table.urdf", basePosition=[0.5, 0, -0.65])

kukaEndEffectorIndex = 6


# KUKA 机械臂版本验证
numJoints = p.getNumJoints(kukaId)
if (numJoints != 7):
    exit()


# 零空间的下界
ll = [-.967, -2, -2.96, 0.19, -2.96, -2.09, -3.05]
# 零空间的上界
ul = [.967, 2, 2.96, 2.29, 2.96, 2.09, 3.05]
# 零空间的联合范围
jr = [5.8, 4, 5.8, 4, 5.8, 4, 6]
# 零空间的静止位姿
rp = [0, 0, 0, 0.5 * math.pi, 0, -math.pi * 0.5 * 0.66, 0]
# 关节阻尼系数
jd = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]

for i in range(numJoints):
    p.resetJointState(kukaId, i, rp[i])

t = 0.
prevPose = [0, 0, 0]
prevPose1 = [0, 0, 0]
hasPrevPose = 0
useNullSpace = 1

useOrientation = 1
# 如果将 useSimulation 设为 0, 则直接将机械臂姿态设置为逆运动学(IK)解算结果, 而不使用动态控制.
# 这可以用来测试逆运动学结果的准确性.
useSimulation = 1
useRealTimeSimulation = 0
ikSolver = 0
p.setRealTimeSimulation(useRealTimeSimulation)
# trailDuration 是调试线段在自动删除前的持续时间(以秒为单位).
# 使用 0 表示不删除(永久保留).
trailDuration = 15

i = 0
while 1:
    i += 1
    # p.getCameraImage(
    #     320,
    #     200,
    #     flags=p.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX,
    #     renderer=p.ER_BULLET_HARDWARE_OPENGL,
    #     )
    if useRealTimeSimulation:
        dt = datetime.now()
        t = (dt.second / 60.) * 2. * math.pi
    else:
        t = t + 0.01

    if useSimulation and useRealTimeSimulation == 0:
        p.stepSimulation()

    for i in range(1):
        pos = [-0.4, 0.2 * math.cos(t), 0. + 0.2 * math.sin(t)]
        # 末端执行器指向下方, 而不是上方(在 useOrientation == 1 的情况下).
        orn = p.getQuaternionFromEuler([0, -math.pi, 0])

    if useNullSpace == 1:
        if useOrientation == 1:
            jointPoses = p.calculateInverseKinematics(
                kukaId, 
                kukaEndEffectorIndex, 
                pos, orn, 
                ll, ul, jr, rp, 
                )
        else:
            jointPoses = p.calculateInverseKinematics(
                kukaId, 
                kukaEndEffectorIndex, 
                pos, 
                lowerLimits = ll, 
                upperLimits = ul, 
                jointRanges = jr, 
                restPoses = rp, 
                )
    else:
        if useOrientation == 1:
            jointPoses = p.calculateInverseKinematics(
                kukaId, 
                kukaEndEffectorIndex, 
                pos, 
                orn, 
                jointDamping = jd, 
                solver = ikSolver, 
                maxNumIterations = 100, 
                residualThreshold = .01, 
                )
        else:
            jointPoses = p.calculateInverseKinematics(
                kukaId, 
                kukaEndEffectorIndex, 
                pos, 
                solver = ikSolver, 
                )

    if useSimulation:
        for i in range(numJoints):
            p.setJointMotorControl2(
                bodyIndex = kukaId, 
                jointIndex = i, 
                controlMode = p.POSITION_CONTROL, 
                targetPosition = jointPoses[i], 
                targetVelocity = 0, 
                force = 500, 
                positionGain = 0.03, 
                velocityGain = 1, 
                )
    else:
        # 重置关节状态(忽略所有动力学, 不建议在仿真过程中使用).
        for i in range(numJoints):
            p.resetJointState(kukaId, i, jointPoses[i])

    ls = p.getLinkState(kukaId, kukaEndEffectorIndex)
    if hasPrevPose:
        p.addUserDebugLine(prevPose, pos, [0, 0, 0.3], 1, trailDuration)
        p.addUserDebugLine(prevPose1, ls[4], [1, 0, 0], 1, trailDuration)
    prevPose = pos
    prevPose1 = ls[4]
    hasPrevPose = 1

p.disconnect()
