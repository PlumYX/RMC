import pybullet as p
import pybullet_data
import time

# 连接到物理引擎
physicsClient = p.connect(p.GUI)  # 使用 GUI 模式，默认是 CPU 渲染, 无图形界面 p.DIRECT

# 添加资源路径
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# 设置重力
p.setGravity(0, 0, -10)

# 加载地面
planeId = p.loadURDF("plane.urdf")

# 创建一个立方体
cubeStartPos = [0, 0, 1]
cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
boxId = p.loadURDF("r2d2.urdf", cubeStartPos, cubeStartOrientation)
# boxId = p.loadURDF("first_robot.urdf", cubeStartPos, cubeStartOrientation)

# 机械臂信息
num_joints = p.getNumJoints(boxId)
print(f"关节数量: {num_joints}")
# for i in range(num_joints):
#     joint_info = p.getNumJoints(boxId, i)
#     print(f"关节 {i} 信息: {joint_info}")

# 运行模拟
while True:
    p.stepSimulation()
    time.sleep(1./240.)  # 控制模拟速度

# 断开连接
p.disconnect()
