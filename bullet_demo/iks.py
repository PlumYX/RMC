import numpy as np
import copy


class IKSolver(object):
    def __init__(self, forward_kinematics, manual_jacobin_matrix=None):
        """
        初始化IK求解器

        Arge:
            forward_kinematics: function 正向运动学方程
            manual_jacobin_matrix: function 手动计算雅可比矩阵
        """
        self.forward_kinematics = forward_kinematics
        self.manual_jacobin_matrix = manual_jacobin_matrix

    def _update_jacobian_matrix(self, current_theta, manual=False, delta=1e-6):
        """ 
        更新 J 矩阵: theta, self.forward_kinematics --> J

        Args:
            current_theta: np.array 当前各关节角度
            manual: bool 是否手动计算
            delta: float 微分扰动

        Return:
            J: np.array 雅可比矩阵
        """ 
        if manual:
            J = self.manual_jacobin_matrix(current_theta)
        else: 
            pos = self.forward_kinematics(current_theta)
            m = len(pos)
            n = len(current_theta)
            J = np.zeros((m, n))
            for i in range(len(current_theta)):
                d_theta = copy.deepcopy(current_theta)
                d_theta[i] += delta
                delta_pos = self.forward_kinematics(d_theta)
                J[:, i] = (delta_pos - pos) / delta
        return J

    def _update_jacobian_add_matrix(self, J, damping_factor):
        """ 
        更新 J_add 矩阵: J, self.damping_factor --> J_add

        Args:
            J: np.array 雅可比矩阵
            damping_factor: float 阻尼因子

        Return:
            J_add: np.array 雅可比加矩阵
        """ 
        J_square_matrix = J @ J.T
        mp_matrix = np.linalg.inv(
            J_square_matrix + damping_factor * np.eye(J_square_matrix.shape[0])
            )
        J_add = J.T @ mp_matrix
        return J_add

    def _update_delta_theta(self, target_pos, current_theta, J_add):
        """ 
        更新 delta_theta : target_pos, current_theta, J_add --> delta_theta

        Args:
            target_pos: list or np.array 目标位置
            current_theta: list or np.array 当前各关节角度
            J_add: np.array 雅可比加矩阵

        Return:
            delta_theta: np.array theta 更新量
        """ 
        delta_pos = target_pos - self.forward_kinematics(current_theta)
        delta_theta = J_add @ delta_pos
        return delta_theta

    def dls_fit(self, target_pos, current_theta, alpha=0.5, iter=100, damping_factor=0.5):
        """ 
        阻尼最小二乘法求运动学逆解

        Args:
            target_pos: list or np.array 目标位置
            current_theta: list or np.array 当前各关节角度
            alpha: float 学习率
            iter: int 迭代次数
            damping_factor: float 阻尼因子

        Return:
            target_theta: np.array 各关节角度
        """ 
        # 初始化 target_theta
        target_theta = copy.deepcopy(current_theta)
        for _ in range(iter):
            # 更新雅可比矩阵
            J = self._update_jacobian_matrix(target_theta)
            # 更新 J+ 矩阵
            J_add = self._update_jacobian_add_matrix(J, damping_factor)
            # 更新 delta_theta
            delta_theta = self._update_delta_theta(target_pos, target_theta, J_add)
            # 更新 target_theta
            target_theta = target_theta + alpha * delta_theta
        return target_theta

    def _update_hessian_matrix(self):
        pass


if __name__ == "__main__": 

    if test := 0:

        def forward_kinematics(theta):
            """ 
            定义正向运动学方程: theta --> pos
            """ 
            l = [3, 4, 5]
            for i in range(len(theta)):
                theta[i] = np.deg2rad(theta[i])
            x = l[0] * np.cos(theta[0]) + l[1] * np.cos(theta[0] + theta[1]) + l[2] * np.cos(theta[0] + theta[1] + theta[2])
            y = l[0] * np.sin(theta[0]) + l[1] * np.sin(theta[0] + theta[1]) + l[2] * np.sin(theta[0] + theta[1] + theta[2])
            pos = np.array([x, y])
            return pos

        def manual_jacobin_matrix(current_theta):
            """
            手动计算雅可比矩阵
            """
            l = [3, 4]
            J = np.array(
                [
                    [
                        -l[0] * np.sin(current_theta[0]) - l[1] * np.sin(current_theta[0] + current_theta[1]), 
                        -l[1] * np.sin(current_theta[0] + current_theta[1]),
                    ], 
                    [
                        l[0] * np.cos(current_theta[0]) + l[1] * np.cos(current_theta[0] + current_theta[1]), 
                        l[1] * np.cos(current_theta[0] + current_theta[1]),
                    ], 
                ]
            )
            return J

        iks = IKSolver(forward_kinematics, manual_jacobin_matrix)

        if test_J := 0:
            J1 = iks._update_jacobian_matrix(current_theta=[30, 20])
            J2 = iks._update_jacobian_matrix(current_theta=[30, 20], manual=True)
            print("数值计算的雅可比矩阵: \n", J1)
            print("手动计算的雅可比矩阵: \n", J2, "\n")

        if dls := 0:
            target_pos = [9, 2]
            current_theta = [50, 50, 50]
            target_theta = iks.dls_fit(target_pos, current_theta, alpha=0.5, iter=10, damping_factor=0.05)
            print(f"关节目标角度: {target_theta}")
            print(f"目标位置: {target_pos}")
            print(f"所求位置: {forward_kinematics(target_theta)}")
