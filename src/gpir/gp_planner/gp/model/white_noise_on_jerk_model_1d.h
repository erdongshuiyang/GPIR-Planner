/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#pragma once

#include <Eigen/Core>
#include <array>

namespace planning {

// 该类主要提供了高斯过程轨迹规划所需的几种矩阵：
// WhiteNoiseOnJerkModel1D 类用于描述“加速度的导数（即 jerk，冲击）”的白噪声模型。
// 该模型是一种用于描述路径状态在位置、速度、加速度之间动态关系的模型，它假设车辆的加速度是随时间变化的，并且其变化（jerk）受到过程噪声的影响。

class WhiteNoiseOnJerkModel1D {
 public:

  //Q 矩阵：过程噪声协方差矩阵，用于描述加速度导数（jerk）影响下路径的不确定性。
  //见论文公式（35）
  static inline Eigen::Matrix3d Q(const double qc, const double tau) {
    Eigen::Matrix3d q;
    std::array<double, 5> tau_powers;
    constexpr double one_third = 1.0 / 3.0;
    constexpr double one_sixth = 1.0 / 6.0;

    tau_powers[0] = tau * qc;
    for (int i = 1; i < 5; ++i) tau_powers[i] = tau * tau_powers[i - 1];

    // q(0,0) = 0.5 in the origin GPMP paper, which is a mistake
    q(0, 0) = 0.05 * tau_powers[4];
    q(0, 1) = 0.125 * tau_powers[3];
    q(0, 2) = one_sixth * tau_powers[2];
    q(1, 0) = q(0, 1);
    q(1, 1) = one_third * tau_powers[2];
    q(1, 2) = 0.5 * tau_powers[1];
    q(2, 0) = q(0, 2);
    q(2, 1) = q(1, 2);
    q(2, 2) = tau_powers[0];
    return q;
  }

  // Phi 矩阵：状态转移矩阵，描述了路径点之间的动态特性。
  // 论文公式（34）
  static inline Eigen::Matrix3d Phi(const double tau) {
    Eigen::Matrix3d phi = Eigen::Matrix3d::Identity();
    phi(0, 1) = tau;
    phi(0, 2) = 0.5 * tau * tau;
    phi(1, 2) = tau;
    return phi;
  }


  //LambdaAndPsi 函数：用于描述状态变化的中间量，结合了不同时间点的状态信息
  //这些矩阵用于描述节点之间的插值。对于高斯过程插值，Lambda 和 Psi 确保在相邻节点之间插入的点也符合相同的平滑性和障碍约束。
  static inline void LambdaAndPsi(const double qc, const double delta,
                                  const double tau, Eigen::Matrix3d* lambda,
                                  Eigen::Matrix3d* psi) {
    *psi = Q(qc, tau) * (Phi(delta - tau).transpose()) * QInverse(qc, delta);
    *lambda = Phi(tau) - (*psi) * Phi(delta);
  }

  //QInverse 矩阵：过程噪声协方差矩阵的逆矩阵。
  static inline Eigen::Matrix3d QInverse(const double qc, const double tau) {
    Eigen::Matrix3d q_inv;
    std::array<double, 5> tau_powers;
    const double tau_inv = 1.0 / tau;

    tau_powers[0] = tau_inv / qc;
    for (int i = 1; i < 5; ++i) tau_powers[i] = tau_inv * tau_powers[i - 1];

    q_inv(0, 0) = 720 * tau_powers[4];
    q_inv(0, 1) = -360 * tau_powers[3];
    q_inv(0, 2) = 60 * tau_powers[2];
    q_inv(1, 0) = q_inv(0, 1);
    q_inv(1, 1) = 192 * tau_powers[2];
    q_inv(1, 2) = -36 * tau_powers[1];
    q_inv(2, 0) = q_inv(0, 2);
    q_inv(2, 1) = q_inv(1, 2);
    q_inv(2, 2) = 9 * tau_powers[0];
    return q_inv;
  }
};
}  // namespace planning
