/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#pragma once

#include "gp_planner/gp/model/white_noise_on_jerk_model_1d.h"
#include "gtsam/nonlinear/NonlinearFactor.h"

namespace planning {
//GPPriorFactor是从 gtsam::NoiseModelFactor2<gtsam::Vector3, gtsam::Vector3> 派生的类，属于 GTSAM 库的一部分。
//二元因子（NoiseModelFactor2），意味着它连接两个节点，作用于它们之间的约束。这两个节点分别表示不同时间步长的状态，例如位置、速度等信息。
//gtsam::Vector3 表示因子的变量是 3 维向量，这可以用来描述路径节点的状态，比如横向位置、横向速度和加速度。
class GPPriorFactor
    : public gtsam::NoiseModelFactor2<gtsam::Vector3, gtsam::Vector3> {
 public:
  //gtsam::Key key1, key2: 分别表示因子图中的两个节点标识符。
  //delta: 这是离散时间步长，即两个节点之间的时间间隔。
  //Qc: 过程噪声的协方差，用于建模系统的动态不确定性。
  GPPriorFactor(gtsam::Key key1, gtsam::Key key2, const double delta, const double Qc) :
         NoiseModelFactor2(gtsam::noiseModel::Gaussian::Covariance(WhiteNoiseOnJerkModel1D::Q(Qc, delta)),key1, key2),
         delta_(delta),
         phi_(WhiteNoiseOnJerkModel1D::Phi(delta)){};
  //语法层面：该构造函数利用成员初始化列表
  // 在构造 GPPriorFactor 对象的同时调用其父类 NoiseModelFactor2 的构造函数。
  // 噪声是高斯噪声 (Gaussian)，并通过协方差矩阵来定义
  // WhiteNoiseOnJerkModel1D::Q(Qc, delta) 返回过程噪声的协方差矩阵
  // delta_ 是一个表示两个节点之间距离或时间间隔的变量
  // phi_ 是状态转移矩阵，WhiteNoiseOnJerkModel1D::Phi(delta) 计算了状态转移矩阵 phi_


  ~GPPriorFactor() = default;

  gtsam::Vector evaluateError(
      const gtsam::Vector3& x1, const gtsam::Vector3& x2,
      boost::optional<gtsam::Matrix&> H1 = boost::none,
      boost::optional<gtsam::Matrix&> H2 = boost::none) const override;

 private:
  double delta_ = 0.0;
  gtsam::Matrix33 phi_;
};
}  // namespace planning