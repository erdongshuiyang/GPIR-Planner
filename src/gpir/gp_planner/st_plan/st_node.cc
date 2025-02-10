/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#include "gp_planner/st_plan/st_node.h"

#include <cmath>
#include <limits>

namespace planning {

double StNode::ref_v_ = 0.0;
StNodeWeights StNode::weights_;

std::unique_ptr<StNode> StNode::Forward(const double delta_t,
                                        const double a) const {
  std::unique_ptr<StNode> st_node = std::make_unique<StNode>();
  st_node->a = a;
  st_node->t = t + delta_t;
  st_node->s = s + v * delta_t + 0.5 * a * delta_t * delta_t;
  st_node->v = v + a * delta_t;
  st_node->cost = cost;
  // st_node->cost += weights_.ref_v * std::fabs(v + a * delta_t / 2.0 - ref_v_);
  // st_node->cost += weights_.control * fabs(a) * delta_t;
  st_node->parent = this;

  // 计算代价
  st_node->CalTotalCost(0.0);  // 这里的障碍物距离将在调用时更新

  return st_node;
}

void StNode::CalTotalCost(const double obstacle_distance) {
  // 参考速度代价
  cost += weights_.ref_v * std::fabs(v - ref_v_);

  // 障碍物代价
  constexpr double kMinSafeDistance = 1.0;
  if (obstacle_distance <= kMinSafeDistance) {
    cost += 1e9;  // 碰撞惩罚
  } else if (obstacle_distance < 2.5) {
    cost += weights_.obstacle * (2.5 / obstacle_distance);
  }

  // 控制量代价
  cost += weights_.control * std::fabs(a);

  // 加加速度代价
  if (parent) {
    double jerk = GetJerk(t - parent->t);
    cost += weights_.jerk * jerk * jerk;
  }

  // 速度变化代价
  if (parent) {
    cost += weights_.v_change * std::fabs(v - parent->v);
  }
}

}  // namespace planning
