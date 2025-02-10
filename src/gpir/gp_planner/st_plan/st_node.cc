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

// 添加静态成员初始化
double StNode::target_distance_ = 100.0;  // 默认值，可以后续更新

std::unique_ptr<StNode> StNode::Forward(const double delta_t,
                                        const double a) const {
  std::unique_ptr<StNode> st_node = std::make_unique<StNode>();
  st_node->a = a;
  st_node->t = t + delta_t;
  st_node->s = s + v * delta_t + 0.5 * a * delta_t * delta_t;
  st_node->v = v + a * delta_t;
  // st_node->cost = cost;
  // st_node->cost += weights_.ref_v * std::fabs(v + a * delta_t / 2.0 - ref_v_);
  // st_node->cost += weights_.control * fabs(a) * delta_t;
  st_node->parent = this;

  // 计算完整代价
  st_node->CalFullCost(weights_);

  return st_node;
}

// 添加新的代价计算函数
void StNode::CalFullCost(const StNodeWeights& weights) {
    // 继承父节点代价
    cost = parent ? parent->cost : 0.0;
    
    // 1. 基础代价(原Forward中的计算)
    double dt = parent ? (t - parent->t) : 0.0;
    cost += weights.ref_v * std::fabs(v - ref_v_);
    cost += weights.control * std::fabs(a) * dt;
    
    // 2. Jerk代价
    cost += weights.jerk * CalJerkCost();
    
    // 3. 效率代价
    cost += weights.efficiency * CalEfficiencyCost();
    
    // 4. 能耗代价
    cost += weights.energy * CalEnergyCost();
}

double StNode::CalJerkCost() const {
    if (!parent) return 0.0;
    double dt = t - parent->t;
    if (dt < 1e-6) return 0.0;
    
    double jerk = (a - parent->a) / dt;
    return std::fabs(jerk);
}

double StNode::CalEfficiencyCost() const {
    if (target_distance_ < 1e-6) return 0.0;
    
    // 计算归一化进度
    double progress = s / target_distance_;
    
    // 速度效率
    double velocity_efficiency = 
        (v > kMinVelocity) ? (1.0 - v / kMaxVelocity) : 1.0;
    
    return (1.0 - progress) + velocity_efficiency;
}

double StNode::CalEnergyCost() const {
    // 加速能耗
    double acc_energy = std::pow(a, 2);
    
    // 速度能耗
    double vel_energy = std::pow(v, 2) / std::pow(kMaxVelocity, 2);
    
    return acc_energy + vel_energy;
}

void StNode::CalObstacleCost(const double d) {
  constexpr double kMinSafeDistance = 1.0;
  if (d <= kMinSafeDistance) {
    cost += 1e9;
  } else if (d < 2.5) {
    cost += weights_.obstacle * 10 / d;
  }
}
}  // namespace planning
