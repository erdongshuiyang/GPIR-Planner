/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#pragma once

#include <memory>

namespace planning {

struct StNodeWeights {
  double ref_v = 3.0;
  double obstacle = 10.0;
  double control = 0.5;
  double jerk = 2.0;       // 加加速度权重
  double efficiency = 1.0;  // 效率权重
  double energy = 0.8;     // 能耗权重  
};

class StNode {
 public:
  StNode() = default;
  StNode(const double s, const double v, const double a = 0.0)
      : s(s), v(v), a(a) {}

  std::unique_ptr<StNode> Forward(const double delta_t, const double a) const;

  // 添加新的代价计算相关函数
  void CalFullCost(const StNodeWeights& weights);
  double CalJerkCost() const;
  double CalEfficiencyCost() const;
  double CalEnergyCost() const;

  void CalObstacleCost(const double d);
  inline double GetDistance(const double delta_t, const double a) const {
    return s + v * delta_t + 0.5 * a * delta_t * delta_t;
  }

  static void SetReferenceSpeed(const double ref_v) { ref_v_ = ref_v; }
  static void SetWeights(const StNodeWeights& weights) { weights_ = weights; }
  static double reference_speed() { return ref_v_; }

  double t = 0.0;
  double s = 0.0;
  double v = 0.0;
  double a = 0.0;
  double cost = 0.0;
  const StNode* parent = nullptr;

  // 添加静态方法来设置和获取target_distance
  static void SetTargetDistance(const double distance) { 
      target_distance_ = distance; 
  }
  
  static double GetTargetDistance() { 
      return target_distance_; 
  }

 private:
  static double ref_v_;
  static StNodeWeights weights_;

  // 添加新的成员变量
  static double target_distance_;
  static constexpr double kMaxVelocity = 20.0;
  static constexpr double kMinVelocity = 0.1;
};

}  // namespace planning
