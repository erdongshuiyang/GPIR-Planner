/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#pragma once

#include <memory>
#include <vector>

#include "gp_planner/gp/utils/gp_path.h"
#include "gp_planner/sdf/signed_distance_field_2d.h"
#include "gtsam/nonlinear/ISAM2.h"
#include "planning_core/navigation/reference_line.h"
#include "planning_core/planning_common/perception_uncertainty.h"
#include "planning_core/planning_common/obstacle.h"  // 添加这行

namespace planning {

class GPIncrementalPathPlanner {
 public:
  GPIncrementalPathPlanner() = default;
  GPIncrementalPathPlanner(std::shared_ptr<SignedDistanceField2D> sdf)
      : sdf_(sdf){};

   // 添加不确定性相关的设置方法
  void SetDefaultUncertainty(const PerceptionUncertainty& uncertainty) {
    default_uncertainty_ = uncertainty;
  }

  // 设置参考线的方法放在公共方法部分
  void SetReferenceLine(const ReferenceLine* reference_line) {
    reference_line_ = reference_line;
  }

  bool GenerateInitialGPPath(const ReferenceLine& reference_line,
                             const common::FrenetState& initial_state,
                             const double length,
                             const std::vector<double>& obstacle_location_hint,
                             const std::vector<Obstacle>& obstacles,  // 添加障碍物参数
                             GPPath* gp_path);

  bool TmpTest(const ReferenceLine& reference_line,
               const common::FrenetState& initial_state, const double length,
               const std::vector<double>& obstacle_location_hint,
               GPPath* gp_path);

  bool UpdateGPPath(const ReferenceLine& reference_line,
                    const vector_Eigen3d& frenet_s, 
                    const std::vector<Obstacle>& obstacles,
                    GPPath* gp_path);

  bool UpdateGPPathNonIncremental(const ReferenceLine& reference_line,
                                  const vector_Eigen3d& frenet_s,
                                  const std::vector<Obstacle>& obstacles,
                                  GPPath* gp_path);

  inline void set_sdf(std::shared_ptr<SignedDistanceField2D> sdf) {
    sdf_ = sdf;
  }
  inline void set_enable_curvature_constraint(const bool option) {
    enable_curvature_constraint_ = option;
  }
  inline void set_enable_incremental_refinement(const bool option) {
    enable_incremental_refinemnt_ = option;
  }

 protected:
  bool DecideInitialPathBoundary(
      const Eigen::Vector2d& init_pos, const double init_angle,
      const std::vector<double>& obstacle_location_hint,
      const std::vector<std::vector<std::pair<double, double>>>& boundaries,
      std::vector<double>* lb, std::vector<double>* ub);

  int FindLocationIndex(const double s);

 private:
  gtsam::ISAM2 isam2_;
  gtsam::NonlinearFactorGraph graph_;
  gtsam::Values map_result_;

  int num_of_nodes_ = 21;
  double ego_width_ = 2.1;
  double ego_half_width_ = 1.0;
  const double kappa_limit_ = 0.2;
  double interval_ = 0.0;
  std::vector<double> node_locations_;
  std::shared_ptr<SignedDistanceField2D> sdf_;

  // 用于处理不确定性的辅助方法
  void AddUncertaintyFactors(const std::vector<Obstacle>& obstacles,
                            double current_s,
                            gtsam::Key key);

  // 增加安全性检查方法
  bool ValidatePathSafety(const GPPath& path,
                         const std::vector<Obstacle>& obstacles) const;

  // 添加默认的不确定性参数
  PerceptionUncertainty default_uncertainty_;

  // 安全性相关参数
  static constexpr double kSafetyThreshold = 0.5;
  static constexpr double kMaxCollisionProb = 0.05;

  // options
  bool enable_curvature_constraint_ = true;
  bool enable_incremental_refinemnt_ = true;

  const ReferenceLine* reference_line_{nullptr}; 
  double kappa_r_{0.0}; 

};
}  // namespace planning
