/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#pragma once

#include <memory>

#include "gp_planner/gp/utils/gp_path.h"
#include "gp_planner/sdf/signed_distance_field_2d.h" //表示路径规划中的距离场
#include "gtsam/nonlinear/ISAM2.h"   //增量优化库
#include "planning_core/navigation/reference_line.h"

namespace planning {

class GPIncrementalPathPlanner {
 public:
  //两种构造函数
  GPIncrementalPathPlanner() = default; //默认版
  GPIncrementalPathPlanner(std::shared_ptr<SignedDistanceField2D> sdf)
      : sdf_(sdf){};  //允许在构造时传入距离场 (SDF)，初始化 sdf_。SDF 用于路径生成中的碰撞检测和距离估计。

  //用于生成初始的高斯过程路径, obstacle_location_hint：障碍物位置提示，帮助路径生成时避开障碍物, gp_path：用于存储生成的路径。
  bool GenerateInitialGPPath(const ReferenceLine& reference_line,
                             const common::FrenetState& initial_state,
                             const double length,
                             const std::vector<double>& obstacle_location_hint,
                             GPPath* gp_path);

  bool TmpTest(const ReferenceLine& reference_line,
               const common::FrenetState& initial_state, const double length,
               const std::vector<double>& obstacle_location_hint,
               GPPath* gp_path);

  //UpdateGPPath()：用于对路径进行增量更新。
    //frenet_s：表示需要更新的位置，通常是受障碍物影响的路径部分。
    //gp_path：存储更新后的路径。
  bool UpdateGPPath(const ReferenceLine& reference_line,
                    const vector_Eigen3d& frenet_s, GPPath* gp_path);

  //UpdateGPPathNonIncremental(): 用于非增量更新的路径生成方法，重新计算整条路径，适用于当路径偏离较大时使用。
  bool UpdateGPPathNonIncremental(const ReferenceLine& reference_line,
                                  const vector_Eigen3d& frenet_s,
                                  GPPath* gp_path);

  //set_sdf()：设置 SDF，用于在路径生成过程中使用距离场信息。
  inline void set_sdf(std::shared_ptr<SignedDistanceField2D> sdf) {
    sdf_ = sdf;
  }
  //set_enable_curvature_constraint()：用于启用或禁用曲率约束，控制路径的平滑性和曲率限制。
  inline void set_enable_curvature_constraint(const bool option) {
    enable_curvature_constraint_ = option;
  }
  //set_enable_incremental_refinement()：用于启用或禁用增量优化，可以控制路径的更新方式。
  inline void set_enable_incremental_refinement(const bool option) {
    enable_incremental_refinemnt_ = option;
  }

 protected:
  //确定初始路径的边界。通过给定初始位置、角度、障碍物位置提示和边界信息，计算路径的上下边界 (lb 和 ub)，用于确保路径生成在可行的区域内。
  bool DecideInitialPathBoundary(
      const Eigen::Vector2d& init_pos, const double init_angle,
      const std::vector<double>& obstacle_location_hint,
      const std::vector<std::vector<std::pair<double, double>>>& boundaries,
      std::vector<double>* lb, std::vector<double>* ub);
  
  //FindLocationIndex()：查找指定弧长 s 在路径中的位置索引。通常用于更新路径节点或查找某个位置的路径状态。
  int FindLocationIndex(const double s);

 private:
  gtsam::ISAM2 isam2_; //ISAM2 是一种增量优化工具，用于高效更新路径以适应动态环境。它通过不断地向优化问题添加或更新因子，确保路径优化是实时的。
  gtsam::NonlinearFactorGraph graph_; //用于存储非线性因子图
  gtsam::Values map_result_; //存储优化后的路径节点状态

  int num_of_nodes_ = 21;
  double ego_width_ = 2.1;
  double ego_half_width_ = 1.0;
  const double kappa_limit_ = 0.2; //曲率限制
  double interval_ = 0.0; //路径点间隔
  std::vector<double> node_locations_; //节点位置，用于存储路径上节点的弧长信息。
  std::shared_ptr<SignedDistanceField2D> sdf_; //距离场，用于在路径生成过程中进行碰撞检测，确保路径的安全性。

  // options
  bool enable_curvature_constraint_ = true;
  bool enable_incremental_refinemnt_ = true;
};
}  // namespace planning
