/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#pragma once

#include <Eigen/Dense>

#include "common/smoothing/spline1d.h"
#include "gp_planner/gp/utils/gp_path.h"
#include "gp_planner/sdf/signed_distance_field_2d.h"
#include "gp_planner/st_plan/st_node.h"
#include "planning_core/planning_common/obstacle.h"

#include "gp_planner/st_plan/jps/st_jps.h"


namespace planning {

struct StPoint {
  double s_l;
  double s_u;
  double t;

  StPoint(const double s_l, const double s_u, const double t)
      : s_l(s_l), s_u(s_u), t(t) {}
};

class StGraph {
 public:
  StGraph() = default;
  // StGraph(const Eigen::Vector3d& init_s) : init_s_(init_s) {}

  explicit StGraph(const Eigen::Vector3d& init_s);

  void SetInitialState(const Eigen::Vector3d& init_s) { init_s_ = init_s; }
  void SetReferenceSpeed(const double ref_v) const;

  void BuildStGraph(const std::vector<Obstacle>& dynamic_obstacles,
                    const GPPath& gp_path);

  bool TopKSearch(const int k);

  bool SearchWithLocalTruncation(const int k, std::vector<StNode>* result);

  // 使用JPS进行搜索
  bool SearchWithJPS(const double ref_velocity, std::vector<StNode>* result);

  bool GenerateInitialSpeedProfile(const GPPath& gp_path);

  bool IsTrajectoryFeasible(const GPPath& gp_path, vector_Eigen3d* frenet_s);

  bool UpdateSpeedProfile(const GPPath& gp_path);

  bool OptimizeTest();

  void GenerateTrajectory(const ReferenceLine& reference_line,
                          const GPPath& gp_path,
                          common::Trajectory* trajectory);

  void VisualizeStGraph();

  void SaveSnapShot(const std::string& path);

  const OccupancyMap& grid_map() const { return sdf_->occupancy_map(); }

 private: 
  void GetObstacleBlockSegment(
      const Obstacle& obstacle, const GPPath& gp_path,
      std::vector<std::vector<StPoint>>* st_block_segment);

  void GetFrenetState(const double t, Eigen::Vector3d* s);

 private:
  Eigen::Vector3d init_s_;
  double stamp_now_ = 0.0;
  double ego_half_length_ = 0.0;
  double safety_margin_ = 0.0;
  double a_max_ = 2.0;
  double a_min_ = -4.0;
  const double lat_a_max_ = 4.0;

  double max_arc_length_ = 0.0;

  std::unique_ptr<SignedDistanceField2D> sdf_;
  std::vector<std::vector<std::vector<StPoint>>> st_block_segments_;
  std::vector<std::vector<std::unique_ptr<StNode>>> search_tree_;

  std::vector<StNode> st_nodes_;

  const double step_length_ = 0.2;
  std::vector<double> t_knots_;
  common::Spline1d st_spline_;

   // JPS规划器
  // std::unique_ptr<StJps> jps_planner_;

   // 改为shared_ptr，避免对齐内存的复杂处理
  std::shared_ptr<StJps> jps_planner_;

  // 添加对齐操作符
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // double refernce_speed_ = 0.0;  // 添加引用速度成员变量
  // double reference_speed_ = 8.0;  // 添加引用速度成员变量

    // 添加剪枝配置结构体
  struct PruningConfig {
    double max_acceleration = 2.0;     // 最大加速度
    double max_deceleration = -4.0;    // 最大减速度
    double max_velocity = 20.0;        // 最大速度
    double min_velocity = 0.0;         // 最小速度
    double collision_threshold = 1.0;   // 碰撞阈值
    double max_cost = 1e5;             // 最大代价阈值
    int collision_check_steps = 5;      // 碰撞检查步数
  };

  // 添加新的成员变量
  PruningConfig pruning_config_;
  
  // 添加新的私有方法
  bool ShouldPruneNode(const StNode* node, double next_acc) const;
  bool CheckCollision(const StNode* current_node, double acc, 
                     std::unique_ptr<StNode>& next_node) const;

   // 添加初始化函数声明
  void InitializePruningConfig();

};

}  // namespace planning
