#pragma once

#include <memory>
#include <queue>
#include <unordered_set>
#include <vector>

#include "common/frenet/frenet_state.h" 
#include "gp_planner/sdf/signed_distance_field_2d.h"
#include "gp_planner/st_plan/jps/st_jps_node.h"

namespace planning {

// JPS搜索参数
struct StJpsConfig {
  // 约束条件
  double max_acceleration = 2.0;    // 最大加速度
  double min_acceleration = -4.0;   // 最小加速度(减速度) 
  double max_velocity = 20.0;       // 最大速度
  double min_velocity = 0.0;        // 最小速度
  double max_curvature = 0.2;       // 最大曲率

  // 代价权重
  double w_obstacle = 5.0;    // 障碍物代价权重
  double w_reference = 1.0;    // 参考速度代价权重
  double w_smooth = 0.5;       // 平滑代价权重
  double w_s = 3.0; //纵向权重

  // 搜索参数 
  double step_length = 0.8;    // 搜索步长
  double grid_resolution = 0.4; // 栅格分辨率
  int num_sample_a = 7;        // 加速度采样数
  double safe_distance = 2.0;  // 安全距离
  double time_resolution = 0.1; // 时间分辨率 
  int max_iter = 10000;          // 最大迭代次数
};

class StJps {
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 
   explicit StJps(const SignedDistanceField2D* sdf) : sdf_(sdf) {}
  
  // 设置配置参数
  void SetConfig(const StJpsConfig& config) { config_ = config; }
  const StJpsConfig& config() const { return config_; }
  
  // 主搜索函数
  bool Search(const Eigen::Vector3d& init_state,
             std::vector<StJpsNode>* result);
             
  // 设置目标状态
  void SetGoalStates(const std::vector<Eigen::Vector3d>& goal_states);


private:
  // 初始化搜索 
  bool InitializeSearch();
  
  // 获取邻居节点
  std::vector<StJpsNodePtr> GetNeighbors(const StJpsNodePtr& node);
  
  // 计算启发代价
  double CalcHeuristic(const StJpsNodePtr& node,
                      const std::vector<StJpsNodePtr>& goals);
                      
  // 计算转移代价 
  double CalcTransitionCost(const StJpsNodePtr& from,
                           const StJpsNodePtr& to);
                           
  // 判断是否为跳点
  bool IsJumpPoint(const StJpsNodePtr& node,
                  const std::vector<StJpsNodePtr>& neighbors);
                  
  // 判断是否为目标点
  bool IsGoalNode(const StJpsNodePtr& node);
  
  // 构建结果路径
  void BuildResult(const StJpsNodePtr& final_node,
                  std::vector<StJpsNode>* result);
  
  // 重新连接路径
  void ReconnectPath(std::vector<StJpsNodePtr>* path);
  
  // 计算两个节点间的插值节点
  std::vector<StJpsNodePtr> GetInterpolatedNodes(
      const StJpsNodePtr& from, const StJpsNodePtr& to);
  
  // 平滑轨迹
  void SmoothTrajectory(std::vector<StJpsNodePtr>* path);
  
  // 检查节点可行性
  bool ValidateNode(const StJpsNodePtr& node);
  
  // 检查轨迹可行性
  bool CheckTrajectoryFeasible(const StJpsNodePtr& from,
                              const StJpsNodePtr& to);

private:
  // 配置参数
  StJpsConfig config_;
  
  // SDF地图
  const SignedDistanceField2D* sdf_{nullptr};

  // 搜索相关
  Eigen::Vector3d init_state_;  // 初始状态[s,s_dot,s_ddot]
  std::vector<StJpsNodePtr> goals_; // 目标点
  
  std::priority_queue<StJpsNodePtr, std::vector<StJpsNodePtr>, 
                     StJpsNodeComparator> open_set_;
  std::unordered_set<StJpsNodePtr> closed_set_;

//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace planning