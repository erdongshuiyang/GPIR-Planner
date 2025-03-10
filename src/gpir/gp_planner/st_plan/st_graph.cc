/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#include "gp_planner/st_plan/st_graph.h"

#include <glog/logging.h>
#include <omp.h>

#include <fstream>
#include <random>

#include "common/frenet/frenet_state.h"
#include "common/frenet/frenet_transform.h"
#include "common/smoothing/osqp_spline1d_solver.h"
#include "common/utils/timer.h"

namespace planning {

// 实现构造函数
StGraph::StGraph(const Eigen::Vector3d& init_s) : init_s_(init_s) {
    // 初始化现有成员
    stamp_now_ = 0.0;
    ego_half_length_ = 0.0;
    safety_margin_ = 0.0;
    
    // 初始化剪枝配置
    InitializePruningConfig();
}

// 实现初始化函数
void StGraph::InitializePruningConfig() {
    pruning_config_.max_acceleration = a_max_;
    pruning_config_.max_deceleration = a_min_;
    pruning_config_.max_velocity = std::max(20.0, 2.0 * StNode::reference_speed());
    pruning_config_.min_velocity = 0.0;
    pruning_config_.collision_threshold = safety_margin_ + 0.2;
    pruning_config_.max_cost = 1e5;
    pruning_config_.collision_check_steps = 5;
}
bool StGraph::ShouldPruneNode(const StNode* node, double next_acc) const {
  // 速度约束检查
  double next_vel = node->v + next_acc * 1.0;  // 1.0是时间步长
  if (next_vel > pruning_config_.max_velocity || 
      next_vel < pruning_config_.min_velocity) {
    return true;
  }

  // 加速度约束检查
  if (next_acc > pruning_config_.max_acceleration || 
      next_acc < pruning_config_.max_deceleration) {
    return true;
  }

  // 代价阈值检查
  if (node->cost > pruning_config_.max_cost) {
    return true;
  }

  return false;
}

bool StGraph::CheckCollision(const StNode* current_node, double acc,
                           std::unique_ptr<StNode>& next_node) const {
  for (int k = 1; k <= pruning_config_.collision_check_steps; ++k) {
    double check_time = current_node->t + k / 
                       static_cast<double>(pruning_config_.collision_check_steps);
    double check_dist = current_node->GetDistance(
        k / static_cast<double>(pruning_config_.collision_check_steps), acc);
    
    double dist = sdf_->SignedDistance(Eigen::Vector2d(check_time, check_dist));
    
    if (dist < pruning_config_.collision_threshold) {
      return true;  // 发现碰撞
    }
    
    next_node->CalObstacleCost(dist);
  }
  return false;  // 无碰撞
}

void StGraph::BuildStGraph(const std::vector<Obstacle>& dynamic_obstacles,
                           const GPPath& gp_path) {

  // 获取路径最大弧长
  max_arc_length_ = gp_path.MaximumArcLength();

  // 处理动态障碍物，并行计算每个障碍物的ST阻塞区域
  int size = dynamic_obstacles.size();
  st_block_segments_.resize(size);

  omp_set_num_threads(size);
  {
#pragma omp parallel for
    for (int i = 0; i < size; ++i) {
      GetObstacleBlockSegment(dynamic_obstacles[i], gp_path,
                              &st_block_segments_[i]);
    }
  }

  // 初始化栅格地图参数
  OccupancyMap grid_map;
  grid_map.set_origin({0.0, init_s_[0]});
  grid_map.set_resolution({0.1, 0.1});
  grid_map.set_cell_number({82, 1500});

  // 将ST阻塞区域转换为多边形
  vector_Eigen<Eigen::Vector2d> corners;
  corners.resize(4);

  // 遍历所有ST阻塞区域，填充到栅格地图中
  for (const auto& st_block_segment : st_block_segments_) {
    if (st_block_segment.empty()) continue;
    // 对每个阻塞区域构建四边形并填充
    for (const auto& st_points : st_block_segment) {
      // 构建四边形顶点
      if (st_points.empty()) continue;
      if (st_points.size() == 1) {
        corners[0] = Eigen::Vector2d(st_points[0].t, st_points[0].s_l);
        corners[1] = Eigen::Vector2d(st_points[0].t, st_points[0].s_u);
        corners[2] = Eigen::Vector2d(st_points[0].t + 0.1, st_points[0].s_u);
        corners[3] = Eigen::Vector2d(st_points[0].t + 0.1, st_points[0].s_l);
        grid_map.FillConvexPoly(corners);
        continue;
      }
      for (size_t i = 0; i < st_points.size() - 1; ++i) {
        corners[0] = Eigen::Vector2d(st_points[i].t, st_points[i].s_l);
        corners[1] = Eigen::Vector2d(st_points[i].t, st_points[i].s_u);
        corners[2] = Eigen::Vector2d(st_points[i + 1].t, st_points[i + 1].s_u);
        corners[3] = Eigen::Vector2d(st_points[i + 1].t, st_points[i + 1].s_l);
        grid_map.FillConvexPoly(corners);
      }
    }
  }
  // 基于占用栅格地图创建SDF
  sdf_ = std::make_unique<SignedDistanceField2D>(std::move(grid_map));
  // 更新垂直方向的SDF，用于快速碰撞检测
  sdf_->UpdateVerticalSDF();

  // cv::imshow("st", sdf_->occupancy_map().BinaryImage());
  // cv::imshow("st_sdf", sdf_->esdf().ImageSec());
  // cv::waitKey(0);
}

void StGraph::GetObstacleBlockSegment(
    const Obstacle& obstacle, const GPPath& gp_path,
    std::vector<std::vector<StPoint>>* st_block_segment) {
  const double length = obstacle.length();
  const double width = obstacle.width();
  double s_l = 0.0, s_u = 0.0;

  bool extend_from_previous = false;
  std::vector<StPoint>* current_seg = nullptr;
  // 遍历障碍物的预测轨迹点
  for (const auto& future_point : obstacle.prediction()) {
    // 检查是否与路径有重叠
    if (gp_path.HasOverlapWith(future_point, length, width, &s_l, &s_u)) {
      if (!extend_from_previous) {
        st_block_segment->emplace_back();
        current_seg = &st_block_segment->back();
        extend_from_previous = true;
      }
      // 记录重叠区域的上下边界
      current_seg->emplace_back(
          StPoint(s_l, s_u, future_point.stamp - stamp_now_));
    } else {
      extend_from_previous = false;
    }
  }
}

void StGraph::SetReferenceSpeed(const double ref_v) const {
  StNode::SetReferenceSpeed(ref_v);
}

bool StGraph::SearchWithJPS(double ref_velocity,std::vector<StNode>* result) {
   // 1. 初始状态检查
  LOG(INFO) << "SearchWithJPS start with initial state:"
            << " s=" << init_s_[0] 
            << " v=" << init_s_[1]
            << " a=" << init_s_[2];
  if (!jps_planner_) {
    // 使用make_shared简化创建过程
    jps_planner_ = std::make_shared<StJps>(sdf_.get());
  }

  // 2. 配置参数日志  
  LOG(INFO) << "JPS config settings:"
            << " max_velocity=" << ref_velocity  //reference_speed_
            << " max_acc=" << a_max_
            << " min_acc=" << a_min_
            << " step_length=" << step_length_;


  // 配置JPS搜索参数
  StJpsConfig config;
  config.max_acceleration = a_max_;
  config.min_acceleration = a_min_;
  config.safe_distance = safety_margin_; 
  config.max_velocity = ref_velocity; //refernce_speed_ reference_speed_
  config.step_length = step_length_;
  jps_planner_->SetConfig(config);

  // 设置目标点(使用ST图的最大弧长)
  std::vector<Eigen::Vector3d> goals;
  double t_horizon = 8.0;  // 时间范围
  double v_target = ref_velocity;  // 目标速度 refernce_speed_ reference_speed_
  
  // 添加多个目标状态以增加规划灵活性
  for (double t = 6.0; t <= t_horizon; t += 1.0) {
    double s = max_arc_length_;
    goals.push_back(Eigen::Vector3d(s, v_target, 0.0));
    LOG(INFO) << "Add goal state: s=" << s << " v=" << v_target 
              << " t=" << t;
  }
  jps_planner_->SetGoalStates(goals);

  LOG(INFO) << "Set " << goals.size() << " goal states with max_arc_length=" 
            << max_arc_length_;

  // 执行JPS搜索
  std::vector<StJpsNode> jps_path;
  if (!jps_planner_->Search(init_s_, &jps_path)) {
    LOG(ERROR) << "JPS search failed";
    return false;
  }

  // 转换结果到StNode
  result->clear();
  for (const auto& node : jps_path) {
    result->emplace_back(node.s(), node.v(), node.a());
    // 设置时间戳
    result->back().t = node.t();
  }

  // 记录搜索结果到st_nodes_用于后续优化
  st_nodes_ = *result;

  return true;
}

bool StGraph::SearchWithLocalTruncation(const int k,
                                        std::vector<StNode>* result) {
  // 在搜索开始前设置目标距离
  SetTargetDistance(max_arc_length_);

  CHECK(k % 2 == 1) << "k needs to be an odd number, while k is " << k;

  // 1. 初始化加速度离散值
  int num_a_per_side = static_cast<int>(k / 2.0);
  std::vector<double> discrete_a;
  discrete_a.reserve(k);
  discrete_a.emplace_back(0.0);
  for (int i = 0; i < num_a_per_side; ++i) {
    discrete_a.emplace_back(a_max_ * (i + 1) / num_a_per_side);
    discrete_a.emplace_back(a_min_ * (i + 1) / num_a_per_side);
  }

  // 2. 设置搜索代价权重
  StNodeWeights weight;
  // weight.control = 0.5;
  weight.obstacle = 10;
  weight.ref_v = 3;
  StNode::SetWeights(weight);

  // 3. 创建初始节点并初始化搜索树
  // 搜索树是一个二维数组,每一层存储该层的所有节点
  std::unique_ptr<StNode> inital_node =
      std::make_unique<StNode>(init_s_[0], init_s_[1], init_s_[2]);
  search_tree_.resize(9); // 0-8共9层
  // 放入初始节点到第0层
  search_tree_[0].emplace_back(std::move(inital_node));

  // LOG(INFO) << "[search tree] init velocity: " << init_s_[1];
  // LOG(INFO) << "[search tree] reference velocity: "
  // << StNode::reference_speed();

  TIC;
  double t_expand = 0.0;
  double t_sort = 0.0;
  double t_compare = 0.0;
  double kEpsilon = 0.1;

  // 4. 主搜索循环
  for (int i = 0; i < 8; ++i) { // 遍历每一层(0-7层)
    std::vector<std::unique_ptr<StNode>> cache;  // 临时存储扩展出的节点

    // 对当前层的每个节点进行扩展
    for (int j = 0; j < search_tree_[i].size(); ++j) 
    {
      const auto& current_node = search_tree_[i][j];
      // 使用不同加速度值进行扩展
      for (const auto& a : discrete_a) {

         // 使用新的剪枝检查
        if (ShouldPruneNode(current_node.get(), a)) {
          continue;
        } 
        
        // 向前扩展1s,使用加速度a
        auto next_node = current_node->Forward(1.0, a);

         // 使用新的碰撞检查
        if (!CheckCollision(current_node.get(), a, next_node) && 
            next_node->cost < pruning_config_.max_cost) {
          // 如果合理则保留该节点
          cache.emplace_back(std::move(next_node));
        }
      }
    }
    // 按纵向位置s排序所有扩展节点
    std::sort(cache.begin(), cache.end(),
              [](const std::unique_ptr<StNode>& n1,
                 const std::unique_ptr<StNode>& n2) { return n1->s < n2->s; });

    if (cache.empty()) {
      LOG(ERROR) << "cannot find valid path";
      result = nullptr;
      return false;
    }

    // 对相近位置节点进行分组
    int min_index = 0;
    int min_cost = cache[0]->cost;
    double start_s = cache[0]->s;
    bool is_new_group = false;

    for (int j = 0; j < cache.size(); ++j) {
      // 如果位置差小于阈值,归入当前组
      if (cache[j]->s - start_s <= kEpsilon) {
        // 更新组内最优节点
        if (cache[j]->cost < min_cost) {
          min_cost = cache[j]->cost;
          min_index = j;
        }
      } else {
        // 开始新的一组
        is_new_group = true;
      }

      // 当组结束时,保留该组最优节点到下一层
      if (is_new_group || j == cache.size() - 1) {
         // 重置组相关变量
        start_s = cache[j]->s;
        search_tree_[i + 1].emplace_back(std::move(cache[min_index]));
        min_index = j + 1;
        is_new_group = false;
      }
    }

    // 在搜索树的较低层（早期时间），车辆的位置差异较小。随着时间推进（搜索树的高层），由于速度和加速度的累积效应，相邻轨迹之间的位置差异会逐渐增大
    kEpsilon *= 1.3; // 将截断阈值扩大1.3倍，随层数增加 kEpsilon 以适应速度差异，
                      
  }
  //首先对最后一层的节点按代价排序
  std::sort(
      search_tree_.back().begin(), search_tree_.back().end(),
      [](const std::unique_ptr<StNode>& n1, const std::unique_ptr<StNode>& n2) {
        return n1->cost < n2->cost;
      });

  // extract answer
  //从代价最小的终点节点开始回溯
  const StNode* current_node = search_tree_.back().front().get();
  st_nodes_.emplace_back(*current_node);
  // 通过parent指针回溯到起点
  while (current_node->parent != nullptr) {
    current_node = current_node->parent;
    st_nodes_.emplace_back(*current_node);
  }
  //将路径反转得到从起点到终点的顺序
  std::reverse(st_nodes_.begin(), st_nodes_.end());

  return true;
}

bool StGraph::GenerateInitialSpeedProfile(const GPPath& gp_path) {
  std::vector<double> lbs;
  std::vector<double> ubs;
  std::vector<double> refs;
  double lb, ub;
  const auto& grid_map = sdf_->occupancy_map();
  // 为每个搜索树节点获取ST图中的纵向边界
  for (const auto& node : st_nodes_) {
    t_knots_.emplace_back(node.t); // 记录时间节点
    // 在该时间点找到安全的位置上下边界
    grid_map.FindVerticalBoundary(node.t, node.s, &lb, &ub);
    lbs.emplace_back(lb); // 位置下界
    ubs.emplace_back(ub); // 位置上界
    refs.emplace_back(node.s); // 参考位置
  }

  // 采样约束条件
  std::vector<double> t_samples, v_min, v_max, a_max, a_min;
  for (double t = t_knots_.front() + step_length_; t < t_knots_.back();
       t += step_length_) {
    // 计算采样点的状态
    int index = static_cast<int>(t);
    double delta = t - st_nodes_[index].t;
    // 使用运动学方程计算位置
    double s = st_nodes_[index].s + st_nodes_[index].v * delta +
               0.5 * st_nodes_[index + 1].a * delta * delta;
    // 根据曲率计算速度限制
    // 这里使用离心加速度公式: a_lat = v²/r = v²*κ ≤ lat_a_max_
    double max_abs_v =
        1.2 * std::sqrt(lat_a_max_ / std::fabs(gp_path.GetCurvature(s)));

    // 记录约束条件
    t_samples.emplace_back(t);
    v_min.emplace_back(-max_abs_v);
    v_max.emplace_back(max_abs_v);
    a_min.emplace_back(a_min_);
    a_max.emplace_back(a_max_);
  }

  // 第三部分：配置优化求解器
  // 创建5阶样条优化求解器
  common::OsqpSpline1dSolver solver(t_knots_, 5);
  auto kernel = solver.mutable_kernel();

  // 添加优化目标
  kernel->AddRegularization(1e-5); // 正则化项，防止病态解
  kernel->AddSecondOrderDerivativeMatrix(5); // 最小化加速度变化
  kernel->AddThirdOrderDerivativeMatrix(20); // 最小化加加速度
  kernel->AddReferenceLineKernelMatrix(t_knots_, refs, 20); // 跟踪参考路径

  // 添加约束条件
  auto constraint = solver.mutable_constraint();
  constraint->AddThirdDerivativeSmoothConstraint(); // 平滑性约束
  constraint->AddPointConstraint(t_knots_.front(), init_s_[0]); // 初始位置
  constraint->AddPointDerivativeConstraint(t_knots_.front(), init_s_[1]); // 初始速度
  // constraint->AddPointSecondDerivativeConstraint(t_knots_.front(),
  // init_s_[2]);
  constraint->AddBoundary(t_knots_, lbs, ubs);
  // constraint->AddDerivativeBoundary(t_samples, v_min, v_max);
  // constraint->AddSecondDerivativeBoundary(t_samples, a_min, a_max);

  if (!solver.Solve()) {
    LOG(ERROR) << "fail to optimize";
    return false;
  }

  st_spline_ = solver.spline();

  return true;
}

bool StGraph::UpdateSpeedProfile(const GPPath& gp_path) {
  TIC;
  std::vector<double> lbs;
  std::vector<double> ubs;
  std::vector<double> refs;
  double lb, ub;
  const auto& grid_map = sdf_->occupancy_map();
  for (const auto& node : st_nodes_) {
    grid_map.FindVerticalBoundary(node.t, node.s, &lb, &ub);
    lbs.emplace_back(lb);
    ubs.emplace_back(ub);
  }
  for (const auto& t : t_knots_) {
    refs.emplace_back(st_spline_(t));
  }

  std::vector<double> t_samples, v_min, v_max, a_max, a_min;
  for (double t = t_knots_.front() + 0.1; t < t_knots_.back(); t += 0.1) {
    double s = st_spline_(t);
    double max_abs_v =
        std::sqrt(lat_a_max_ / std::fabs(gp_path.GetCurvature(s))) + 1;
    t_samples.emplace_back(t);
    v_min.emplace_back(-max_abs_v);
    v_max.emplace_back(max_abs_v);
  }

  common::OsqpSpline1dSolver solver(t_knots_, 5);
  auto kernel = solver.mutable_kernel();
  kernel->AddRegularization(1e-5);
  // kernel->AddSecondOrderDerivativeMatrix(5);
  kernel->AddThirdOrderDerivativeMatrix(30);
  kernel->AddReferenceLineKernelMatrix(t_knots_, refs, 10);
  auto constraint = solver.mutable_constraint();
  constraint->AddThirdDerivativeSmoothConstraint();
  constraint->AddPointConstraint(t_knots_.front(), init_s_[0]);
  constraint->AddPointDerivativeConstraint(t_knots_.front(), init_s_[1]);
  constraint->AddPointSecondDerivativeConstraint(t_knots_.front(), init_s_[2]);
  constraint->AddBoundary(t_knots_, lbs, ubs);
  // constraint->AddDerivativeBoundary(t_samples, v_min, v_max);
  // constraint->AddSecondDerivativeBoundary(t_samples, a_min, a_max);

  if (!solver.Solve()) {
    LOG(ERROR) << "fail to optimize";
    return false;
  }

  st_spline_ = solver.spline();

  return true;
}

void StGraph::GetFrenetState(const double t, Eigen::Vector3d* s) {
  s->x() = st_spline_(t);
  s->y() = st_spline_.Derivative(t);
  s->z() = st_spline_.SecondOrderDerivative(t);
}

bool StGraph::IsTrajectoryFeasible(const GPPath& gp_path,
                                   vector_Eigen3d* frenet_s) {
  frenet_s->clear();
  double lat_acc = 0.0;
  Eigen::Vector3d s, d;
  for (double t = t_knots_.front() + step_length_; t < t_knots_.back();
       t += step_length_) {
    GetFrenetState(t, &s);
    if (s(0) > max_arc_length_) break;
    gp_path.GetInterpolateNode(s(0), &d);
    lat_acc = d(2) * s(1) * s(1) + d(1) * s(2);
    if (std::fabs(lat_acc) > 1.05 * lat_a_max_) {
      // printf("invalid at %f, acc: %f\n", t, lat_acc);
      frenet_s->emplace_back(s);
    }
  }
  return frenet_s->empty() ? true : false;
}

void StGraph::GenerateTrajectory(const ReferenceLine& reference_line,
                                 const GPPath& gp_path,
                                 common::Trajectory* trajectory) {
  trajectory->clear();
  const double t_final = st_nodes_.back().t;
  const double maximum_s = gp_path.MaximumArcLength();
  Eigen::Vector3d d;
  common::State state;
  for (double t = 0.0; t <= t_final; t += 0.1) {
    Eigen::Vector3d s(st_spline_(t), st_spline_.Derivative(t),
                      st_spline_.SecondOrderDerivative(t));
    if (s[0] > maximum_s) break;
    gp_path.GetInterpolateNode(s[0], &d);
    auto ref = reference_line.GetFrenetReferncePoint(s[0]);
    auto ref_point = reference_line.GetFrenetReferncePoint(s[0]);
    common::FrenetTransfrom::FrenetStateToState(common::FrenetState(s, d),
                                                ref_point, &state);
    state.stamp = t;
    state.frenet_d = d;
    state.velocity = st_spline_.Derivative(t);

    const double one_minus_kappa_rd = 1 - ref.kappa * d[0];

    const double tan_delta_theta = d[1] / one_minus_kappa_rd;
    const double delta_theta = std::atan2(d[1], one_minus_kappa_rd);
    const double cos_delta_theta = std::cos(delta_theta);
    state.frenet_s = s;
    trajectory->emplace_back(state);
  }
}
}  // namespace planning
