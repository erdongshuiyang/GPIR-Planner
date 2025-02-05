#include "gp_planner/st_plan/jps/st_jps.h"

#include <glog/logging.h>
#include <algorithm>
#include <cmath>

namespace planning {

// StJps::StJps(const SignedDistanceField2D* sdf) : sdf_(sdf) {}

void StJps::SetGoalStates(const std::vector<Eigen::Vector3d>& goal_states) {
  goals_.clear();
  for (const auto& state : goal_states) {
    auto goal = std::make_shared<StJpsNode>(state[0], state[1], state[2], 0.0);
    goals_.push_back(goal);
  }
}

bool StJps::Search(const Eigen::Vector3d& init_state,
                   std::vector<StJpsNode>* result) {
  if (!InitializeSearch()) {
    LOG(ERROR) << "Failed to initialize JPS search";
    return false;
  }

  init_state_ = init_state;
  
  // 创建起点
  auto start = std::make_shared<StJpsNode>(0.0, init_state_[0], 
                                          init_state_[1], init_state_[2]);
  LOG(INFO) << "Search starting from node: " << start->DebugString();              
  start->set_g_cost(0.0); 
  start->set_h_cost(CalcHeuristic(start, goals_));
  start->set_status(JpsNodeStatus::kOpen);
  LOG(INFO) << "Start node costs: g=" << start->g_cost() 
            << " h=" << start->h_cost() 
            << " f=" << start->f_cost();
  
  open_set_.push(start);

  int iter = 0;
  while (!open_set_.empty() && iter++ < config_.max_iter) {
    // if(iter % 100 == 0) { // 每100次迭代输出一次
    //   LOG(INFO) << "Iter " << iter << ": open_set size=" << open_set_.size()
    //             << " closed_set size=" << closed_set_.size();
    // }

    auto current = open_set_.top();
    open_set_.pop();

    // 检查是否到达目标
    if (IsGoalNode(current)) {
      LOG(INFO) << "Goal reached at node: " << current->DebugString();  
      BuildResult(current, result);
      return true;
    }

    current->set_status(JpsNodeStatus::kClosed);
    closed_set_.insert(current);

    // 获取邻居节点
    auto neighbors = GetNeighbors(current);
    // LOG(INFO) << "Node " << current->DebugString() 
    //           << " has " << neighbors.size() << " neighbors";

    // 判断是否为跳点
    if (!IsJumpPoint(current, neighbors)) {
      continue;
    }

    // 遍历邻居节点
    int added_to_open = 0;  // 统计加入open_set的数量

    // 在遍历前记录当前open_set大小
    int orig_open_size = open_set_.size();
    // LOG(INFO) << "Before expansion: open_set size = " << orig_open_size;

    // 遍历邻居节点
    for (const auto& next : neighbors) {
      if (next->status() == JpsNodeStatus::kClosed) {
        //  LOG(INFO) << "Skip closed node: " << next->DebugString();
        continue;
      }

      double new_g_cost = current->g_cost() + 
                         CalcTransitionCost(current, next);

      if (next->status() != JpsNodeStatus::kOpen || 
          new_g_cost < next->g_cost()) {
        next->set_g_cost(new_g_cost);
        next->set_h_cost(CalcHeuristic(next, goals_));

        // LOG(INFO) << "Add to open_set: " << next->DebugString()
        //     << " g=" << next->g_cost()
        //     << " h=" << next->h_cost() 
        //     << " f=" << next->f_cost()
        //     << " status=" << static_cast<int>(next->status());

        next->set_parent(current.get());
        next->set_status(JpsNodeStatus::kOpen);
        open_set_.push(next);
        added_to_open++;
      }else {
        // LOG(INFO) << "Skip node(cost not better): " << next->DebugString()
        //         << " old_g=" << next->g_cost()
        //         << " new_g=" << new_g_cost;
      }
    }
    // 在遍历后记录最终状态
    // LOG(INFO) << "After expansion: " 
    //           << added_to_open << "/" << neighbors.size() << " neighbors added to open_set"
    //           << ", open_set size = " << open_set_.size()
    //           << ", closed_set size = " << closed_set_.size();
  }

  LOG(WARNING) << "JPS search failed:"
               << " iter=" << iter
               << " open_set_size=" << open_set_.size()
               << " closed_set_size=" << closed_set_.size();
  return false;
}

bool StJps::InitializeSearch() {
  while (!open_set_.empty()) {
    open_set_.pop();
  }
  closed_set_.clear();

  if (goals_.empty()) {
    LOG(ERROR) << "No goal states set";
    return false;
  }

  return true;
}

// std::vector<StJpsNodePtr> StJps::GetNeighbors(const StJpsNodePtr& node) {
//   // 1. 函数入口日志
//   // LOG(INFO) << "=== GetNeighbors start for node: " << node->DebugString();

//   std::vector<StJpsNodePtr> neighbors;

//   // 2. 采样配置日志 
//   // LOG(INFO) << "Acceleration sampling config:"
//   //           << " min_acc=" << config_.min_acceleration
//   //           << " max_acc=" << config_.max_acceleration
//   //           << " num_samples=" << config_.num_sample_a;
  
//   // 采样加速度
//   std::vector<double> acc_samples;
//   double da = (config_.max_acceleration - config_.min_acceleration) / 
//               (config_.num_sample_a - 1);
              
//   for (int i = 0; i < config_.num_sample_a; ++i) {
//     acc_samples.push_back(config_.min_acceleration + i * da);
//   }

//   // 3. 采样结果日志
//   // LOG(INFO) << "Generated " << acc_samples.size() << " acceleration samples:";
//   // for(auto acc : acc_samples) {
//   //   LOG(INFO) << "  acc=" << acc;
//   // }

//   // 根据运动学模型生成邻居节点
//   for (double acc : acc_samples) {
//     double dt = config_.step_length;
//     double new_s = node->s() + node->v() * dt + 0.5 * acc * dt * dt;
//     double new_v = node->v() + acc * dt;

//     // // 4. 记录每个候选邻居节点
//     // LOG(INFO) << "Candidate neighbor:" 
//     //           << " dt=" << dt
//     //           << " new_s=" << new_s 
//     //           << " new_v=" << new_v
//     //           << " acc=" << acc;
    
//     auto next = std::make_shared<StJpsNode>(
//         node->t() + dt, new_s, new_v, acc);
    
//     if (ValidateNode(next)) {
//       neighbors.push_back(next);
//        // 5. 记录有效的邻居节点
//       // LOG(INFO) << "Valid neighbor found: " << next->DebugString();
//     }else {
//       // 6. 记录被过滤掉的节点原因
//       // LOG(INFO) << "Invalid neighbor filtered: " << next->DebugString();
//     }
//   }
//   // 7. 函数返回日志  
//   // LOG(INFO) << "=== GetNeighbors complete. Generated " 
//   //           << neighbors.size() << " valid neighbors";

//   return neighbors;
// }

// std::vector<StJpsNodePtr> StJps::GetNeighbors(const StJpsNodePtr& node) {
//  std::vector<StJpsNodePtr> neighbors;
 
//  // 扩大时间步长和加速度范围
//  std::vector<double> time_steps = {0.5, 0.8, 1.0};
//  std::vector<double> acc_samples;
//  double da = (config_.max_acceleration - config_.min_acceleration) / 4;
//  for (double a = config_.min_acceleration; 
//       a <= config_.max_acceleration; 
//       a += da) {
//    acc_samples.push_back(a);
//  }

//  // 节点扩展
//  for (double dt : time_steps) {
//    for (double acc : acc_samples) {
//      double new_v = node->v() + acc * dt;
//      if (new_v < 0) continue;
     
//      double new_s = node->s() + node->v() * dt + 0.5 * acc * dt * dt;
//      auto next = std::make_shared<StJpsNode>(
//          node->t() + dt, new_s, new_v, acc);
         
//      if (ValidateNode(next)) {
//        neighbors.push_back(next);
//      }
//    }
//  }

//  LOG(INFO) << "Generated " << neighbors.size() 
//            << " neighbors for node at s=" << node->s();
           
//  return neighbors;
// }

// 1. GetNeighbors函数调整
std::vector<StJpsNodePtr> StJps::GetNeighbors(const StJpsNodePtr& node) {
  // 更大时间步长
  const std::vector<double> time_steps = {0.8, 1.2, 1.6};
  
  // 基于期望速度计算加速度范围
  double v_target = config_.max_velocity;
  double t_to_target = 2 * (v_target - node->v()) / config_.max_acceleration;
  
  std::vector<double> acc_samples;
  double da = (config_.max_acceleration - config_.min_acceleration) / 4.0;
  for (double a = config_.min_acceleration; a <= config_.max_acceleration; a += da) {
    acc_samples.push_back(a);
  }

  std::vector<StJpsNodePtr> neighbors;
  for (double dt : time_steps) {
    for (double acc : acc_samples) {
      double new_v = node->v() + acc * dt; 
      double new_s = node->s() + node->v() * dt + 0.5 * acc * dt * dt;
      
      if (new_s > node->s() + 1.0) { // 要求最小前进距离
        auto next = std::make_shared<StJpsNode>(node->t() + dt, new_s, new_v, acc);
        if (ValidateNode(next)) {
          neighbors.push_back(next);
        }
      }
    }
  }

  // 精简日志
  if (!neighbors.empty()) {
    LOG(INFO) << "Expanded " << neighbors.size() << " nodes from s=" << node->s();
  }
  return neighbors;
}



// double StJps::CalcHeuristic(const StJpsNodePtr& node,
//                            const std::vector<StJpsNodePtr>& goals) {
//   if (goals.empty()) {
//     return 0.0;
//   }

//   // 计算到所有目标点的最小启发代价
//   double min_cost = std::numeric_limits<double>::max();
//   for (const auto& goal : goals) {
//     // 时间距离代价
//     double dt = std::abs(goal->t() - node->t());
    
//     // 位置距离代价
//     double ds = std::abs(goal->s() - node->s());
    
//     // 速度差异代价 
//     double dv = std::abs(goal->v() - node->v());
    
//     double cost = config_.w_reference * (dt + ds) + 
//                  config_.w_smooth * dv;
                 
//     min_cost = std::min(min_cost, cost);
//   }

//   return min_cost;
// }


// 启发式函数优化:增加纵向驱动力和速度渐进性
// double StJps::CalcHeuristic(const StJpsNodePtr& node,
//                           const std::vector<StJpsNodePtr>& goals) {
//  if (goals.empty()) return 0.0;

//  double min_cost = std::numeric_limits<double>::max(); 
//  for (const auto& goal : goals) {
//    // 纵向距离(增大权重)
//    double ds = std::abs(goal->s() - node->s());
   
//    // 速度差异(渐进性约束) 
//    double dv = std::abs(goal->v() - node->v());
//    double v_progress = std::max(0.0, node->v() - config_.max_velocity);
   
//    // 时间差异
//    double dt = std::abs(goal->t() - node->t());
   
//    // 总代价
//    double cost = 3.0 * ds +                     // 纵向权重增大
//                 1.0 * dt +                      // 时间权重降低
//                 0.5 * dv +                      // 速度差异权重降低 
//                 2.0 * v_progress;               // 惩罚速度超限
                
//    min_cost = std::min(min_cost, cost);
//  }

//  return min_cost;
// }


double StJps::CalcHeuristic(const StJpsNodePtr& node,
                         const std::vector<StJpsNodePtr>& goals) {
 if (goals.empty()) return 0.0;

 double min_cost = std::numeric_limits<double>::max();
 for (const auto& goal : goals) {
   // s方向权重大幅提高
   double ds = std::abs(goal->s() - node->s());
   double dv = std::abs(goal->v() - node->v());
   double dt = std::abs(goal->t() - node->t());
   
   double cost = 8.0 * ds +     // s方向权重提高 
                0.5 * dv +      // 降低速度权重
                0.3 * dt;       // 降低时间权重
                
   min_cost = std::min(min_cost, cost);
 }

 return min_cost;
}


// double StJps::CalcTransitionCost(const StJpsNodePtr& from,
//                                 const StJpsNodePtr& to) {
//   double cost = 0.0;

//   // 障碍物代价
//   Eigen::Vector2d pos(to->t(), to->s());
//   double dist = sdf_->SignedDistance(pos);
//   cost += config_.w_obstacle * std::exp(-dist / config_.safe_distance);

//   // 速度参考代价
//   double dv = std::abs(to->v() - config_.max_velocity);
//   cost += config_.w_reference * dv;

//   // 平滑代价
//   double da = std::abs(to->a() - from->a());
//   cost += config_.w_smooth * da;

//   return cost;
// }

// 转移代价优化:平衡安全性和效率
double StJps::CalcTransitionCost(const StJpsNodePtr& from,
                               const StJpsNodePtr& to) {
 // 纵向代价
 double ds = to->s() - from->s();
 double forward_cost = 2.0 * std::max(0.0, -ds);  // 惩罚后退

 // 速度代价:鼓励渐进加速
 double dv = to->v() - from->v();
 double vel_cost = 0.0;
 if (dv > 0) {
   vel_cost = 0.5 * dv;  // 允许适度加速
 } else {
   vel_cost = std::abs(dv);  // 惩罚减速
 }
 
 // 加速度平滑代价
 double acc_cost = 0.8 * std::abs(to->a());

 // 障碍物安全代价
 Eigen::Vector2d pos(to->t(), to->s());
 double obs_dist = sdf_->SignedDistance(pos);
 double obs_cost = 3.0 * std::exp(-obs_dist / config_.safe_distance);

 return forward_cost + vel_cost + acc_cost + obs_cost;
}

// bool StJps::IsJumpPoint(const StJpsNodePtr& node,
//                        const std::vector<StJpsNodePtr>& neighbors) {
//   if (neighbors.empty()) {
//     return false;
//   }

//   // 起点或终点为跳点
//   if (node->parent() == nullptr || IsGoalNode(node)) {
//     return true;
//   }

//   // 计算搜索方向
//   double dt = node->t() - node->parent()->t();
//   double ds = node->s() - node->parent()->s();
//   Eigen::Vector2d dir(dt, ds);
//   dir.normalize();

//   // 检查强制邻居
//   int forced_neighbors = 0;
//   for (const auto& next : neighbors) {
//     Eigen::Vector2d next_dir(next->t() - node->t(),
//                             next->s() - node->s());
//     next_dir.normalize();

//     // 方向变化大,说明存在强制邻居
//     if (std::abs(dir.dot(next_dir)) < 0.6) {
//       forced_neighbors++;
//     }
//   }

//   return forced_neighbors > 0;
// }

// bool StJps::IsJumpPoint(const StJpsNodePtr& node,
//                       const std::vector<StJpsNodePtr>& neighbors) {
//  // 定义所有阈值常量
//  const double kVelChangeThreshold = 0.3 * config_.max_velocity;  // 速度变化阈值(10%参考速度)
//  const double kObsDistThreshold = config_.safe_distance * 1.2;   // 安全距离的1.5倍
//  const double kVelRefThreshold = 0.95 * config_.max_velocity;     // 参考速度的90%
 
//  // 统计日志
//  static int check_count = 0;
//  static int jump_point_count = 0;
//  check_count++;

//  // 每1000个节点输出统计
//  if (check_count % 1000 == 0) {
//    LOG(INFO) << "=== Jump Point Stats ===\n"
//              << "Total checked: " << check_count << "\n"
//              << "Jump points: " << jump_point_count << "\n" 
//              << "Current node: " << node->DebugString();
//  }

//  bool is_jump = false;
//  std::string reason;

//  // 1. 起点或终点必定是跳点
//  if (node->parent() == nullptr || IsGoalNode(node)) {
//    is_jump = true;
//    reason = "Start/Goal node";
//  }

//  // 2. 速度或加速度变化较大的点是跳点 
//  if (!is_jump && node->parent() != nullptr) {
//    double vel_change = std::abs(node->v() - node->parent()->v());
//    if (vel_change > kVelChangeThreshold) {
//      is_jump = true;
//      reason = "Large velocity change: " + std::to_string(vel_change);
//    }
//    // 加速度变号
//    if (node->parent()->a() * node->a() < 0) {
//      is_jump = true;
//      reason = "Acceleration direction change";
//    }
//  }

//  // 3. 接近障碍物的点是跳点
//  if (!is_jump) {
//    Eigen::Vector2d pos(node->t(), node->s());
//    double dist = sdf_->SignedDistance(pos);
//    if (dist < kObsDistThreshold) {
//      is_jump = true; 
//      reason = "Close to obstacle: dist=" + std::to_string(dist);
//    }
//  }

//  // 4. 速度接近参考速度时是跳点
//  if (!is_jump && std::abs(node->v()) > kVelRefThreshold) {
//    is_jump = true;
//    reason = "Near reference velocity";
//  }

//  // 5. 检查邻居节点变化趋势
// if (!is_jump) {
//     int significant_changes = 0;
//     for (const auto& next : neighbors) {
//         if (std::abs(next->v() - node->v()) > kVelChangeThreshold || 
//             next->a() * node->a() < 0) {
//             significant_changes++;
//         }
//     }
//     // 要求至少30%的邻居节点有显著变化
//     if (significant_changes > neighbors.size() * 0.3) {
//         is_jump = true;
//         reason = "Multiple significant neighbor changes";
//     }
// }

//  // 记录跳点
//  if (is_jump) {
//    jump_point_count++;
//    if (jump_point_count % 500 == 0) {
//      LOG(INFO) << "Jump point #" << jump_point_count << ": " << node->DebugString() 
//                << "\nReason: " << reason
//                << "\nParent: " << (node->parent() ? node->parent()->DebugString() : "null");
//    }
//  }

//  return is_jump;
// }


// bool StJps::IsJumpPoint(const StJpsNodePtr& node,
//                       const std::vector<StJpsNodePtr>& neighbors) {
//   // 提高阈值要求
//   const double kVelChangeThreshold = 0.8 * config_.max_velocity;
//   const double kAccChangeThreshold = 0.3 * config_.max_acceleration;
//   const double kObsDistThreshold = config_.safe_distance * 1.5;
  
//   // 节点检查统计(每1000个节点输出一次)
//   static int check_count = 0;
//   static int jump_point_count = 0;
//   check_count++;
  
//   if (check_count % 1000 == 0) {
//     LOG(INFO) << "Jump Point Check Stats:"
//               << " Total:" << check_count 
//               << " JumpPoints:" << jump_point_count
//               << " Node:" << node->DebugString();
//   }

//   bool is_jump = false;
//   std::string reason;

//   // 1. 起点或终点判断
//   if (node->parent() == nullptr || IsGoalNode(node)) {
//     is_jump = true;
//     reason = "Start/Goal node";
//   }

//   // 速度增大且接近参考速度时一定保留
//   // if (node->v() > node->parent()->v() && 
//   //     node->v() > 0.7 * config_.max_velocity) {
//   //   return true; 
//   // }

//   // 先判断parent是否为空
// if (node->parent() && 
//     node->v() > node->parent()->v() && 
//     node->v() > 0.7 * config_.max_velocity) {
//   is_jump = true;
//   reason = "Approaching target velocity";
// }

//   // 2. 关键状态变化判断
//   if (!is_jump && node->parent()) {
//     // 显著的速度突变
//     double vel_change = std::abs(node->v() - node->parent()->v());
//     if (vel_change > kVelChangeThreshold) {
//       is_jump = true;
//       reason = "Velocity jump: " + std::to_string(vel_change);
//     }

//     // 加速度方向反转且幅值较大
//     if (node->parent()->a() * node->a() < 0 && 
//         std::abs(node->a()) > kAccChangeThreshold) {
//       is_jump = true;
//       reason = "Acceleration reversal: " + std::to_string(node->a());
//     }
//   }

//   // 3. 障碍物相关判断
//   if (!is_jump) {
//     Eigen::Vector2d pos(node->t(), node->s());
//     double obs_dist = sdf_->SignedDistance(pos);
    
//     // 接近障碍物且有速度变化
//     if (obs_dist < kObsDistThreshold && node->parent() &&
//         std::abs(node->v() - node->parent()->v()) > 0.2 * config_.max_velocity) {
//       is_jump = true;
//       reason = "Near obstacle with velocity change: dist=" + std::to_string(obs_dist);
//     }
//   }

//   // 仅记录每500个跳点的日志
//   if (is_jump) {
//     jump_point_count++;
//     if (jump_point_count % 500 == 0) {
//       LOG(INFO) << "Jump point #" << jump_point_count 
//                 << "\nNode: " << node->DebugString()
//                 << "\nReason: " << reason 
//                 << "\nParent: " << (node->parent() ? node->parent()->DebugString() : "null");
//     }
//   }

//   return is_jump;
// }

// bool StJps::IsJumpPoint(const StJpsNodePtr& node,
//                      const std::vector<StJpsNodePtr>& neighbors) {
//  // 配置阈值                    
//  const double kVelChangeThreshold = 0.5 * config_.max_velocity;
//  const double kAccChangeThreshold = 0.3 * config_.max_acceleration; 
//  const double kObsDistThreshold = config_.safe_distance * 1.5;
//  const double kMinProgressThreshold = 1.0; // 最小纵向进展
 
//  // 日志统计
//  static int check_count = 0;
//  static int jump_point_count = 0;
//  check_count++;
 
//  if (check_count % 1000 == 0) {
//    LOG(INFO) << "Jump Point Stats - Total:" << check_count 
//              << " JumpPoints:" << jump_point_count
//              << " Node:" << node->DebugString();
//  }

//  bool is_jump = false;
//  std::string reason;

//  // 1. 起点或终点必定是跳点
//  if (!node->parent() || IsGoalNode(node)) {
//    is_jump = true;
//    reason = "Start/Goal node";
//  }

//  // 2. 显著纵向进展是跳点
//  if (!is_jump && node->parent()) {
//    double ds = node->s() - node->parent()->s();
//    if (ds > kMinProgressThreshold) {
//      is_jump = true;
//      reason = "Progress jump:" + std::to_string(ds);
//    }
//  }

//  // 3. 速度接近目标或显著变化是跳点
//  if (!is_jump && node->parent()) {
//    if (std::abs(node->v() - config_.max_velocity) < 2.0) {
//      is_jump = true;
//      reason = "Near target velocity";
//    }
   
//    double dv = std::abs(node->v() - node->parent()->v());
//    if (dv > kVelChangeThreshold) {
//      is_jump = true; 
//      reason = "Velocity jump:" + std::to_string(dv);
//    }
//  }

//  // 4. 加速度显著变化是跳点
//  if (!is_jump && node->parent()) {
//    if (node->parent()->a() * node->a() < 0 && 
//        std::abs(node->a()) > kAccChangeThreshold) {
//      is_jump = true;
//      reason = "Acc reversal:" + std::to_string(node->a());
//    }
//  }

//  // 5. 接近障碍物且速度变化是跳点
//  if (!is_jump && node->parent()) {
//    Eigen::Vector2d pos(node->t(), node->s());
//    double obs_dist = sdf_->SignedDistance(pos);
//    if (obs_dist < kObsDistThreshold &&
//        std::abs(node->v() - node->parent()->v()) > 0.2 * config_.max_velocity) {
//      is_jump = true;
//      reason = "Near obstacle:" + std::to_string(obs_dist);
//    }
//  }

//  if (is_jump) {
//    jump_point_count++;
//    if (jump_point_count % 500 == 0) {
//      LOG(INFO) << "Jump point #" << jump_point_count 
//                << "\nNode:" << node->DebugString()
//                << "\nReason:" << reason
//                << "\nParent:" << (node->parent() ? node->parent()->DebugString() : "null");
//    }
//  }

//  return is_jump;
// }

// 核心函数实现:
bool StJps::IsJumpPoint(const StJpsNodePtr& node,
                     const std::vector<StJpsNodePtr>& neighbors) {
 // 日志统计
 static int check_count = 0;
 static int jump_point_count = 0;
 check_count++;
 
 if (check_count % 1000 == 0) {
   LOG(INFO) << "Jump Point Stats - Total:" << check_count 
             << " JumpPoints:" << jump_point_count
             << " Node:" << node->DebugString()
             << " s=" << node->s();
 }

 bool is_jump = false;
 std::string reason;

 // 1. 起点终点判断
 if (!node->parent() || IsGoalNode(node)) {
   is_jump = true;
   reason = "Start/Goal node";
 }

 // 2. s方向显著进展
 if (!is_jump && node->parent()) {
   double ds = node->s() - node->parent()->s();
   if (ds > 3) {
     is_jump = true;
     reason = "Forward progress:" + std::to_string(ds);
   }
 }

 // 3. 速度接近目标
 if (!is_jump && std::abs(node->v() - config_.max_velocity) < 2.0) {
   is_jump = true;
   reason = "Near target velocity";
 }

 // 4. 障碍物相关判断
 if (!is_jump) {
   Eigen::Vector2d pos(node->t(), node->s());
   double obs_dist = sdf_->SignedDistance(pos);
   if (obs_dist < config_.safe_distance * 1.2) {
     is_jump = true;
     reason = "Near obstacle:" + std::to_string(obs_dist);
   }
 }

 if (is_jump) {
   jump_point_count++;
   if (jump_point_count % 500 == 0) {
     LOG(INFO) << "Jump point #" << jump_point_count 
               << "\nNode:" << node->DebugString()
               << "\nReason:" << reason;
   }
 }

 return is_jump;
}


bool StJps::IsGoalNode(const StJpsNodePtr& node) {
  if (goals_.empty()) {
    return false;
  }

  for (const auto& goal : goals_) {
    if (std::abs(node->t() - goal->t()) < config_.grid_resolution &&
        std::abs(node->s() - goal->s()) < config_.grid_resolution &&
        std::abs(node->v() - goal->v()) < 0.5) {
      return true;
    }
  }
  
  return false;
}

void StJps::BuildResult(const StJpsNodePtr& final_node,
                       std::vector<StJpsNode>* result) {
  CHECK_NOTNULL(result);
  result->clear();

  // 回溯构建原始路径
  std::vector<StJpsNodePtr> raw_path;
  StJpsNode* current = final_node.get();
  while (current != nullptr) {
    raw_path.push_back(std::make_shared<StJpsNode>(*current));
    current = current->parent();
  }

  std::reverse(raw_path.begin(), raw_path.end());

  // 重连接路径
  ReconnectPath(&raw_path);

  // 轨迹平滑
  SmoothTrajectory(&raw_path); 

  // 转换为结果格式
  for (const auto& node : raw_path) {
    result->push_back(*node);
  }
}

void StJps::ReconnectPath(std::vector<StJpsNodePtr>* path) {
  if (path->empty()) {
    return;
  }

  std::vector<StJpsNodePtr> reconnected_path;
  reconnected_path.push_back(path->front());

  for (size_t i = 1; i < path->size(); ++i) {
    const auto& current = reconnected_path.back();
    const auto& next = (*path)[i];

    // 插值得到中间节点
    auto interpolated = GetInterpolatedNodes(current, next);

    // 添加合法的插值节点
    for (const auto& node : interpolated) {
      if (ValidateNode(node) && 
          CheckTrajectoryFeasible(current, node)) {
        reconnected_path.push_back(node);
      }
    }

    reconnected_path.push_back(next);
  }

  *path = std::move(reconnected_path);
}

std::vector<StJpsNodePtr> StJps::GetInterpolatedNodes(
    const StJpsNodePtr& from, const StJpsNodePtr& to) {
  std::vector<StJpsNodePtr> nodes;

  double dt = to->t() - from->t();
  double ds = to->s() - from->s();
  int num_segments = std::ceil(
      std::max(std::abs(dt), std::abs(ds)) / config_.step_length);

  if (num_segments <= 1) {    
    return nodes;   
  }

  // 线性插值
  double dt_step = dt / num_segments;
  double ds_step = ds / num_segments;
  double dv = (to->v() - from->v()) / num_segments;
  double da = (to->a() - from->a()) / num_segments;

  for (int i = 1; i < num_segments; ++i) {
    double t = from->t() + dt_step * i;
    double s = from->s() + ds_step * i;
    double v = from->v() + dv * i;
    double a = from->a() + da * i;

    nodes.push_back(std::make_shared<StJpsNode>(t, s, v, a));
  }

  return nodes;
}

void StJps::SmoothTrajectory(std::vector<StJpsNodePtr>* path) {
  if (path->size() <= 2) {
    return;
  }

  constexpr int window_size = 3;
  std::vector<StJpsNodePtr> smoothed = *path;

  for (size_t i = window_size; i < path->size() - window_size; ++i) {
    double v_sum = 0.0;
    double a_sum = 0.0;
    
    for (int j = -window_size; j <= window_size; ++j) {
      v_sum += (*path)[i + j]->v();
      a_sum += (*path)[i + j]->a();
    }

    smoothed[i] = std::make_shared<StJpsNode>(
        (*path)[i]->t(),
        (*path)[i]->s(),
        v_sum / (2 * window_size + 1),
        a_sum / (2 * window_size + 1));
  }

  *path = std::move(smoothed);
}

bool StJps::ValidateNode(const StJpsNodePtr& node) {
  // 速度约束
  // if (node->v() > config_.max_velocity ||  node->v() < config_.min_velocity)
  if (node->v() > config_.max_velocity ) {
      LOG_IF(INFO, node->v() > config_.max_velocity) 
        << "Node velocity " << node->v() << " exceeds max " << config_.max_velocity;
    // LOG_IF(INFO, node->v() < config_.min_velocity) 
    //     << "Node velocity " << node->v() << " below min " << config_.min_velocity;
    return false;
  }

  // 加速度约束
  if (node->a() > config_.max_acceleration || 
      node->a() < config_.min_acceleration) {
    LOG_IF(INFO, node->a() > config_.max_acceleration) 
        << "Node acceleration " << node->a() << " exceeds max " << config_.max_acceleration;
    LOG_IF(INFO, node->a() < config_.min_acceleration) 
        << "Node acceleration " << node->a() << " below min " << config_.min_acceleration;
    return false;
  }

  // 边界检查
  Eigen::Vector2d pos(node->t(), node->s());
  if (!sdf_->occupancy_map().IsInMap(pos)) {
    return false;
  }

  // 碰撞检查
  double dist = sdf_->SignedDistance(pos);
  if (dist < config_.safe_distance) {
    LOG_IF(INFO, dist < config_.safe_distance)
        << "Node too close to obstacle: dist=" << dist 
        << " min_required=" << config_.safe_distance;
    return false;
  }

  return true;
}

bool StJps::CheckTrajectoryFeasible(const StJpsNodePtr& from,
                                   const StJpsNodePtr& to) {
  double dt = to->t() - from->t();
  if (dt <= 0.0) return false;

  // 检查轨迹中间点是否满足约束
  int check_points = std::ceil(dt / config_.time_resolution);
  
  for (int i = 1; i < check_points; ++i) {
    double t = from->t() + i * config_.time_resolution;
    // 二次插值计算位置
    double ratio = (t - from->t()) / dt;
    double s = from->s() + (to->s() - from->s()) * ratio;
    
    Eigen::Vector2d pos(t, s);
    if (sdf_->SignedDistance(pos) < config_.safe_distance) {
      return false;
    }
  }

  return true;
}

} // namespace planning