#pragma once

#include <Eigen/Dense>
#include <memory>
#include <vector>

#include "common/base/state.h" 
#include "common/frenet/frenet_state.h"

namespace planning {

// JPS节点的搜索状态 
enum class JpsNodeStatus {
  kUnknown = 0,
  kOpen = 1,     // 在openset中
  kClosed = 2,   // 在closedset中
};

class StJpsNode {
 public:
  StJpsNode() = default;
  StJpsNode(const double t, const double s);
  StJpsNode(const double t, const double s, const double v, const double a);
  
  // ST图上的状态量
  double t() const { return t_; }
  double s() const { return s_; }
  double v() const { return v_; }
  double a() const { return a_; } 
  
  // 总代价 = g代价 + h代价
  double f_cost() const { return g_cost_ + h_cost_; }
  double g_cost() const { return g_cost_; }
  double h_cost() const { return h_cost_; }

  // 设置代价
  void set_g_cost(double cost) { g_cost_ = cost; }
  void set_h_cost(double cost) { h_cost_ = cost; }
  
  // 父节点
  StJpsNode* parent() { return parent_; }
  void set_parent(StJpsNode* parent) { parent_ = parent; }

  // 搜索状态
  JpsNodeStatus status() const { return status_; }
  void set_status(JpsNodeStatus status) { status_ = status; }

  // 辅助功能
  bool IsSameState(const StJpsNode& other) const;
  std::string DebugString() const;
  
private:
  // ST状态量
  double t_ = 0.0;  // 时间
  double s_ = 0.0;  // 位置
  double v_ = 0.0;  // 速度  
  double a_ = 0.0;  // 加速度

  // 搜索相关
  double g_cost_ = 0.0;   // 实际代价
  double h_cost_ = 0.0;   // 启发代价
  StJpsNode* parent_ = nullptr;  // 父节点
  JpsNodeStatus status_ = JpsNodeStatus::kUnknown;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// 节点比较器 - 用于优先队列
// struct StJpsNodeComparator {
//   bool operator()(const StJpsNode* node1, const StJpsNode* node2) {
//     return node1->f_cost() > node2->f_cost();
//   }
// };
struct StJpsNodeComparator {
  bool operator()(const std::shared_ptr<StJpsNode>& node1, 
                 const std::shared_ptr<StJpsNode>& node2) {
    return node1->f_cost() > node2->f_cost();
  }
};

using StJpsNodePtr = std::shared_ptr<StJpsNode>;

} // namespace planning