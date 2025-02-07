#include "gp_planner/st_plan/jps/st_jps_node.h"
#include <sstream>

namespace planning {

StJpsNode::StJpsNode(const double t, const double s) 
    : t_(t), s_(s) {}

StJpsNode::StJpsNode(const double t, const double s,
                     const double v, const double a)
    : t_(t), s_(s), v_(v), a_(a) {}

bool StJpsNode::IsSameState(const StJpsNode& other) const {
  static constexpr double kEpsilon = 1e-6;
  return std::abs(t_ - other.t_) < kEpsilon && 
         std::abs(s_ - other.s_) < kEpsilon &&
         std::abs(v_ - other.v_) < kEpsilon;
}

std::string StJpsNode::DebugString() const {
  std::ostringstream oss;
  oss << "StJpsNode: "
      << "t=" << t_ 
      << ", s=" << s_
      << ", v=" << v_ 
      << ", a=" << a_
      << ", f_cost=" << f_cost()
      << ", g_cost=" << g_cost_
      << ", h_cost=" << h_cost_;
  return oss.str();
}

} // namespace planning