/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#pragma once


// 前向声明
namespace planning {
class PlanningCore;  // 前向声明
}

#include "planning_core/navigation/navigation_map.h"

namespace planning {

class Planner {

protected:
  PlanningCore* parent_{nullptr};

 public:
  Planner() = default;

  virtual void SetStaticObstacleUncertainty(
        const PerceptionUncertainty& uncertainty) = 0;
        
  virtual ~Planner() = default;

  virtual void Init() = 0;
  virtual void PlanOnce(NavigationMap* navigation_map_) = 0;
  virtual void LogDebugInfo() {}

  void SetParent(PlanningCore* parent) { parent_ = parent; }
};

}  // namespace planning
