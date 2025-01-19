set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(gp_planner_gp)
add_library(gp_planner::gp ALIAS gp_planner_gp)

target_sources(gp_planner_gp
  PRIVATE
    factors/gp_prior_factor.cc
    factors/gp_obstacle_factor.cc
    factors/gp_kappa_limit_factor.cc
    factors/gp_lat_acc_limit_factor.cc
    factors/gp_interpolate_obstacle_factor.cc
    factors/gp_interpolate_kappa_limit_factor.cc
    factors/gp_perception_uncertainty_factor.cc
    factors/static_obstacle_uncertainty_factor.cc
    factors/control_uncertainty_factor.cc
    utils/gp_utils.cc
    utils/gp_path.cc
    gp_incremental_path_planner.cc
)

# 可以使用 target_compile_features 来设置特定目标的 C++ 标准
target_compile_features(gp_planner_gp PUBLIC cxx_std_17)

target_link_libraries(gp_planner_gp PUBLIC
  gtsam
  planning_core::navigation
  common::smoothing
  gp_planner::initializer
  planning_core_common
)

target_include_directories(gp_planner_gp
  PUBLIC
    ${CMAKE_SOURCE_DIR}  # 确保这行存在
)