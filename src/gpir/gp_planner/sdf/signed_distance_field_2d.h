/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#pragma once

#include <utility>
#include <vector>

#include "gp_planner/sdf/grid_map_2d.h"
// 添加感知不确定性头文件
#include "planning_core/planning_common/perception_uncertainty.h"
#include <Eigen/Dense>  // 确保Eigen头文件也被包含

namespace planning {

/**
 * @class SignedDistanceField2D
 * @brief two dimensional euclidean signed distance field
 *        coordinate: left-bottom cornor is the origin
 *
 *
 *         y ^_______________________________
 *  Y  ^     |                               |
 *     |     |     Signed distance field     |
 *     |     |                               |
 *     |     o-------->______________________|
 *     |  origin     x
 *     |
 *     O----------------->X
 *   Global Coordinate
 */

using OccupancyMap = GridMap2D<uint8_t>;
using DistanceMap = GridMap2D<double>;

class SignedDistanceField2D {
 public:
  SignedDistanceField2D(std::array<double, 2> origin, std::array<int, 2> dim,
                        const double map_resolution);
  SignedDistanceField2D(OccupancyMap&& occupancy_map);
  ~SignedDistanceField2D() = default;

  inline void set_origin(std::array<double, 2> origin);
  inline void set_resolution(const double map_resolution);
  inline void set_cell_num(std::array<int, 2> cell_num);
  inline void resize(std::array<int, 2> dim);

  inline const OccupancyMap& occupancy_map() const { return occupancy_map_; }
  inline OccupancyMap* mutable_occupancy_map() { return &occupancy_map_; }

  inline const DistanceMap& esdf() const { return esdf_; }
  inline DistanceMap* mutable_esdf() { return &esdf_; }

  void UpdateSDF();

  // * only calculate distance along y axis
  void UpdateVerticalSDF();

  inline double SignedDistance(const Eigen::Vector2d& coord,
                               Eigen::Vector2d* grad = nullptr) const {
    return esdf_.GetValueBilinear(coord, grad);
  }

  /**
   * @brief 考虑不确定性的距离场查询
   * @param coord 查询点的坐标
   * @param uncertainty 感知不确定性信息
   * @param grad 可选的梯度输出
   * @return 修正后的有向距离
   */
  double GetSignedDistanceWithUncertainty(
      const Eigen::Vector2d& coord,
      const PerceptionUncertainty& uncertainty,
      Eigen::Vector2d* grad = nullptr) const;

  /**
   * @brief 检查给定点是否在考虑不确定性的情况下安全
   * @param coord 待检查点的坐标
   * @param uncertainty 感知不确定性信息
   * @param safety_threshold 安全阈值
   * @param collision_prob 输出的碰撞概率
   * @return 是否安全
   */
  bool CheckSafetyWithUncertainty(
      const Eigen::Vector2d& coord,
      const PerceptionUncertainty& uncertainty,
      double safety_threshold,
      double* collision_prob = nullptr) const;

 protected:
  void EuclideanDistanceTransform(
      std::array<int, 2> dim,
      std::function<bool(const int x, const int y)> is_occupied,
      DistanceMap* output_map);

  // * only perform column scan
  void VerticalEuclideanDistanceTransform(
      std::array<int, 2> dim,
      std::function<bool(const int x, const int y)> is_occupied,
      DistanceMap* output_map);
  
  // 获取静态障碍物位置列表
  const std::vector<Eigen::Vector2d>& static_obstacle_positions() const {
    return static_obstacle_positions_;
  }
 private:
  double map_resolution_;

  OccupancyMap occupancy_map_;
  DistanceMap esdf_;

  // 存储静态障碍物位置
  std::vector<Eigen::Vector2d> static_obstacle_positions_;
};

// inline function
inline void SignedDistanceField2D::set_origin(std::array<double, 2> origin) {
  occupancy_map_.set_origin(origin);
  esdf_.set_origin(origin);
}

inline void SignedDistanceField2D::set_resolution(const double map_resolution) {
  map_resolution_ = map_resolution;
  occupancy_map_.set_resolution(
      std::array<double, 2>{map_resolution, map_resolution});
  esdf_.set_resolution(std::array<double, 2>{map_resolution, map_resolution});
}

inline void SignedDistanceField2D::set_cell_num(std::array<int, 2> cell_num) {
  occupancy_map_.set_cell_number(cell_num);
  esdf_.set_cell_number(cell_num);
}
}  // namespace planning根据编译错误，修改那些位置，同时再给你补充一些已经修改过的代码。