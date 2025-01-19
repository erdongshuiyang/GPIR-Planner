/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#include "gp_planner/sdf/signed_distance_field_2d.h"
#include <cmath>  // 添加这行,用于std::sqrt和std::erf

#include <omp.h>

#include <future>
#include <iostream>

namespace planning {

SignedDistanceField2D::SignedDistanceField2D(std::array<double, 2> origin,
                                             std::array<int, 2> dim,
                                             const double map_resolution)
    : map_resolution_(map_resolution) {
  occupancy_map_.set_origin(origin);
  occupancy_map_.set_cell_number(dim);
  occupancy_map_.set_resolution(
      std::array<double, 2>{map_resolution, map_resolution});
  esdf_.ResizeFrom(occupancy_map_);
}

SignedDistanceField2D::SignedDistanceField2D(OccupancyMap&& occupancy_map)
    : occupancy_map_(std::move(occupancy_map)) {
  esdf_.ResizeFrom(occupancy_map_);
  map_resolution_ = occupancy_map_.resolution()[0];
}

void SignedDistanceField2D::EuclideanDistanceTransform(
    std::array<int, 2> dim,
    std::function<bool(const int x, const int y)> is_occupied,
    DistanceMap* output_map) {
  int inf = dim[0] + dim[1] + 10;

  std::vector<std::vector<int>> g(dim[0], std::vector<int>(dim[1], 0));
  omp_set_num_threads(4);
  {
#pragma omp parallel for
    // column scan
    for (int x = 0; x < dim[0]; ++x) {
      g[x][0] = is_occupied(x, 0) ? 0 : inf;

      for (int y = 1; y < dim[1]; ++y) {
        g[x][y] = is_occupied(x, y) ? 0 : 1 + g[x][y - 1];
      }

      for (int y = dim[1] - 2; y >= 0; --y) {
        if (g[x][y + 1] < g[x][y]) g[x][y] = 1 + g[x][y + 1];
      }
    }
  }

  // row scan
  omp_set_num_threads(4);
  {
#pragma omp parallel for
    for (int y = 0; y < dim[1]; ++y) {
      int q = 0, w;
      std::vector<int> s(dim[0], 0);
      std::vector<int> t(dim[0], 0);

      auto f = [&g, &y](int x, int i) -> double {
        return (x - i) * (x - i) + g[i][y] * g[i][y];
      };

      for (int u = 1; u < dim[0]; ++u) {
        while (q >= 0 && f(t[q], s[q]) > f(t[q], u)) {
          --q;
        }

        if (q < 0) {
          q = 0;
          s[0] = u;
        } else {
          w = 1 + std::floor((u * u - s[q] * s[q] + g[u][y] * g[u][y] -
                              g[s[q]][y] * g[s[q]][y]) /
                             (2 * (u - s[q])));
          if (w < dim[0]) {
            ++q;
            s[q] = u;
            t[q] = w;
          }
        }
      }

      for (int u = dim[0] - 1; u >= 0; --u) {
        output_map->SetValue(u, y, map_resolution_ * std::sqrt(f(u, s[q])));
        if (u == t[q]) --q;
      }
    }
  }
}

void SignedDistanceField2D::VerticalEuclideanDistanceTransform(
    std::array<int, 2> dim,
    std::function<bool(const int x, const int y)> is_occupied,
    DistanceMap* output_map) {
  int inf = 1e9;

  std::vector<std::vector<int>> g(dim[0], std::vector<int>(dim[1], 0));
  omp_set_num_threads(4);
  {
#pragma omp parallel for
    // column scan
    for (int x = 0; x < dim[0]; ++x) {
      g[x][0] = is_occupied(x, 0) ? 0 : inf;

      for (int y = 1; y < dim[1]; ++y) {
        g[x][y] = is_occupied(x, y) ? 0 : 1 + g[x][y - 1];
      }

      for (int y = dim[1] - 2; y >= 0; --y) {
        if (g[x][y + 1] < g[x][y]) g[x][y] = 1 + g[x][y + 1];
      }

      for (int y = 0; y < dim[1]; ++y) {
        output_map->SetValue(x, y, map_resolution_ * g[x][y]);
      }
    }
  }
}

// void SignedDistanceField2D::UpdateSDF() {
//   LOG(INFO) << "\n================ SDF Update Start ================";
//   LOG(INFO) << "Map properties:"
//             << "\nOrigin: " << occupancy_map_.origin()[0] << ", " 
//             << occupancy_map_.origin()[1]
//             << "\nResolution: " << map_resolution_
//             << "\nDimensions: " << occupancy_map_.cell_num()[0] << " x " 
//             << occupancy_map_.cell_num()[1];

//   // 先清空静态障碍物位置列表
//   static_obstacle_positions_.clear();
  
//   // 构建距离图和反距离图
//   DistanceMap distance_map, inv_distance_map;
//   distance_map.ResizeFrom(occupancy_map_);
//   inv_distance_map.ResizeFrom(occupancy_map_);
//   OccupancyMap& map = occupancy_map_;

//   auto dim = occupancy_map_.cell_num();
//   for (int x = 0; x < dim[0]; ++x) {
//     for (int y = 0; y < dim[1]; ++y) {
//       if (occupancy_map_.IsOccupied(x, y)) {
//         Eigen::Vector2d coord;
//         occupancy_map_.Index2Coordinate(Eigen::Vector2i(x, y), &coord);
//         static_obstacle_positions_.push_back(coord);
//       }
//     }
//   }

//   // 打印障碍物位置信息
//   LOG(INFO) << "Number of static obstacles: " << static_obstacle_positions_.size();
//   for (const auto& pos : static_obstacle_positions_) {
//     LOG(INFO) << "Static obstacle at: " << pos.transpose();
//   }
  
//   LOG(INFO) << "================== SDF Update End ==================\n";

//   // 计算到障碍物的欧氏距离
//   EuclideanDistanceTransform(
//       dim,
//       [&map](const int x, const int y) -> bool { return map.IsOccupied(x, y); },
//       &distance_map);
//   // 计算到空闲空间的欧氏距离
//   EuclideanDistanceTransform(
//       dim,
//       [&map](const int x, const int y) -> bool {
//         return !map.IsOccupied(x, y);
//       },
//       &inv_distance_map);

//   const auto& dis_map_data = distance_map.data();
//   const auto& inv_dis_map_data = inv_distance_map.data();
//   auto& esdf_data = *esdf_.mutable_data();

//   omp_set_num_threads(4);
//   {
// #pragma omp parallel for
//  // 合并得到有符号距离场
//     for (int x = 0; x < dim[0]; ++x) {
//       for (int y = 0; y < dim[1]; ++y) {
//         int address = occupancy_map_.Index2Address(x, y);
//         esdf_data[address] = dis_map_data[address];
//         if (inv_dis_map_data[address] > 0) {
//           esdf_data[address] += (-inv_dis_map_data[address] + map_resolution_);
//         }
//       }
//     }
//   }
// }

void SignedDistanceField2D::UpdateSDF() {
  LOG(INFO) << "\n================ SDF Update Start ================";
  LOG(INFO) << "Map properties:"
            << "\nOrigin: " << occupancy_map_.origin()[0] << ", " 
            << occupancy_map_.origin()[1]
            << "\nResolution: " << map_resolution_
            << "\nDimensions: " << occupancy_map_.cell_num()[0] << " x " 
            << occupancy_map_.cell_num()[1];

  // 2. 清空地图阶段的日志
  auto dim = occupancy_map_.cell_num();
  // LOG(INFO) << "Clearing occupancy map...";
  // int total_cells = dim[0] * dim[1];
  // for (int x = 0; x < dim[0]; ++x) {
  //   for (int y = 0; y < dim[1]; ++y) {
  //     occupancy_map_.SetGridState(x, y, false);  // 设置为空闲状态
  //   }
  // }
  // LOG(INFO) << "Cleared " << total_cells << " cells in occupancy map";


  // // 3. 记录初始障碍物信息
  // static_obstacle_positions_.clear();
  // LOG(INFO) << "\n=== Processing input obstacles ===";
  // LOG(INFO) << "Initial static_obstacle_positions size: " 
  //           << static_obstacle_positions_.size();

 
  // // 4. 障碍物注册阶段的日志
  // LOG(INFO) << "\n=== Registering obstacles to grid ===";
  // int registered_obstacles = 0;
  // int out_of_bounds_obstacles = 0;
  //   // 遍历所有障碍物并设置占据状态
  // for (const auto& obstacle_pos : static_obstacle_positions_) {
  //   // 将世界坐标转换为栅格索引
  //   Eigen::Vector2i idx;
  //   occupancy_map_.Coordinate2Index(obstacle_pos, &idx);  // 移除bool返回值

  //   LOG(INFO) << "Processing obstacle at world position: " << obstacle_pos.transpose();
  //   LOG(INFO) << "Converted to grid index: " << idx.transpose();
    
  //   // 检查索引是否在有效范围内
  //   if (idx.x() >= 0 && idx.x() < dim[0] && 
  //       idx.y() >= 0 && idx.y() < dim[1]) {
  //     // 设置障碍物占据
  //     occupancy_map_.SetGridState(idx.x(), idx.y(), true);
      
  //     // 为了确保障碍物有足够大小，将周围的格子也设为占据
  //     constexpr int inflate_size = 2;  // 膨胀半径，可以根据需要调整
  //     int inflated_cells = 0;
  //     for (int dx = -inflate_size; dx <= inflate_size; ++dx) {
  //       for (int dy = -inflate_size; dy <= inflate_size; ++dy) {
  //         int nx = idx.x() + dx;
  //         int ny = idx.y() + dy;
  //         if (nx >= 0 && nx < dim[0] && ny >= 0 && ny < dim[1]) {
  //           occupancy_map_.SetGridState(nx, ny, true);
  //           inflated_cells++;
  //         }
  //       }
  //     }
      
  //     LOG(INFO) << "Successfully registered obstacle at grid: " << idx.transpose();
  //     LOG(INFO) << "Inflated to " << inflated_cells << " additional cells";
  //     registered_obstacles++;
  //   } else {
  //     LOG(WARNING) << "Obstacle out of map bounds: " << obstacle_pos.transpose()
  //                 << " (grid index: " << idx.transpose() << ")";
  //     out_of_bounds_obstacles++;
  //   }
  // }

  // 5. 障碍物检测和更新阶段的日志
  LOG(INFO) << "\n=== Scanning grid for occupied cells ===";
  int occupied_cells = 0;
  // 更新static_obstacle_positions_列表
  for (int x = 0; x < dim[0]; ++x) {
    for (int y = 0; y < dim[1]; ++y) {
      if (occupancy_map_.IsOccupied(x, y)) {
        Eigen::Vector2d coord;
        occupancy_map_.Index2Coordinate(Eigen::Vector2i(x, y), &coord);
        static_obstacle_positions_.push_back(coord);
        occupied_cells++;
        // LOG(INFO) << "Found occupied cell at grid (" << x << "," << y 
        //           << ") -> world: " << coord.transpose();
      }
    }
  }
  LOG(INFO) << "Total occupied cells found: " << occupied_cells;


  // 6. 距离场构建阶段的日志
  LOG(INFO) << "\n=== Building distance fields ===";
  DistanceMap distance_map, inv_distance_map;
  distance_map.ResizeFrom(occupancy_map_);
  inv_distance_map.ResizeFrom(occupancy_map_);

  LOG(INFO) << "Computing EDT for occupied space...";
  // 计算距离场
  EuclideanDistanceTransform(
      dim,
      [this](const int x, const int y) -> bool { 
        return this->occupancy_map_.IsOccupied(x, y); 
      },
      &distance_map);

  LOG(INFO) << "Computing EDT for free space...";
  EuclideanDistanceTransform(
      dim,
      [this](const int x, const int y) -> bool { 
        return !this->occupancy_map_.IsOccupied(x, y); 
      },
      &inv_distance_map);

  // 7. 最终SDF合并阶段的日志
  LOG(INFO) << "\n=== Merging distance fields into final SDF ===";
  // 合并得到最终的SDF
  auto& esdf_data = *esdf_.mutable_data();
  const auto& dis_map_data = distance_map.data();
  const auto& inv_dis_map_data = inv_distance_map.data();

  int modified_cells = 0;
  for (int x = 0; x < dim[0]; ++x) {
    for (int y = 0; y < dim[1]; ++y) {
      int address = occupancy_map_.Index2Address(x, y);
      esdf_data[address] = dis_map_data[address];
      if (inv_dis_map_data[address] > 0) {
        esdf_data[address] += (-inv_dis_map_data[address] + map_resolution_);
        modified_cells++;
      }
    }
  }

  LOG(INFO) << "Modified " << modified_cells << " cells in final SDF";

  // 8. 总结信息
  LOG(INFO) << "\n=== Final Statistics ==="
            // << "\nTotal obstacles registered: " << registered_obstacles
            << "\nTotal occupied cells: " << occupied_cells
            << "\nTotal cells modified in SDF: " << modified_cells;
  
  LOG(INFO) << "================== SDF Update End ==================\n";
}

void SignedDistanceField2D::UpdateVerticalSDF() {
  DistanceMap distance_map, inv_distance_map;
  distance_map.ResizeFrom(occupancy_map_);
  inv_distance_map.ResizeFrom(occupancy_map_);
  OccupancyMap& map = occupancy_map_;

  auto dim = occupancy_map_.cell_num();
  VerticalEuclideanDistanceTransform(
      dim,
      [&map](const int x, const int y) -> bool { return map.IsOccupied(x, y); },
      &distance_map);
  VerticalEuclideanDistanceTransform(
      dim,
      [&map](const int x, const int y) -> bool {
        return !map.IsOccupied(x, y);
      },
      &inv_distance_map);

  const auto& dis_map_data = distance_map.data();
  const auto& inv_dis_map_data = inv_distance_map.data();
  auto& esdf_data = *esdf_.mutable_data();

  omp_set_num_threads(4);
  {
#pragma omp parallel for
    for (int x = 0; x < dim[0]; ++x) {
      for (int y = 0; y < dim[1]; ++y) {
        int address = occupancy_map_.Index2Address(x, y);
        esdf_data[address] = dis_map_data[address];
        if (inv_dis_map_data[address] > 0) {
          esdf_data[address] += (-inv_dis_map_data[address] + map_resolution_);
        }
      }
    }
  }
}

double SignedDistanceField2D::GetSignedDistanceWithUncertainty(
    const Eigen::Vector2d& coord,
    const PerceptionUncertainty& uncertainty,
    Eigen::Vector2d* grad) const {
    
  // 获取基础SDF距离和梯度
  Eigen::Vector2d base_gradient;
  double base_distance = SignedDistance(coord, &base_gradient);
  
  if (grad) {
  // 确保梯度非空且在边界情况下仍然有效
    static constexpr double kMinGradientNorm = 1e-8;
    static constexpr double kEpsilon = 1e-6;
  if (base_gradient.norm() < kMinGradientNorm) {
    // 在边界情况下使用数值差分计算梯度
    const double eps = 1e-4;
    Eigen::Vector2d dx(eps, 0.0);
    Eigen::Vector2d dy(0.0, eps);
    base_gradient[0] = (SignedDistance(coord + dx) - 
                       SignedDistance(coord - dx)) / (2.0 * eps);
    base_gradient[1] = (SignedDistance(coord + dy) - 
                       SignedDistance(coord - dy)) / (2.0 * eps);
  }
  *grad = base_gradient.normalized();
  }

  // 如果不确定性信息无效,直接返回基础距离
  if (!uncertainty.IsValid()) {
    return base_distance;
  }

  // 计算距离场梯度方向上的不确定性
  double sigma = std::sqrt(base_gradient.transpose() * 
                          uncertainty.position_covariance * 
                          base_gradient);

  // 考虑几何尺寸的不确定性
  double size_uncertainty = std::sqrt(uncertainty.length_variance + 
                                    uncertainty.width_variance);

  // 使用3σ规则修正距离值
  double modified_distance = base_distance - 3.0 * sigma - size_uncertainty;

  return modified_distance;
}

bool SignedDistanceField2D::CheckSafetyWithUncertainty(
    const Eigen::Vector2d& coord,
    const PerceptionUncertainty& uncertainty,
    double safety_threshold,
    double* collision_prob) const {

  // 在获取距离之前添加日志
    // LOG(INFO) << "Query point: " << coord.transpose() 
    //           << " in map: " << occupancy_map_.IsInMap(coord);
    // LOG(INFO) << "Map origin: " << occupancy_map_.origin()[0] 
    //           << ", " << occupancy_map_.origin()[1];
    // LOG(INFO) << "Map size: " << occupancy_map_.cell_num()[0] 
    //           << " x " << occupancy_map_.cell_num()[1];
    // LOG(INFO) << "Resolution: " << map_resolution_;

  // 获取SDF距离和梯度
  Eigen::Vector2d gradient;
  double distance = SignedDistance(coord, &gradient);

  // 获取距离后立即添加日志
    // LOG(INFO) << "Computed distance: " << distance;
    // LOG(INFO) << "Gradient: " << gradient.transpose();

  // 计算梯度方向上的不确定性
  double sigma = std::sqrt(gradient.transpose() * 
                          uncertainty.position_covariance * 
                          gradient);

  // 考虑几何尺寸不确定性
  double total_uncertainty = std::sqrt(
      uncertainty.length_variance + 
      uncertainty.width_variance);
  
  // 使用误差函数计算碰撞概率
  // P(collision) = P(distance + N(0,sigma^2) < safety_threshold)
  double normalized_distance = (distance - safety_threshold) / 
                             (std::sqrt(2.0) * (sigma + total_uncertainty));
  double prob = 0.5 * (1.0 + std::erf(-normalized_distance));

  if (collision_prob) {
    *collision_prob = prob;
  }

  // 如果碰撞概率低于阈值则认为安全
  constexpr double kMaxAllowedCollisionProb = 0.2;  // 5%的碰撞概率阈值 0.05
  return prob < kMaxAllowedCollisionProb;
}


}  // namespace planning
