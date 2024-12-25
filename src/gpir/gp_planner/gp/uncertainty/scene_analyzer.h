#pragma once

#include <memory>
#include <vector>
#include "common/math/vec2d.h"
#include "planning_core/navigation/reference_line.h"
#include "planning_core/planning_common/obstacle.h"

namespace planning {

/**
 * 场景特征结构体，包含了场景的各种特征信息
 * 这些特征将用于评估不同类型的不确定性
 */
struct SceneFeatures {
  // 交通场景特征
  double traffic_density{0.0};         // 交通密度
  double interaction_complexity{0.0};   // 交互复杂度
  int vehicle_count{0};                // 周围车辆数量
  double min_distance{0.0};            // 最近车辆距离
  double relative_speed{0.0};          // 相对速度

  // 道路场景特征
  double curvature{0.0};              // 道路曲率
  double slope{0.0};                  // 道路坡度
  double lane_width{0.0};             // 车道宽度
  bool is_intersection{false};         // 是否处于路口
  bool is_merge_area{false};          // 是否处于汇流区
  bool is_narrow{false};              // 是否为窄路段
  
  // 感知质量特征
  double occlusion_ratio{0.0};        // 视野遮挡率
  double gps_quality{1.0};            // GPS信号质量
  double visual_quality{1.0};         // 视觉特征质量
  double lidar_quality{1.0};          // 激光雷达质量
  
  // 环境特征
  bool is_rain{false};                // 是否下雨
  bool is_fog{false};                 // 是否有雾
  bool is_night{false};               // 是否夜间
  double visibility{1.0};             // 能见度
};

/**
 * 场景分析器类
 * 负责分析场景特征并提供给不确定性评估模块
 */
class SceneAnalyzer {
 public:
  SceneAnalyzer() = default;
  ~SceneAnalyzer() = default;

  /**
   * 初始化场景分析器
   * @param config 配置参数
   */
  bool Init(const SceneAnalyzerConfig& config);

  /**
   * 更新并分析场景
   * @param reference_line 参考线
   * @param obstacles 障碍物列表
   * @param ego_state 自车状态
   */
  void Update(const ReferenceLine& reference_line,
             const std::vector<Obstacle>& obstacles,
             const common::State& ego_state);

  /**
   * 获取指定位置的场景特征
   * @param s frenet坐标s
   * @return 场景特征
   */
  SceneFeatures GetSceneFeatures(const double s) const;

 private:
  /**
   * 分析交通场景特征
   */
  void AnalyzeTrafficFeatures(const double s,
                            const std::vector<Obstacle>& obstacles,
                            SceneFeatures* features);

  /**
   * 分析道路场景特征
   */
  void AnalyzeRoadFeatures(const double s,
                         const ReferenceLine& reference_line,
                         SceneFeatures* features);

  /**
   * 分析感知质量
   */
  void AnalyzePerceptionQuality(const double s,
                              SceneFeatures* features);

  /**
   * 分析环境特征
   */
  void AnalyzeEnvironmentFeatures(const double s,
                                SceneFeatures* features);

 private:
  SceneAnalyzerConfig config_;
  std::shared_ptr<ReferenceLine> reference_line_;
  common::State ego_state_;
  std::vector<SceneFeatures> cached_features_;  // 缓存的场景特征
};

} // namespace planning