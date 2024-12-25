// gp_planner/gp/uncertainty/scene_analyzer.cc
#include "gp_planner/gp/uncertainty/scene_analyzer.h"

namespace planning {

bool SceneAnalyzer::Init(const SceneAnalyzerConfig& config) {
  config_ = config;
  return true;
}

void SceneAnalyzer::Update(const ReferenceLine& reference_line,
                         const std::vector<Obstacle>& obstacles,
                         const common::State& ego_state) {
  reference_line_ = std::make_shared<ReferenceLine>(reference_line);
  ego_state_ = ego_state;
  
  // 清空缓存
  cached_features_.clear();
  
  // 沿参考线采样分析场景
  double step = config_.analysis_step;
  for (double s = 0; s < reference_line_->length(); s += step) {
    SceneFeatures features;
    
    // 分析各个维度的特征
    AnalyzeTrafficFeatures(s, obstacles, &features);
    AnalyzeRoadFeatures(s, reference_line, &features);
    AnalyzePerceptionQuality(s, &features);
    AnalyzeEnvironmentFeatures(s, &features);
    
    cached_features_.push_back(features);
  }
}

SceneFeatures SceneAnalyzer::GetSceneFeatures(const double s) const {
  // 从缓存中查找最近的特征
  int index = static_cast<int>(s / config_.analysis_step);
  index = std::min(static_cast<int>(cached_features_.size()) - 1,
                  std::max(0, index));
  return cached_features_[index];
}

void SceneAnalyzer::AnalyzeTrafficFeatures(
    const double s,
    const std::vector<Obstacle>& obstacles,
    SceneFeatures* features) {
  constexpr double kSearchRadius = 50.0;  // 搜索半径
  int count = 0;
  double min_dist = std::numeric_limits<double>::max();
  
  // 统计周围车辆
  for (const auto& obstacle : obstacles) {
    auto proj = reference_line_->GetProjection(obstacle.position());
    if (std::abs(proj.s - s) > kSearchRadius) continue;
    
    count++;
    min_dist = std::min(min_dist, 
        (obstacle.position() - ego_state_.position).norm());
  }
  
  features->vehicle_count = count;
  features->min_distance = min_dist;
  features->traffic_density = count / (2 * kSearchRadius);
  
  // 计算交互复杂度
  features->interaction_complexity = 
      features->traffic_density * (1.0 + 1.0 / features->min_distance);
}

void SceneAnalyzer::AnalyzeRoadFeatures(
    const double s,
    const ReferenceLine& reference_line,
    SceneFeatures* features) {
  // 获取道路几何特征
  features->curvature = reference_line.GetCurvature(s);
  features->lane_width = reference_line.GetLaneWidth(s);
  
  // 判断特殊区域
  features->is_intersection = reference_line.IsInIntersection(s);
  features->is_merge_area = reference_line.IsInMergeArea(s);
  features->is_narrow = features->lane_width < config_.narrow_lane_threshold;
}

void SceneAnalyzer::AnalyzePerceptionQuality(
    const double s,
    SceneFeatures* features) {
  // 这里应该根据实际的传感器状态评估感知质量
  // 目前使用简化的模型
  features->gps_quality = GetGPSQuality(s);
  features->visual_quality = GetVisualQuality(s);
  features->lidar_quality = GetLidarQuality(s);
  
  // 计算遮挡率
  features->occlusion_ratio = ComputeOcclusionRatio(s);
}

void SceneAnalyzer::AnalyzeEnvironmentFeatures(
    const double s,
    SceneFeatures* features) {
  // 从环境感知模块获取天气状况
  features->is_rain = IsRaining();
  features->is_fog = IsFoggy();
  features->is_night = IsNightTime();
  
  // 计算能见度
  features->visibility = ComputeVisibility(s);
}

} // namespace planning