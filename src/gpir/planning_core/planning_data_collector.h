/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#pragma once

#include <vector>
#include <string>
#include <fstream>
#include <ctime>
#include <iomanip>
#include <sys/stat.h>
#include "common/base/state.h"
#include "gp_planner/gp/utils/gp_path.h"

namespace planning {

/**
 * @brief 存储单次路径规划的数据结构
 */
struct PathPlanningData {
    // 存储路径点的x坐标
    std::vector<double> x_coords;
    // 存储路径点的y坐标
    std::vector<double> y_coords;
    // 存储路径上每个点的曲率
    std::vector<double> curvatures;
    // 存储路径上点的弧长坐标
    std::vector<double> s_coords;
    // 当前路径的迭代次数
    int iteration_count;
    
    /**
     * @brief 清空所有数据
     */
    void Clear() {
        x_coords.clear();
        y_coords.clear();
        curvatures.clear();
        s_coords.clear();
    }
};

/**
 * @brief 路径规划数据收集器类，用于收集和保存规划数据
 */
class PlanningDataCollector {
private:
    // 定义数据存储的基础目录
    static const std::string base_directory_;
    // 当前实验的目录
    static std::string current_experiment_dir_;
    // 实验计数器
    static int experiment_count_;

    /**
     * @brief 创建目录的辅助函数
     * @param path 要创建的目录路径
     * @return 是否创建成功
     */
    static bool CreateDirectory(const std::string& path) {
        struct stat st = {0};
        if (stat(path.c_str(), &st) == -1) {
            return mkdir(path.c_str(), 0777) == 0;
        }
        return true;
    }
    
    /**
     * @brief 获取格式化的时间戳
     * @return 格式化的时间戳字符串 (YYYYMMDD_HHMMSS)
     */
    static std::string GetTimeStamp() {
        auto now = std::time(nullptr);
        auto ltm = std::localtime(&now);
        std::ostringstream oss;
        oss << std::put_time(ltm, "%Y%m%d_%H%M%S");
        return oss.str();
    }

public:
    /**
     * @brief 初始化数据收集器
     * @return 是否初始化成功
     */
    static bool Initialize() {
        // 确保基础目录存在
        if (!CreateDirectory(base_directory_)) {
            LOG(ERROR) << "Failed to create base directory: " << base_directory_;
            return false;
        }
        
        // 创建带时间戳的实验目录
        current_experiment_dir_ = base_directory_ + "/experiment_" + GetTimeStamp();
        if (!CreateDirectory(current_experiment_dir_)) {
            LOG(ERROR) << "Failed to create experiment directory: " << current_experiment_dir_;
            return false;
        }
        
        experiment_count_ = 0;
        return true;
    }

    /**
     * @brief 收集路径规划数据
     * @param path GPPath对象，包含路径信息
     * @param sample_interval 采样间隔
     * @param data 用于存储收集到的数据
     */
    static void CollectPathData(const GPPath& path, 
                              const double sample_interval,
                              PathPlanningData* data) {
        data->Clear();
        
        // 沿路径采样点并收集数据
        for(double s = path.start_s(); s <= path.MaximumArcLength(); 
            s += sample_interval) {
            common::State state;
            path.GetState(s, &state);
            
            // 记录位置信息
            data->x_coords.push_back(state.position.x());
            data->y_coords.push_back(state.position.y());
            
            // 记录曲率信息
            data->curvatures.push_back(path.GetCurvature(s));
            
            // 记录路径坐标
            data->s_coords.push_back(s);
        }
    }
    
      /**
     * @brief 保存规划数据到CSV文件
     * @param all_data 包含所有迭代数据的vector
     */
    static void SaveToCSV(const std::vector<PathPlanningData>& all_data) {
        // 在当前实验目录下生成文件名
        std::string filename = current_experiment_dir_ + "/path_planning_data_" + 
                             std::to_string(experiment_count_++) + ".csv";
                             
        std::ofstream file(filename);
        if (!file.is_open()) {
            LOG(ERROR) << "Failed to open file for writing: " << filename;
            return;
        }

        // 写入CSV头
        file << "iteration,s,x,y,curvature\n";
        
        // 写入数据
        for(const auto& path_data : all_data) {
            for(size_t i = 0; i < path_data.s_coords.size(); ++i) {
                file << path_data.iteration_count << ","
                     << path_data.s_coords[i] << ","
                     << path_data.x_coords[i] << ","
                     << path_data.y_coords[i] << ","
                     << path_data.curvatures[i] << "\n";
            }
        }
        file.close();
        
        LOG(INFO) << "Saved planning data to: " << filename;
    }
};

// 定义静态成员变量
const std::string PlanningDataCollector::base_directory_ = 
    "/home/erdong2004/GPIR_planner_ws/src/gpir/data/planning_experiments";
std::string PlanningDataCollector::current_experiment_dir_;
int PlanningDataCollector::experiment_count_ = 0;

} // namespace planning