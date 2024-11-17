/**
 * Copyright (C) 2022, RAM-LAB, Hong Kong University of Science and Technology
 * This file is part of GPIR (https://github.com/jchengai/gpir).
 * If you find this repo helpful, please cite the respective publication as
 * listed on the above website.
 */

#include "gp_planner/initializer/gp_initializer.h"

#include "common/smoothing/osqp_spline1d_solver.h" //用于求解一维样条拟合问题的求解器，OsqpSpline1dSolver 类用于生成一维样条曲线，保证路径的平滑性。

namespace planning {

void GPInitializer::SetBoundary(
    std::vector<std::vector<std::pair<double, double>>> boundary) {
  // tmp test
  boundary_.clear();
  for (const auto b : boundary) {
    auto a = b.front();
    a.first += 2.5;
    a.second -= 2.5;
    // std::cout << a.first << ", " << a.second << std::endl;
    boundary_.emplace_back(a);
  }
}

// 函数用于生成初始路径，生成的路径将用于作为后续优化过程的起点。它采用样条曲线（spline）拟合的方法生成符合边界条件和参考线约束的初始轨迹。
// 根据起点 x0 和终点 xn，生成路径的初始猜测（即初始路径），并将其存入 result 中。
// 输入包括参考路径位置 s_refs，障碍物位置提示 obstacle_location_hint，路径的上下边界 lb 和 ub 等。
bool GPInitializer::GenerateInitialPath(
    const Eigen::Vector3d& x0, const Eigen::Vector3d& xn,
    const std::vector<double> s_refs,
    const std::vector<double>& obstacle_location_hint,
    const std::vector<double>& lb, std::vector<double>& ub,
    vector_Eigen3d* result) {
  double start = s_refs.front();
  double length = s_refs.back() - start; //start 和 length：从 s_refs 中获取起点和路径长度。
  std::vector<double> knots{start, start + length / 4.0, start + length / 2.0,
                            start + length * 3.0 / 4.0, start + length}; //将路径划分为多个节点（控制点），这些节点用于定义样条曲线。这里将路径分为五个部分（四个间隔），使路径的生成更加细致。

  common::OsqpSpline1dSolver spline1d_solver(knots, 5); //使用 OsqpSpline1dSolver 对象 spline1d_solver 来求解路径的样条曲线。knots 定义样条的节点，5 表示使用五次样条曲线来进行拟合，保证了路径的平滑性。

  std::vector<double> ref(s_refs.size(), 0); //ref：定义参考线，用于约束样条生成的路径不要偏离太多，初始值为 0。
 
  ////mutable_kernel 和 mutable_constraint：获取 spline1d_solver 的核函数和约束条件对象，用于对样条的行为施加各种约束。
  auto mutable_kernel = spline1d_solver.mutable_kernel();
  auto mutable_constraint = spline1d_solver.mutable_constraint(); 
  //核函数设置：
  mutable_kernel->AddReferenceLineKernelMatrix(s_refs, ref, 1);//添加二阶导数和三阶导数的约束，用于控制路径的平滑性。较高的数值表示更强的约束，使路径更平滑。
  mutable_kernel->AddSecondOrderDerivativeMatrix(200);
  mutable_kernel->AddThirdOrderDerivativeMatrix(1000); //添加二阶导数和三阶导数的约束，用于控制路径的平滑性。较高的数值表示更强的约束，使路径更平滑。
  //设置路径点的边界和约束
  //点约束——添加起点和终点的约束，使得样条曲线在起点和终点处满足位置、导数（速度）和二阶导数（加速度）的要求
  mutable_constraint->AddPointConstraint(start, x0(0));
  mutable_constraint->AddPointDerivativeConstraint(start, x0(1));
  mutable_constraint->AddPointSecondDerivativeConstraint(start, x0(2));
  mutable_constraint->AddPointConstraint(start + length, xn(0));
  mutable_constraint->AddPointDerivativeConstraint(start + length, xn(1));
  mutable_constraint->AddPointSecondDerivativeConstraint(start + length, xn(2));
  //平滑约束——为样条曲线添加二阶导数的平滑约束，确保路径曲率变化平滑。
  mutable_constraint->AddSecondDerivativeSmoothConstraint();
  
  //添加障碍物边界
  // 如果提供了障碍物位置提示，则调用 AddBoundary，根据障碍物位置和路径边界，设置相应的约束，确保生成的路径能够避开障碍物。
  if (!obstacle_location_hint.empty()) {
    mutable_constraint->AddBoundary(obstacle_location_hint, lb, ub);
  }
  
  //Solve()：调用求解器求解样条问题。
  if (!spline1d_solver.Solve()) {
    LOG(ERROR) << "solve failed";
    return false;
  }
  

  auto spline = spline1d_solver.spline(); //获取求解后的样条对象
  //遍历每个参考点 s_refs[i]，并计算该点的样条值（包括位置、一阶导数、二阶导数）。
  //使用 Eigen::Vector3d 来存储这些值，并将它们添加到结果向量 result 中。
  for (int i = 0; i < s_refs.size(); ++i) {
    result->emplace_back(
        Eigen::Vector3d(spline(s_refs[i]), spline.Derivative(s_refs[i]),
                        spline.SecondOrderDerivative(s_refs[i])));
  }
  return true;
}
}  // namespace planning
