/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 **/
#include "modules/planning/tasks/optimizers/path_time_heuristic/dp_st_cost.h"

#include <algorithm>
#include <limits>

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/speed/st_point.h"
#include "modules/planning/tasks/utils/st_gap_estimator.h"

namespace apollo {
namespace planning {
namespace {
constexpr double kInf = std::numeric_limits<double>::infinity();
}

DpStCost::DpStCost(const DpStSpeedOptimizerConfig& config, const double total_t,
                   const double total_s,
                   const std::vector<const Obstacle*>& obstacles,
                   const STDrivableBoundary& st_drivable_boundary,
                   const common::TrajectoryPoint& init_point)
    : config_(config),
      obstacles_(obstacles),
      st_drivable_boundary_(st_drivable_boundary),
      init_point_(init_point),
      unit_t_(config.unit_t()),
      total_s_(total_s) {
  int index = 0;
  for (const auto& obstacle : obstacles)
   {
    boundary_map_[obstacle->path_st_boundary().id()] = index++;
  }

  AddToKeepClearRange(obstacles);

  const auto dimension_t =
      static_cast<uint32_t>(std::ceil(total_t / static_cast<double>(unit_t_))) +
      1;
  boundary_cost_.resize(obstacles_.size());
  for (auto& vec : boundary_cost_) {
    vec.resize(dimension_t, std::make_pair(-1.0, -1.0));
  }
  accel_cost_.fill(-1.0);
  jerk_cost_.fill(-1.0);
}

void DpStCost::AddToKeepClearRange(
    const std::vector<const Obstacle*>& obstacles) {
  for (const auto& obstacle : obstacles) {
    if (obstacle->path_st_boundary().IsEmpty()) {
      continue;
    }
    if (obstacle->path_st_boundary().boundary_type() !=
        STBoundary::BoundaryType::KEEP_CLEAR) {
      continue;
    }

    double start_s = obstacle->path_st_boundary().min_s();
    double end_s = obstacle->path_st_boundary().max_s();
    keep_clear_range_.emplace_back(start_s, end_s);
  }
  SortAndMergeRange(&keep_clear_range_);
}

void DpStCost::SortAndMergeRange(
    std::vector<std::pair<double, double>>* keep_clear_range) {
  if (!keep_clear_range || keep_clear_range->empty()) {
    return;
  }
  std::sort(keep_clear_range->begin(), keep_clear_range->end());
  size_t i = 0;
  size_t j = i + 1;
  while (j < keep_clear_range->size()) {
    if (keep_clear_range->at(i).second < keep_clear_range->at(j).first) {
      ++i;
      ++j;
    } else {
      keep_clear_range->at(i).second = std::max(keep_clear_range->at(i).second,
                                                keep_clear_range->at(j).second);
      ++j;
    }
  }
  keep_clear_range->resize(i + 1);
}

bool DpStCost::InKeepClearRange(double s) const {
  for (const auto& p : keep_clear_range_) 
  {
    if (p.first <= s && p.second >= s) {
      return true;
    }
  }
  return false;
}

double DpStCost::GetObstacleCost(const StGraphPoint& st_graph_point) 
{
  // s/t 点
  const double s = st_graph_point.point().s();
  const double t = st_graph_point.point().t();

  double cost = 0.0;

  if (FLAGS_use_st_drivable_boundary) 
  {
    // TODO(Jiancheng): move to configs
    static constexpr double boundary_resolution = 0.1;

    int index = static_cast<int>(t / boundary_resolution);
    // ST可行使边界的下边界
    const double lower_bound =
        st_drivable_boundary_.st_boundary(index).s_lower();
    // ST可行使边界的上边界
    const double upper_bound =
        st_drivable_boundary_.st_boundary(index).s_upper();
    // ST采样点需要在边界内
    if (s > upper_bound || s < lower_bound) {
      return kInf;
    }
  }
  //1 遍历障碍物
  for (const auto* obstacle : obstacles_) 
  {
    // Not applying obstacle approaching cost to virtual obstacle like created
    // stop fences
    //1.1 if目标是虚拟障碍物，下个循环
    if (obstacle->IsVirtual()) {
      continue;
    }

    // Stop obstacles are assumed to have a safety margin when mapping them out,
    // so repelling force in dp st is not needed as it is designed to have adc
    // stop right at the stop distance we design in prior mapping process
    //1.2 if目标足够远，下个循环
    if (obstacle->LongitudinalDecision().has_stop()) {
      continue;
    }
    // 障碍物的边界
    auto boundary = obstacle->path_st_boundary();
    //1.3  障碍物下边界> horizon, 或（障碍物边界最小 时间> t  or 障碍物边界最大 时间< t 即没有碰撞风险） ，下个循环
    if (boundary.min_s() > FLAGS_speed_lon_decision_horizon) {
      continue;
    }

    if (t < boundary.min_t() || t > boundary.max_t()) {
      continue;
    }

    // 1.4 若采样点在边界内，返回 无穷大
    if (boundary.IsPointInBoundary(st_graph_point.point())) 
    {
      return kInf;
    }

    double s_upper = 0.0;
    double s_lower = 0.0;

    // 目标边界id
    int boundary_index = boundary_map_[boundary.id()];
    //目标上边界<0 ,即目标在自车后？？？
    if (boundary_cost_[boundary_index][st_graph_point.index_t()].first < 0.0) 
    {
      boundary.GetBoundarySRange(t, &s_upper, &s_lower);
      boundary_cost_[boundary_index][st_graph_point.index_t()] =
          std::make_pair(s_upper, s_lower);
    } 
    else //目标上边界>=0 ,即目标在自车前， 读取目标上下边界
    {
      s_upper = boundary_cost_[boundary_index][st_graph_point.index_t()].first;
      s_lower = boundary_cost_[boundary_index][st_graph_point.index_t()].second;
    }


    //1.5  采样点s< 下边界s_lower
    if (s < s_lower) 
    {
      // 安全跟车距离参数
      const double follow_distance_s = config_.safe_distance();
      
      //if    采样点+ 安全跟车距离 < s_lower, 不考虑这个目标，进行下个循环
      //else  障碍物代价= 障碍物代价系数* 默认代价障碍物代价系数* 距离差平方
      if (s + follow_distance_s < s_lower) 
      {
        continue;

      } 
      else {
        auto s_diff = follow_distance_s - s_lower + s; //距离差
        cost += config_.obstacle_weight() * config_.default_obstacle_cost() *
                s_diff * s_diff;
      }
    } 
    // 1.6 采样点s>=下边界s_lower
      // if （s > s_upper），安全超车距离=overtake_distance_s=10m/20m
      //  if (s > s_upper + overtake_distance_s) ， 不考虑这个目标，进行下个循环 
      //  if （s > s_upper）&&(s<= s_upper + overtake_distance_s) ,
              // --->障碍物代价= 障碍物代价系数* 默认代价障碍物代价系数* 距离差平方
    else if (s > s_upper) 
    {
      const double overtake_distance_s =
          StGapEstimator::EstimateSafeOvertakingGap();

      if (s > s_upper + overtake_distance_s) 
      {  // or calculated from velocity
        continue;
      } 
      else {
        auto s_diff = overtake_distance_s + s_upper - s;
        cost += config_.obstacle_weight() * config_.default_obstacle_cost() *
                s_diff * s_diff;
      }
    }
  }
  // 返回 障碍物代价*1s
  return cost * unit_t_;
}

double DpStCost::GetSpatialPotentialCost(const StGraphPoint& point) {
  return (total_s_ - point.point().s()) * config_.spatial_potential_penalty();
}

double DpStCost::GetReferenceCost(const STPoint& point,
                                  const STPoint& reference_point) const 
                                  {
  return config_.reference_weight() * (point.s() - reference_point.s()) *
         (point.s() - reference_point.s()) * unit_t_;
}


  //1） if (速度很小&& 当前采样点在在禁停区内)，则 速度代价=速度代价+ keep_clear低速代价系数*默认速度代价系数*单位时间1s
        // keep_clear低速代价系数=10； 默认速度代价系数=1000
  //2） if(速度差>0),速度代价= 速度代价+ 超速代价系数*默认速度代价系数*（速度差百分比^2）*单位时间1s
        // 超速代价系数=1000；  
  //3） if(速度差<0),速度代价= 速度代价- 低速代价系数*默认速度代价系数*（速度差百分比）*单位时间1s
        // 低速代价系数=10   
  // 4)if参考速度使能，  目标期望速度差= 边的速度-巡航速度
      // 速度代价= 速度代价+参考速度代价系数*默认速度代价系数*（速度差绝对值）*单位时间1s        

// 结论1：在禁停区需要有一定的cost。
// 结论2：代码鼓励无人车在该路径段内行驶速度低于最高限速，可以看到该情况下cost小于0，有奖励。反之超速了，惩罚与速度平法成正比。
double DpStCost::GetSpeedCost(const STPoint& first, const STPoint& second,
                              const double speed_limit,
                              const double cruise_speed) const 
{
  double cost = 0.0;
  // 边的速度= (s2-s1)/t
  const double speed = (second.s() - first.s()) / unit_t_;
  if (speed < 0) 
  {
    return kInf;
  }
  // 最大停车速度=0.2
  const double max_adc_stop_speed = common::VehicleConfigHelper::Instance()
                                        ->GetConfig()
                                        .vehicle_param()
                                        .max_abs_speed_when_stopped();
  //1） if (速度很小&& 当前采样点在在禁停区内)，则 速度代价=速度代价+ keep_clear低速代价系数*默认速度代价系数*单位时间1s
        // keep_clear低速代价系数=10； 默认速度代价系数=1000
  if (speed < max_adc_stop_speed && InKeepClearRange(second.s())) 
  {
    // first.s in range
    cost += config_.keep_clear_low_speed_penalty() * unit_t_ *
            config_.default_speed_cost();
  }
  // 边的速度与限速的差值百分比
  double det_speed = (speed - speed_limit) / speed_limit;
  //2） if(速度差>0),速度代价= 速度代价+ 超速代价系数*默认速度代价系数*（速度差百分比^2）*单位时间1s
        // 超速代价系数=1000；  
  if (det_speed > 0)
   {
    cost += config_.exceed_speed_penalty() * config_.default_speed_cost() *
            (det_speed * det_speed) * unit_t_;
  } 
    //3） if(速度差<0),速度代价= 速度代价- 低速代价系数*默认速度代价系数*（速度差百分比）*单位时间1s
        // 低速代价系数=10
  else if (det_speed < 0) {
    cost += config_.low_speed_penalty() * config_.default_speed_cost() *
            -det_speed * unit_t_;
  }
  // 4)if参考速度使能，  目标期望速度差= 边的速度-巡航速度
      // 速度代价= 速度代价+参考速度代价系数*默认速度代价系数*（速度差绝对值）*单位时间1s
  if (FLAGS_enable_dp_reference_speed) {
    double diff_speed = speed - cruise_speed;
    cost += config_.reference_speed_penalty() * config_.default_speed_cost() *
            fabs(diff_speed) * unit_t_;
  }

  return cost;
}
//  加速度为正情况下cost=加速度惩罚系数*实际加速度平方
//  加速度为负情况下cost=减速度惩罚系数*实际加速度平方
// 加速总代价cost= cost+加速度平方*减速惩罚系数平方/（1+e^(加速度与最大减速度的差))
                    //+加速度平方*加速惩罚系数平方/（1+e^(-加速度与最大加速度的差)) 
// 返回==加速度总代价cost*单位时间1s

double DpStCost::GetAccelCost(const double accel) 
{
  double cost = 0.0;
  static constexpr double kEpsilon = 0.1;
  static constexpr size_t kShift = 100;
  // 基于加速度的accel_key索引  ，索引范围<size_t>[59.5 120.5]= [59 120], 最大值小于200
  // =2，accel_key=120.5
  // =-4，accel_key=59.5
  const size_t accel_key = static_cast<size_t>(accel / kEpsilon + 0.5 + kShift);

  DCHECK_LT(accel_key, accel_cost_.size());

  // 基于加速度的accel_key索引  ，索引范围<size_t>[59.5 120.5]= [59 120], 最大值小于200
  if (accel_key >= accel_cost_.size())
   {
    return kInf;
  }

  //if 加速度代价为负（初始化为-1，表示未进行计算）
  if (accel_cost_.at(accel_key) < 0.0) 
  {
    const double accel_sq = accel * accel; //实际加速度平方
    double max_acc = config_.max_acceleration();//2
    double max_dec = config_.max_deceleration();//-4
    double accel_penalty = config_.accel_penalty(); //加速度惩罚系数=1
    double decel_penalty = config_.decel_penalty();//减速度惩罚系数=1

    if (accel > 0.0) 
    {
      cost = accel_penalty * accel_sq; //  加速度为正情况下cost=加速度惩罚系数*实际加速度平方
    } 
    else 
    {
      cost = decel_penalty * accel_sq;//  加速度为负情况下cost=减速度惩罚系数*实际加速度平方
    }
    // 加速总代价= cost+加速度平方*减速惩罚系数平方/（1+e^(加速度与最大减速度的差))
                    //+加速度平方*加速惩罚系数平方/（1+e^(-加速度与最大加速度的差)) 

    cost += accel_sq * decel_penalty * decel_penalty /
                (1 + std::exp(1.0 * (accel - max_dec))) +
            accel_sq * accel_penalty * accel_penalty /
                (1 + std::exp(-1.0 * (accel - max_acc)));
    accel_cost_.at(accel_key) = cost;
  } 
  //else 加速度为正
  else {
    cost = accel_cost_.at(accel_key);
  }

  // 返回==加速度总代价*单位时间1s
  return cost * unit_t_;
}

double DpStCost::GetAccelCostByThreePoints(const STPoint& first,
                                           const STPoint& second,
                                           const STPoint& third) 
  {
    //3点加速度= (s1+s3-2*s2)/t^2
  double accel = (first.s() + third.s() - 2 * second.s()) / (unit_t_ * unit_t_);
  return GetAccelCost(accel);
}

double DpStCost::GetAccelCostByTwoPoints(const double pre_speed,
                                         const STPoint& pre_point,
                                         const STPoint& curr_point) 
{
  //GetAccelCostByThreePoints函数和GetAccelCostByTwoPoints函数本质是一样的， 区别在于计算t时刻的加速度，
  //GetAccelCostByThreePoints利用3个节点的累积距离s1，s2，s3来计算
  //GetAccelCostByTwoPoints： 使用两个节点和初始速度来计算加速度：
  // 两点加速度cost用在从第0点到第1点的转移过程，三点求加速度用在其他点转移过程：


  // 边的速度
  double current_speed = (curr_point.s() - pre_point.s()) / unit_t_;
  // 2点加速度=(v2-v1)/t
  double accel = (current_speed - pre_speed) / unit_t_;

  return GetAccelCost(accel);
}

//jerk>0，jerk代价= 正的jerk系数*jerk平方*单位时间； positive_jerk_coeff=1
//jerk<=0，jerk代价= 负的jerk系数*jerk平方*单位时间 ；negative_jerk_coeff=1
double DpStCost::JerkCost(const double jerk) 
{
  double cost = 0.0;
  static constexpr double kEpsilon = 0.1;
  static constexpr size_t kShift = 200;
  const size_t jerk_key = static_cast<size_t>(jerk / kEpsilon + 0.5 + kShift);

  if (jerk_key >= jerk_cost_.size()) {
    return kInf;
  }

  if (jerk_cost_.at(jerk_key) < 0.0) 
  {
    double jerk_sq = jerk * jerk; //jerk平方

    if (jerk > 0) //jerk>0，jerk代价= 正的jerk系数*jerk平方*单位时间； positive_jerk_coeff=1
    {
      cost = config_.positive_jerk_coeff() * jerk_sq * unit_t_;
    } 
    else //jerk<=0，jerk代价= 负的jerk系数*jerk平方*单位时间 ；negative_jerk_coeff=1
    {
      cost = config_.negative_jerk_coeff() * jerk_sq * unit_t_;
    }
    //
    jerk_cost_.at(jerk_key) = cost;
  } 
  else {
    cost = jerk_cost_.at(jerk_key);
  }

  // TODO(All): normalize to unit_t_
  return cost;
}

double DpStCost::GetJerkCostByFourPoints(const STPoint& first,
                                         const STPoint& second,
                                         const STPoint& third,
                                         const STPoint& fourth) {
  //  4点Jerk= (s4-3*s3+3*s2-s1)/t^3  
  double jerk = (fourth.s() - 3 * third.s() + 3 * second.s() - first.s()) /
                (unit_t_ * unit_t_ * unit_t_);
  return JerkCost(jerk);
}

double DpStCost::GetJerkCostByTwoPoints(const double pre_speed,
                                        const double pre_acc,
                                        const STPoint& pre_point,
                                        const STPoint& curr_point) 
{
  //边的速度
  const double curr_speed = (curr_point.s() - pre_point.s()) / unit_t_;
  //边的加速度
  const double curr_accel = (curr_speed - pre_speed) / unit_t_;
  //2点Jerk=（a2-a1）/t
  const double jerk = (curr_accel - pre_acc) / unit_t_;
  return JerkCost(jerk);
}

double DpStCost::GetJerkCostByThreePoints(const double first_speed,
                                          const STPoint& first,
                                          const STPoint& second,
                                          const STPoint& third) 
                                          {
  // 3点Jerk= (a3-a2)/t           
         //前一个边速度v2= (s2-s1)/t  
         //前一个边加速度a2= (v2-v1)/t
        // 当前速度v3= (s3-s2)/t
        // 当前加速度a3= (v3-v2)/t
        // 3点Jerk=  (a3-a2)/t
  const double pre_speed = (second.s() - first.s()) / unit_t_;
  const double pre_acc = (pre_speed - first_speed) / unit_t_;
  const double curr_speed = (third.s() - second.s()) / unit_t_;

  const double curr_acc = (curr_speed - pre_speed) / unit_t_;
  const double jerk = (curr_acc - pre_acc) / unit_t_;
  return JerkCost(jerk);
}

}  // namespace planning
}  // namespace apollo
