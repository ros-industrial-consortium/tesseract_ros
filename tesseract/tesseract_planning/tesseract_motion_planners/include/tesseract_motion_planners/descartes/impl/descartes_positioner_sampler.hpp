/**
 * @file descartes_positioner_sampler.hpp
 * @brief Tesseract Descartes Robot + External Positioner Sampler Implementation
 *
 * @author Levi Armstrong
 * @date April 18, 2018
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef TESSERACT_MOTION_PLANNERS_IMPL_DESCARTES_POSITIONER_SAMPLER_HPP
#define TESSERACT_MOTION_PLANNERS_IMPL_DESCARTES_POSITIONER_SAMPLER_HPP

#include <tesseract/tesseract.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <descartes_light/utils.h>
#include <console_bridge/console.h>
#include <Eigen/Geometry>
#include <vector>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/descartes/descartes_positioner_sampler.h>

namespace tesseract_motion_planners
{
template <typename FloatType>
DescartesPositionerSampler<FloatType>::DescartesPositionerSampler(
    const Eigen::Isometry3d tool_pose,
    const tesseract_motion_planners::ToolPoseSamplerFn tool_pose_sampler,
    const tesseract_kinematics::ForwardKinematics::ConstPtr positioner_kinematics,
    const tesseract_kinematics::InverseKinematics::ConstPtr robot_kinematics,
    const typename descartes_light::CollisionInterface<FloatType>::Ptr collision,
    const tesseract_environment::EnvState::ConstPtr current_state,
    const Eigen::VectorXd positioner_sample_resolution,
    const Eigen::Isometry3d robot_tcp,
    const double robot_reach,
    const bool allow_collision,
    const DescartesIsValidFn<FloatType> is_valid)
  : tool_pose_(std::move(tool_pose))
  , tool_pose_sampler_(tool_pose_sampler)
  , positioner_kinematics_(std::move(positioner_kinematics))
  , robot_kinematics_(std::move(robot_kinematics))
  , collision_(std::move(collision))
  , world_to_positioner_base_(current_state->transforms.at(positioner_kinematics_->getBaseLinkName()))
  , world_to_robot_base_(current_state->transforms.at(robot_kinematics_->getBaseLinkName()))
  , positioner_limits_(positioner_kinematics_->getLimits())
  , positioner_sample_resolution_(std::move(positioner_sample_resolution))
  , robot_tcp_(robot_tcp)
  , robot_reach_(robot_reach)
  , allow_collision_(allow_collision)
  , dof_(positioner_kinematics_->numJoints() + robot_kinematics_->numJoints())
  , ik_seed_(Eigen::VectorXd::Zero(dof_))
  , is_valid_(is_valid)
{
}

template <typename FloatType>
bool DescartesPositionerSampler<FloatType>::sample(std::vector<FloatType>& solution_set)
{
  double distance = std::numeric_limits<double>::min();
  int num_joints = static_cast<int>(positioner_kinematics_->numJoints());
  std::vector<Eigen::VectorXd> dof_range;
  dof_range.reserve(static_cast<std::size_t>(num_joints));
  for (int dof = 0; dof < num_joints; ++dof)
  {
    int cnt = std::ceil(std::abs(positioner_limits_(dof, 1) - positioner_limits_(dof, 0)) /
                        positioner_sample_resolution_(dof));
    dof_range.push_back(Eigen::VectorXd::LinSpaced(cnt, positioner_limits_(dof, 0), positioner_limits_(dof, 1)));
  }

  tesseract_common::VectorIsometry3d tool_poses = tool_pose_sampler_(tool_pose_);
  for (std::size_t i = 0; i < tool_poses.size(); ++i)
  {
    // Tool pose subtract robot tcp
    Eigen::Isometry3d tool_pose = tool_poses[i] * robot_tcp_.inverse();
    Eigen::Matrix<FloatType, Eigen::Dynamic, 1> positioner_pose(num_joints);
    nested_ik(0, dof_range, tool_pose, positioner_pose, solution_set, false, distance);
  }

  if (solution_set.empty() && allow_collision_)
    getBestSolution(solution_set, tool_poses, dof_range);

  return !solution_set.empty();
}

template <typename FloatType>
bool DescartesPositionerSampler<FloatType>::isCollisionFree(const FloatType* vertex)
{
  if (collision_ == nullptr)
    return true;
  else
    return collision_->validate(vertex, dof_);
}

template <typename FloatType>
void DescartesPositionerSampler<FloatType>::nested_ik(
    const int loop_level,
    const std::vector<Eigen::VectorXd>& dof_range,
    const Eigen::Isometry3d& p,
    Eigen::Ref<Eigen::Matrix<FloatType, Eigen::Dynamic, 1>> sample_pose,
    std::vector<FloatType>& solution_set,
    const bool get_best_solution,
    double& distance)
{
  if (loop_level >= positioner_kinematics_->numJoints())
  {
    ikAt(p, sample_pose, solution_set, get_best_solution, distance);
    return;
  }

  for (long i = 0; i < static_cast<long>(dof_range[static_cast<std::size_t>(loop_level)].size()); ++i)
  {
    sample_pose(loop_level) = dof_range[static_cast<std::size_t>(loop_level)][i];
    nested_ik(loop_level + 1, dof_range, p, sample_pose, solution_set, get_best_solution, distance);
  }
}

template <typename FloatType>
bool DescartesPositionerSampler<FloatType>::ikAt(
    const Eigen::Isometry3d& p,
    const Eigen::Ref<const Eigen::Matrix<FloatType, Eigen::Dynamic, 1>>& positioner_pose,
    std::vector<FloatType>& solution_set,
    const bool get_best_solution,
    double& distance)
{
  Eigen::Isometry3d positioner_tf;
  if (!positioner_kinematics_->calcFwdKin(positioner_tf, positioner_pose.template cast<double>()))
    return false;

  Eigen::Isometry3d robot_tool_pose = world_to_robot_base_.inverse() * (world_to_positioner_base_ * positioner_tf * p);
  if (robot_tool_pose.translation().norm() > robot_reach_)
    return false;

  Eigen::VectorXd robot_solution_set;
  int robot_dof = robot_kinematics_->numJoints();
  if (!robot_kinematics_->calcInvKin(robot_solution_set, robot_tool_pose, ik_seed_))
    return false;

  long num_sols = robot_solution_set.size() / robot_dof;
  for (long i = 0; i < num_sols; i++)
  {
    double* sol = robot_solution_set.data() + robot_dof * i;

    std::vector<FloatType> full_sol;
    full_sol.insert(end(full_sol), positioner_pose.data(), positioner_pose.data() + positioner_pose.size());
    full_sol.insert(end(full_sol), std::make_move_iterator(sol), std::make_move_iterator(sol + robot_dof));

    if (!is_valid_(Eigen::Map<Eigen::Matrix<FloatType, Eigen::Dynamic, 1>>(full_sol.data(), full_sol.size())))
      continue;

    if (!get_best_solution)
    {
      if (isCollisionFree(full_sol.data()))
        solution_set.insert(
            end(solution_set), std::make_move_iterator(full_sol.begin()), std::make_move_iterator(full_sol.end()));
    }
    else
    {
      double cur_distance = collision_->distance(full_sol.data(), full_sol.size());
      if (cur_distance > distance)
      {
        distance = cur_distance;
        solution_set.insert(
            begin(solution_set), std::make_move_iterator(full_sol.begin()), std::make_move_iterator(full_sol.end()));
      }
    }
  }

  return !solution_set.empty();
}

template <typename FloatType>
bool DescartesPositionerSampler<FloatType>::getBestSolution(std::vector<FloatType>& solution_set,
                                                            const tesseract_common::VectorIsometry3d& tool_poses,
                                                            const std::vector<Eigen::VectorXd>& dof_range)
{
  double distance = std::numeric_limits<double>::min();
  int num_joints = static_cast<int>(positioner_kinematics_->numJoints());
  for (std::size_t i = 0; i < tool_poses.size(); ++i)
  {
    // Tool pose subtract robot tcp
    Eigen::Isometry3d tool_pose = tool_poses[i] * robot_tcp_.inverse();
    Eigen::Matrix<FloatType, Eigen::Dynamic, 1> positioner_pose(num_joints);
    nested_ik(0, dof_range, tool_pose, positioner_pose, solution_set, true, distance);
  }

  return !solution_set.empty();
}

}  // namespace tesseract_motion_planners
#endif  // TESSERACT_MOTION_PLANNERS_IMPL_DESCARTES_POSITIONER_SAMPLER_HPP