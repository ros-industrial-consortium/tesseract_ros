/**
 * @file online_planning_example.h
 * @brief This example demonstrates using trajopt to plan in an "online" manner. As the environment is changed (using
 * the joint state publisher), the system will attempt to avoid collisions with the moving object and follow the moving
 * target
 *
 * Note: If the target moves too quickly the solver can get stuck in an infeasible point. That is the nature of how the
 * solver is working. Higher level intelligence, a larger step size, or changing the target to a cost can solve that
 * problem.
 *
 * @author Matthew Powelson
 * @date June 9, 2020
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
#ifndef TESSERACT_ROS_EXAMPLES_ONLINE_PLANNING_EXAMPLE_H
#define TESSERACT_ROS_EXAMPLES_ONLINE_PLANNING_EXAMPLE_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <string>
#include <Eigen/Eigen>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/SetBool.h>
#include <trajopt_sqp/qp_problem.h>

#include <trajopt_ifopt/constraints/cartesian_position_constraint.h>
#include <tesseract_rosutils/plotting.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_ros_examples/example.h>
#include <tesseract_visualization/trajectory_player.h>

namespace tesseract_ros_examples
{
/**
 * @brief This example demonstrates running the inner loop of TrajOpt in an "online" manner. As the environment changes,
 * TrajOpt dynamically adjusts to avoid collisions and follow the given toolpath
 */
class OnlinePlanningExample : public tesseract_ros_examples::Example
{
public:
  OnlinePlanningExample(const ros::NodeHandle& nh,
                        bool plotting,
                        bool rviz,
                        int steps,
                        double box_size,
                        bool update_start_state);

  ~OnlinePlanningExample() override = default;
  OnlinePlanningExample(const OnlinePlanningExample&) = default;
  OnlinePlanningExample& operator=(const OnlinePlanningExample&) = default;
  OnlinePlanningExample(OnlinePlanningExample&&) = default;
  OnlinePlanningExample& operator=(OnlinePlanningExample&&) = default;

  /**
   * @brief Runs the example
   * @return True if successful
   */
  bool run() override;

  /**
   * @brief Sets up the IFOPT problem
   * @return True if successful
   */
  bool setupProblem(std::vector<Eigen::VectorXd> initial_trajectory = std::vector<Eigen::VectorXd>());

  /**
   * @brief Runs the online planning process, occoasionally checking ROS callbacks until realtime_running_ = false
   * @return True if successful
   */
  bool onlinePlan();

  /** @brief ROS subscriber callback used to update the dynamic collision obstacle and target position */
  void subscriberCallback(const sensor_msgs::JointState::ConstPtr& joint_state);

  /** @brief ROS service callback for toggling the online planning on/off */
  bool toggleRealtime(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

private:
  ros::NodeHandle nh_;
  int steps_;
  double box_size_;
  bool update_start_state_;

  tesseract_kinematics::ForwardKinematics::Ptr manipulator_fk_;
  tesseract_kinematics::InverseKinematics::Ptr manipulator_ik_;
  std::shared_ptr<tesseract_rosutils::ROSPlotting> plotter_;
  tesseract_environment::AdjacencyMap::Ptr manipulator_adjacency_map_;

  tesseract_visualization::TrajectoryPlayer player_;
  trajopt::TrajArray current_trajectory_;
  Eigen::Isometry3d target_pose_delta_;
  Eigen::Isometry3d target_pose_base_frame_;
  // We need to keep this around so we can update it
  trajopt_ifopt::CartPosConstraint::Ptr target_pose_constraint_;

  std::vector<std::string> joint_names_;
  ros::Subscriber joint_state_subscriber_;
  ros::ServiceServer toggle_realtime_service_;
  trajopt_sqp::QPProblem::Ptr nlp_;

  bool realtime_running_;

  void updateAndPlotTrajectory(const Eigen::VectorXd& osqp_vals);
};
}  // namespace tesseract_ros_examples

#endif  // TESSERACT_ROS_EXAMPLES_BASIC_CARTESIAN_EXAMPLE_H
