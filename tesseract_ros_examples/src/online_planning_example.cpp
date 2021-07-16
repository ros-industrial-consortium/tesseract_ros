/**
 * @file online_planning_example.cpp
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
 * @copyright Copyright (c) 2020, Southwest Research Institute
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
#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <ros/ros.h>

#include <trajopt_sqp/ifopt_qp_problem.h>
#include <trajopt_sqp/trajopt_qp_problem.h>
#include <trajopt_sqp/trust_region_sqp_solver.h>
#include <trajopt_sqp/osqp_eigen_solver.h>
#include <trajopt_sqp/types.h>

#include <trajopt_ifopt/constraints/cartesian_position_constraint.h>
#include <trajopt_ifopt/constraints/joint_position_constraint.h>
#include <trajopt_ifopt/constraints/joint_velocity_constraint.h>
#include <trajopt_ifopt/constraints/discrete_collision_constraint.h>
#include <trajopt_ifopt/constraints/continuous_collision_constraint.h>
#include <trajopt_ifopt/constraints/inverse_kinematics_constraint.h>
#include <trajopt_ifopt/variable_sets/joint_position_variable.h>
#include <trajopt_ifopt/costs/squared_cost.h>
#include <trajopt_ifopt/costs/absolute_cost.h>
#include <trajopt_ifopt/utils/ifopt_utils.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_ros_examples/online_planning_example.h>
#include <tesseract_visualization/markers/axis_marker.h>

using namespace trajopt;
using namespace tesseract_environment;
using namespace tesseract_scene_graph;
using namespace tesseract_collision;
using namespace tesseract_rosutils;
using namespace tesseract_visualization;

/** @brief Default ROS parameter for robot description */
const std::string ROBOT_DESCRIPTION_PARAM = "robot_description";

/** @brief Default ROS parameter for robot description */
const std::string ROBOT_SEMANTIC_PARAM = "robot_description_semantic";

/** @brief Dynamic object joint states topic */
const std::string DYNAMIC_OBJECT_JOINT_STATE = "/joint_states";

namespace tesseract_ros_examples
{
OnlinePlanningExample::OnlinePlanningExample(const ros::NodeHandle& nh,
                                             bool plotting,
                                             bool rviz,
                                             int steps,
                                             double box_size,
                                             bool update_start_state)
  : Example(plotting, rviz)
  , nh_(nh)
  , steps_(steps)
  , box_size_(box_size)
  , realtime_running_(false)
  , update_start_state_(update_start_state)
{
  // Import URDF/SRDF
  std::string urdf_xml_string, srdf_xml_string;
  nh_.getParam(ROBOT_DESCRIPTION_PARAM, urdf_xml_string);
  nh_.getParam(ROBOT_SEMANTIC_PARAM, srdf_xml_string);

  ResourceLocator::Ptr locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
  if (!env_->init<OFKTStateSolver>(urdf_xml_string, srdf_xml_string, locator))
    assert(false);

  // Set up plotting
  plotter_ = std::make_shared<tesseract_rosutils::ROSPlotting>(env_->getSceneGraph()->getRoot());

  // Extract necessary kinematic information
  manipulator_fk_ = env_->getManipulatorManager()->getFwdKinematicSolver("manipulator");
  manipulator_adjacency_map_ = std::make_shared<tesseract_environment::AdjacencyMap>(
      env_->getSceneGraph(), manipulator_fk_->getActiveLinkNames(), env_->getCurrentState()->link_transforms);
  manipulator_ik_ = env_->getManipulatorManager()->getInvKinematicSolver("manipulator");

  // Initialize the trajectory
  current_trajectory_ = trajopt::TrajArray::Zero(steps_, 10);
  joint_names_ = { "gantry_axis_1", "gantry_axis_2", "joint_1", "joint_2",       "joint_3",
                   "joint_4",       "joint_5",       "joint_6", "human_x_joint", "human_y_joint" };

  // Set up ROS interfaces
  joint_state_subscriber_ =
      nh_.subscribe(DYNAMIC_OBJECT_JOINT_STATE, 1, &OnlinePlanningExample::subscriberCallback, this);
  toggle_realtime_service_ = nh_.advertiseService("toggle_realtime", &OnlinePlanningExample::toggleRealtime, this);

  target_pose_delta_ = Eigen::Isometry3d::Identity();
  target_pose_base_frame_ = Eigen::Isometry3d::Identity();
}

void OnlinePlanningExample::subscriberCallback(const sensor_msgs::JointState::ConstPtr& joint_state)
{
  // Set the environment state to update the collision model
  env_->setState(joint_state->name, joint_state->position);

  // Update current_trajectory_ so the live trajectory will be visualized correctly
  for (Eigen::Index i = 0; i < current_trajectory_.rows(); i++)
  {
    current_trajectory_.block(i, 8, 1, 2) << joint_state->position[8], joint_state->position[9];
  }

  // Update the target location
  if (target_pose_constraint_)
  {
    target_pose_delta_ = Eigen::Isometry3d::Identity();
    target_pose_delta_.translate(
        Eigen::Vector3d(joint_state->position[10], joint_state->position[11], joint_state->position[12]));
    target_pose_constraint_->SetTargetPose(target_pose_base_frame_ * target_pose_delta_);

    plotter_->clear();
    tesseract_visualization::AxisMarker am(target_pose_base_frame_ * target_pose_delta_);
    am.setScale(Eigen::Vector3d::Constant(0.3));
    plotter_->plotMarker(am);
  }
}

bool OnlinePlanningExample::toggleRealtime(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
  realtime_running_ = req.data;
  res.success = true;
  if (realtime_running_)
    onlinePlan();
  return true;
}

bool OnlinePlanningExample::run()
{
  ROS_ERROR("Press enter to setup the problem");
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  setupProblem();

  ROS_ERROR("Press enter to run live");
  ROS_ERROR("Then use the joint state publisher gui to move the human_x/y joints or the target_x/y/z joints");
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  realtime_running_ = true;
  onlinePlan();

  return true;
}

bool OnlinePlanningExample::setupProblem(std::vector<Eigen::VectorXd> initial_trajectory)
{
  // 1) Create the problem
  nlp_ = std::make_shared<trajopt_sqp::TrajOptQPProblem>();

  // 2) Add Variables
  Eigen::MatrixX2d joint_limits_eigen = manipulator_fk_->getLimits().joint_limits;
  Eigen::VectorXd current_position = env_->getCurrentJointValues(manipulator_fk_->getJointNames());
  //  Eigen::VectorXd home_position = Eigen::VectorXd::Zero(manipulator_fk_->numJoints());
  Eigen::VectorXd target_joint_position(manipulator_fk_->numJoints());
  target_joint_position << 5.5, 3, 0, 0, 0, 0, 0, 0;

  std::vector<trajopt_ifopt::JointPosition::ConstPtr> vars;
  std::vector<Eigen::VectorXd> initial_states;
  if (initial_trajectory.empty())
    initial_states = trajopt_ifopt::interpolate(current_position, target_joint_position, steps_);
  else
    initial_states = initial_trajectory;

  for (std::size_t ind = 0; ind < static_cast<std::size_t>(steps_); ind++)
  {
    auto var = std::make_shared<trajopt_ifopt::JointPosition>(
        initial_states[ind], manipulator_fk_->getJointNames(), "Joint_Position_" + std::to_string(ind));
    var->SetBounds(joint_limits_eigen);
    vars.push_back(var);
    nlp_->addVariableSet(var);
  }

  // 3) Add costs and constraints
  // Add the home position as a joint position constraint
  {
    //    auto home_position = Eigen::VectorXd::Zero(8);
    std::vector<trajopt_ifopt::JointPosition::ConstPtr> var_vec(1, vars[0]);
    auto home_constraint =
        std::make_shared<trajopt_ifopt::JointPosConstraint>(current_position, var_vec, "Home_Position");
    nlp_->addConstraintSet(home_constraint);
  }
  // Add the target pose constraint for the final step
  {
    target_pose_base_frame_ = manipulator_fk_->calcFwdKin(target_joint_position);
    Eigen::Isometry3d target_tf = target_pose_base_frame_ * target_pose_delta_;
    std::cout << "Target Joint Position: " << target_joint_position.transpose() << std::endl;
    std::cout << "Target TF:\n" << target_tf.matrix() << std::endl;

    auto kinematic_info = std::make_shared<trajopt_ifopt::KinematicsInfo>(
        manipulator_fk_, manipulator_adjacency_map_, Eigen::Isometry3d::Identity());

    trajopt_ifopt::CartPosInfo cart_info(kinematic_info, target_tf, manipulator_fk_->getTipLinkName());
    target_pose_constraint_ = std::make_shared<trajopt_ifopt::CartPosConstraint>(cart_info, vars.back());
    nlp_->addConstraintSet(target_pose_constraint_);
  }
  // Add joint velocity cost for all timesteps
  {
    Eigen::VectorXd vel_target = Eigen::VectorXd::Zero(8);
    auto vel_constraint = std::make_shared<trajopt_ifopt::JointVelConstraint>(vel_target, vars, "JointVelocity");
    nlp_->addCostSet(vel_constraint, trajopt_sqp::CostPenaltyType::SQUARED);
  }
  // Add a collision cost for all steps
  double margin_coeff = 10;
  double margin = 0.1;
  auto collision_config = std::make_shared<trajopt_ifopt::TrajOptCollisionConfig>(margin, margin_coeff);
  collision_config->contact_request.type = tesseract_collision::ContactTestType::ALL;
  collision_config->type = CollisionEvaluatorType::DISCRETE;
  collision_config->collision_margin_buffer = 0.10;

  auto collision_cache = std::make_shared<trajopt_ifopt::CollisionCache>(steps_);
  for (std::size_t i = 1; i < static_cast<std::size_t>(steps_) - 1; i++)
  {
    auto collision_evaluator =
        std::make_shared<trajopt_ifopt::LVSDiscreteCollisionEvaluator>(collision_cache,
                                                                       manipulator_fk_,
                                                                       env_,
                                                                       manipulator_adjacency_map_,
                                                                       Eigen::Isometry3d::Identity(),
                                                                       collision_config,
                                                                       true);

    std::array<trajopt_ifopt::JointPosition::ConstPtr, 3> position_vars = { vars[i - 1], vars[i], vars[i + 1] };

    auto collision_constraint = std::make_shared<trajopt_ifopt::ContinuousCollisionConstraintIfopt>(
        collision_evaluator,
        trajopt_ifopt::ContinuousCombineCollisionData(trajopt_ifopt::CombineCollisionDataMethod::WEIGHTED_AVERAGE),
        position_vars);
    nlp_->addCostSet(collision_constraint, trajopt_sqp::CostPenaltyType::SQUARED);
  }

  nlp_->setup();
  nlp_->print();

  return true;
}

// Convert to joint trajectory
tesseract_common::JointTrajectory getJointTrajectory(const std::vector<std::string>& joint_names,
                                                     const trajopt::TrajArray& current_trajectory)
{
  tesseract_common::JointTrajectory joint_traj;
  joint_traj.reserve(static_cast<std::size_t>(current_trajectory.rows()));
  double total_time = 0;
  for (long i = 0; i < current_trajectory.rows(); ++i)
  {
    tesseract_common::JointState js(joint_names, current_trajectory.row(i));
    js.time = total_time;
    joint_traj.push_back(js);
    total_time += 0.1;
  }
  return joint_traj;
}

void OnlinePlanningExample::updateAndPlotTrajectory(const Eigen::VectorXd& osqp_vals)
{
  // Update manipulator joint values
  Eigen::Map<const trajopt::TrajArray> trajectory(osqp_vals.data(), steps_, 8);
  current_trajectory_.block(0, 0, steps_, 8) = trajectory;

  // Convert to joint trajectory
  tesseract_common::JointTrajectory joint_traj = getJointTrajectory(joint_names_, current_trajectory_);
  player_.setTrajectory(joint_traj);

  // Display Results
  plotter_->plotTrajectory(joint_traj, env_->getStateSolver());
}

bool OnlinePlanningExample::onlinePlan()
{
  ros::spinOnce();

  // Setup Solver
  auto qp_solver = std::make_shared<trajopt_sqp::OSQPEigenSolver>();
  trajopt_sqp::TrustRegionSQPSolver solver(qp_solver);

  // Adjust this to be larger to adapt quicker but more jerkily
  //  solver.init(qp_problem);
  solver.verbose = true;
  solver.solve(nlp_);
  Eigen::VectorXd x = solver.getResults().best_var_vals;
  updateAndPlotTrajectory(x);
  plotter_->waitForInput("View global trajectory. Hit enter to run online planner.");

  solver.params.initial_trust_box_size = box_size_;
  solver.init(nlp_);

  console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO);

  using namespace std::chrono;
  auto prev_start = high_resolution_clock::now();
  while (realtime_running_ && ros::ok())
  {
    ros::spinOnce();
    auto start = high_resolution_clock::now();
    auto dt = 0.01 * duration<double>(start - prev_start).count();
    prev_start = start;

    // Calculate current position of the robot and update environment current state
    if (update_start_state_)
    {
      tesseract_common::JointState state = player_.setCurrentDuration(dt);
      std::vector<Eigen::VectorXd> init_trajectory;
      Eigen::VectorXd time_state = Eigen::VectorXd::LinSpaced(steps_, dt, player_.trajectoryDuration());
      for (Eigen::Index t = 0; t < steps_; t++)
      {
        tesseract_common::JointState state = player_.setCurrentDuration(time_state(t));
        init_trajectory.push_back(state.position.head(manipulator_fk_->numJoints()));
      }

      env_->setState(state.joint_names, state.position);

      // Setup problem again which should use a new start state
      setupProblem(init_trajectory);
    }

    // Reset the box size because now the trust region loop is being ran and it can go to zero
    solver.params.initial_trust_box_size = box_size_;

    // Initialize the solver
    solver.init(nlp_);

    // Loop over the updates because the visualization is slow
    int num_steps = 1;
    for (int i = 0; i < num_steps; i++)
    {
      // Step the SQP Solver
      // This will convexify cost and constraints and run the trust region loop
      solver.stepSQPSolver();

      /** @todo Should check status here and maybe solve global plan again */

      // Update the results
      x = solver.getResults().best_var_vals;

      // Rest box size because now the trust region loop is being ran and it can go to zero
      solver.setBoxSize(box_size_);
    }

    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start) / static_cast<double>(num_steps);

    // Update manipulator joint values and plot trajectory
    updateAndPlotTrajectory(x);

    //    std::string message =
    //        "Solver Frequency (Hz): " + std::to_string(1.0 / static_cast<double>(duration.count()) * 1000000.) +
    //        "\nCost: " + std::to_string(nlp_->evaluateCostFunction(x.data()));
    //    std::cout << message << std::endl;
  }

  return true;
}

}  // namespace tesseract_ros_examples
