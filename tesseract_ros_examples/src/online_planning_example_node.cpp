/**
 * @file online_planning_example_node.cpp
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

#include <tesseract_ros_examples/online_planning_example.h>

using namespace tesseract_ros_examples;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "online_planning_example_node");
  ros::NodeHandle pnh("~");
  ros::NodeHandle nh;

  bool plotting{ true };
  bool rviz{ true };
  int steps{ 12 };
  double box_size{ 0.01 };
  bool update_start_state{ false };

  // Get ROS Parameters
  pnh.param("plotting", plotting, plotting);
  pnh.param("rviz", rviz, rviz);
  pnh.param<int>("steps", steps, steps);
  pnh.param<double>("box_size", box_size, box_size);
  pnh.param<bool>("update_start_state", update_start_state, update_start_state);

  OnlinePlanningExample example(nh, plotting, rviz, steps, box_size, update_start_state);
  example.run();
}
