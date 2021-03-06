/**
 * @file glass_upright_ompl_example_node.cpp
 * @brief Glass upright OMPL example node
 *
 * @author Levi Armstrong
 * @date March 16, 2020
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

#include <tesseract_ros_examples/glass_upright_ompl_example.h>

using namespace tesseract_ros_examples;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "glass_upright_ompl_example_node");
  ros::NodeHandle pnh("~");
  ros::NodeHandle nh;

  bool plotting = true;
  bool rviz = true;
  double range = 0.01;
  bool use_trajopt_constraint = false;
  double planning_time = 60.0;

  // Get ROS Parameters
  pnh.param("plotting", plotting, plotting);
  pnh.param("rviz", rviz, rviz);
  pnh.param("range", range, range);
  pnh.param("use_trajopt_constraint", use_trajopt_constraint, use_trajopt_constraint);
  pnh.param("planning_time", planning_time, planning_time);

  GlassUprightOMPLExample example(nh, plotting, rviz, range, use_trajopt_constraint, planning_time);
  example.run();
}
