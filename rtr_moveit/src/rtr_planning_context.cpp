/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, PickNik LLC
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik LLC nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Henning Kayser
 * Desc: henningkayser@picknik.ai
 */

#include <string>
#include <vector>

#include <rtr_moveit/rtr_planning_context.h>
#include <rtr_moveit/rtr_planner_interface.h>
#include <rtr_moveit/rtr_conversions.h>

#include <rtr-occupancy/Box.hpp>

#include <moveit_msgs/Constraints.h>

namespace rtr_moveit
{
RTRPlanningContext::RTRPlanningContext(const std::string& planning_group, const RoadmapSpecification& roadmap_spec,
                                       const RTRPlannerInterfacePtr& planner_interface)
  : planning_interface::PlanningContext(planning_group + "[" + roadmap_spec.roadmap_id + "]", planning_group)
  , planner_interface_(planner_interface)
  , roadmap_(roadmap_spec)
{
  // TODO(henningkayser): load volume from roadmap config file
  roadmap_.volume.base_frame = "base_link";
  roadmap_.volume.center.x = 0.1;
  roadmap_.volume.center.y = 0.1;
  roadmap_.volume.center.z = 0.1;
  roadmap_.volume.dimensions.size[0] = 1.0;
  roadmap_.volume.dimensions.size[1] = 1.0;
  roadmap_.volume.dimensions.size[2] = 1.0;
}

moveit_msgs::MoveItErrorCodes RTRPlanningContext::solve(robot_trajectory::RobotTrajectoryPtr& trajectory,
                                                        double& planning_time)
{
  ros::Time start_time = ros::Time::now();
  moveit_msgs::MoveItErrorCodes result;
  result.val = result.FAILURE;

  // this should always be satisfied since getPlanningContext() would have failed otherwise
  if (!configured_)
  {
    ROS_ERROR_NAMED(LOGNAME, "solve() was called but planning context has not been configured successfully");
    return result;
  }

  // check planner interface
  if (!planner_interface_->isReady() && !planner_interface_->initialize())
    return result;

  // prepare collision scene
  // TODO(henningkayser): Implement generic collision type for PCL and PlanningScene conversion
  std::vector<rtr::Voxel> collision_voxels;
  planningSceneToRtrCollisionVoxels(planning_scene_, roadmap_.volume, collision_voxels);

  // convert start state
  // TODO(henningkayser): make sure joint positions are in order of kinematic chain
  rtr::Config start_config;
  sensor_msgs::JointState start_state = request_.start_state.joint_state;
  for (double joint_value : start_state.position)
    start_config.push_back(joint_value);

  // run planning attempt
  std::vector<rtr::Config> solution_path;
  double timeout = request_.allowed_planning_time * 1000;  // seconds -> milliseconds
  result.val = result.PLANNING_FAILED;
  if (planner_interface_->solve(roadmap_, start_config, goal_, collision_voxels, timeout, solution_path))
  {
    result.val = result.SUCCESS;
    trajectory.reset(new robot_trajectory::RobotTrajectory(planning_scene_->getCurrentState().getRobotModel(), group_));
    pathRtrToRobotTrajectory(solution_path, planning_scene_->getCurrentState(), start_state.name, *trajectory);
  }
  // TODO(henningkayser): connect start and goal states if necessary
  planning_time = (ros::Time::now() - start_time).toSec();
  return result;
}

bool RTRPlanningContext::solve(planning_interface::MotionPlanResponse& res)
{
  res.error_code_ = solve(res.trajectory_, res.planning_time_);
  return res.error_code_.val == res.error_code_.SUCCESS;
}

bool RTRPlanningContext::solve(planning_interface::MotionPlanDetailedResponse& res)
{
  res.trajectory_.resize(res.trajectory_.size() + 1);
  res.processing_time_.resize(res.processing_time_.size() + 1);
  res.error_code_ = solve(res.trajectory_.back(), res.processing_time_.back());
  res.description_.push_back("plan");
  return res.error_code_.val == res.error_code_.SUCCESS;
}

void RTRPlanningContext::configure(moveit_msgs::MoveItErrorCodes& error_code)
{
  error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;

  // extract RapidPlanGoal;
  if (!getRapidPlanGoal(request_.goal_constraints, goal_))
    return;

  if (request_.num_planning_attempts > 1)
    ROS_WARN_NAMED(LOGNAME, "Ignoring parameter 'num_planning_attempts' - RapidPlan is deterministic");

  error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  configured_ = true;
}

bool RTRPlanningContext::getRapidPlanGoal(const std::vector<moveit_msgs::Constraints>& goal_constraints,
                                          RapidPlanGoal& goal)
{
  if (goal_constraints.empty())
  {
    ROS_ERROR_NAMED(LOGNAME, "Cannot extract goal from empty goal constraints");
    return false;
  }

  moveit_msgs::Constraints goal_constraint = goal_constraints[0];
  if (goal_constraint.joint_constraints.size() > 0)
  {
    // TODO(henningkayser): verify order of joint constraints
    goal.type = RapidPlanGoal::Type::JOINT_STATE;
    goal.joint_state.clear();
    for (const moveit_msgs::JointConstraint& joint_constraint : goal_constraint.joint_constraints)
      goal.joint_state.push_back(joint_constraint.position);
  }
  else
  {
    // TODO(henningkayser): implement position goals
    ROS_ERROR_NAMED(LOGNAME, "Failed to extract goal from constraints. Only joint constraints support is implemented.");
    return false;
  }
  return true;
}

void RTRPlanningContext::clear()
{
}

bool RTRPlanningContext::terminate()
{
  // RapidPlan does not support this right now
  ROS_WARN_STREAM_NAMED(LOGNAME, "Failed to terminate the planning attempt! This is not supported.");
  return false;
}
}  // namespace rtr_moveit
