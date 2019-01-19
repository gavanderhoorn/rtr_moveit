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
const std::string LOGNAME = "rtr_planning_context";

// Short helper function to extract a goal pose from goal constraints.
// This will be replaced by more sophisticated methods, that support
// joint states and generate matching goal tolerances and weights.
bool getRapidPlanGoal(const std::vector<moveit_msgs::Constraints>& goal_constraints, RapidPlanGoal& goal)
{
  // TODO(henningkayser): process goal_constraints
  geometry_msgs::Pose goal_pose;
  goal_pose.position.x = 0.5;
  goal_pose.position.z = 0.3;
  goal_pose.orientation.w = 1.0;
  poseMsgToRtr(goal_pose, goal.transform);
  return true;
}

RTRPlanningContext::RTRPlanningContext(const std::string& name, const std::string& planning_group,
                                       const RTRPlannerInterfacePtr& planner_interface)
  : planning_interface::PlanningContext(name, planning_group), planner_interface_(planner_interface)
{
}

bool RTRPlanningContext::solve(planning_interface::MotionPlanResponse& res)
{
  ros::Time start_time = ros::Time::now();
  res.error_code_.val = res.error_code_.FAILURE;

  // this is always satisfied since getPlanningContext() would have failed otherwise
  if (!has_roadmap_)
    return false;

  // check planner interface
  if (!planner_interface_->isReady() && !planner_interface_->initialize())
    return false;

  // create RapidPlanGoal;
  RapidPlanGoal goal;
  if (!getRapidPlanGoal(request_.goal_constraints, goal))
    return false;

  // convert collision scene
  // TODO(henningkayser): Implement generic collision type for PCL and PlanningScene conversion
  std::vector<rtr::Voxel> collision_voxels;
  planningSceneToRtrCollisionVoxels(planning_scene_, roadmap_.volume, collision_voxels);

  // run planning attempt
  bool success = planner_interface_->solve(roadmap_, request_.start_state, goal, collision_voxels, *res.trajectory_);

  // fill response
  res.planning_time_ = (ros::Time::now() - start_time).toSec();
  res.error_code_.val = success ? res.error_code_.SUCCESS : res.error_code_.PLANNING_FAILED;
  return success;
}

bool RTRPlanningContext::solve(planning_interface::MotionPlanDetailedResponse& res)
{
  return false;
}

void RTRPlanningContext::setRoadmap(const RoadmapSpecification& roadmap)
{
  roadmap_ = roadmap_;
  // TODO(henningkayser): load volume from roadmap config file
  roadmap_.volume.base_frame = "base_link";
  roadmap_.volume.center.x = 0.1;
  roadmap_.volume.center.y = 0.1;
  roadmap_.volume.center.z = 0.1;
  roadmap_.volume.dimensions.size[0] = 1.0;
  roadmap_.volume.dimensions.size[1] = 1.0;
  roadmap_.volume.dimensions.size[2] = 1.0;
  has_roadmap_ = true;
}

void RTRPlanningContext::configure(moveit_msgs::MoveItErrorCodes& error_code)
{
  // TODO(henningkayser): move some checks and preparations to here
  error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  if (!has_roadmap_)
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "No valid roadmap specification found for group " << group_);
    error_code.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
  }
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
