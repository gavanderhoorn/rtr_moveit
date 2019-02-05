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
#include <algorithm>

#include <Eigen/Geometry>

#include <rtr_moveit/rtr_planning_context.h>
#include <rtr_moveit/rtr_planner_interface.h>
#include <rtr_moveit/rtr_conversions.h>

#include <rtr-occupancy/Box.hpp>

#include <moveit_msgs/Constraints.h>

namespace rtr_moveit
{
static const std::string LOGNAME = "rtr_planning_context";
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

  // Iterate goals until we have a solution
  for (const RapidPlanGoal& goal : goals_)
  {
    if (planner_interface_->solve(roadmap_, start_config, goal, collision_voxels, timeout, solution_path))
    {
      if (solution_path.empty())
      {
        ROS_WARN_NAMED(LOGNAME, "Cannot convert empty path to robot trajectory");
      }
      else if (start_state.name.size() != solution_path[0].size())
      {
        ROS_WARN_NAMED(LOGNAME, "Cannot convert path - Joint values don't match joint names");
      }
      else
      {
        // convert solution path to robot trajectory
        result.val = result.SUCCESS;
        const robot_state::RobotState& reference_state = planning_scene_->getCurrentState();
        trajectory.reset(new robot_trajectory::RobotTrajectory(reference_state.getRobotModel(), group_));
        pathRtrToRobotTrajectory(solution_path, reference_state, start_state.name, *trajectory);
        break;
      }
    }
    solution_path.clear();
    // TODO(henningkayser): reduce timeout
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

  if (!planning_scene_)
  {
    ROS_ERROR_NAMED(LOGNAME, "Cannot configure planning context while planning scene has not been set");
    return;
  }

  jmg_ = planning_scene_->getCurrentState().getJointModelGroup(group_);

  // extract RapidPlanGoals;
  if (!getRapidPlanGoals(request_.goal_constraints, goals_))
    return;

  if (request_.num_planning_attempts > 1)
    ROS_INFO_NAMED(LOGNAME, "Ignoring parameter 'num_planning_attempts' - RapidPlan is deterministic");

  error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  configured_ = true;
}

bool RTRPlanningContext::getRapidPlanGoals(const std::vector<moveit_msgs::Constraints>& goal_constraints,
                                           std::vector<RapidPlanGoal>& goals)
{
  bool success = false;
  for (const moveit_msgs::Constraints& goal_constraint : goal_constraints)
  {
    RapidPlanGoal goal;
    if (getRapidPlanGoal(goal_constraint, goal))
      goals.push_back(goal);
  }
  std::string error_msg;
  if (goal_constraints.empty())
    ROS_ERROR_NAMED(LOGNAME, "Goal constraints are empty");
  else if (goals.empty())
    ROS_ERROR_NAMED(LOGNAME, "Failed to extract any goals from constraints");
  else
    success = true;
  return success;
}

bool RTRPlanningContext::getRapidPlanGoal(const moveit_msgs::Constraints& goal_constraint, RapidPlanGoal& goal)
{
  bool success = false;
  std::string warn_msg;

  bool has_joint_constraint = goal_constraint.joint_constraints.size() > 0;
  bool has_position_constraint = goal_constraint.position_constraints.size() == 1;
  bool has_orientation_constraint = goal_constraint.orientation_constraints.size() == 1;
  if (has_joint_constraint == (has_position_constraint || has_orientation_constraint))
  {
    warn_msg = "Constraints must either include joint or position/orientation constraints";
  }
  else if (goal_constraint.visibility_constraints.size() > 0)
  {
    warn_msg = "Found visibility constraints which are not supported";
  }
  else if (has_joint_constraint)  // JOINT GOAL
  {
    std::vector<std::string> joint_names = jmg_->getActiveJointModelNames();
    if (goal_constraint.joint_constraints.size() != joint_names.size())
    {
      warn_msg = "Invalid number of joint constraints";
    }
    else
    {
      goal.type = RapidPlanGoal::Type::JOINT_STATE;
      for (const std::string& joint_name : joint_names)
        for (const moveit_msgs::JointConstraint& joint_constraint : goal_constraint.joint_constraints)
          if (joint_constraint.joint_name == joint_name)
            goal.joint_state.push_back(joint_constraint.position);
      success = goal.joint_state.size() == joint_names.size();  // all joints must be set
      warn_msg = "Joint constraints contain duplicates";        // if !success
    }
  }
  else  // TRANSFORM GOAL
  {
    // set transform tolerance high by default
    goal.tolerance.fill(std::numeric_limits<float>::max());
    goal.type = RapidPlanGoal::Type::TRANSFORM;
    bool position_constraint_failed = has_position_constraint;
    if (has_position_constraint)
    {
      const moveit_msgs::PositionConstraint& position_constraint = goal_constraint.position_constraints[0];
      if (position_constraint.link_name != jmg_->getLinkModelNames().back())
      {
        warn_msg = "Position constraint does not apply to the endeffector";
      }
      else if (position_constraint.header.frame_id != roadmap_.volume.base_frame)
      {
        warn_msg = "Frame of Position constraint does not align with the roadmap frame";
      }
      else if (position_constraint.constraint_region.primitives.size() != 1 &&
               position_constraint.constraint_region.primitive_poses.size() != 1)
      {
        warn_msg = "Invalid number of position constraint region primitives, must be 1";
      }
      else if (position_constraint.constraint_region.primitives[0].type != shape_msgs::SolidPrimitive::BOX)
      {
        warn_msg = "Position constraint region is not of type BOX";
      }
      else
      {
        // TODO(henningkayser): check if primitive orientation aligns with volume region
        goal.transform[0] = position_constraint.constraint_region.primitive_poses[0].position.x;
        goal.transform[1] = position_constraint.constraint_region.primitive_poses[0].position.y;
        goal.transform[2] = position_constraint.constraint_region.primitive_poses[0].position.z;
        goal.tolerance[0] = position_constraint.constraint_region.primitives[0].dimensions[0];
        goal.tolerance[1] = position_constraint.constraint_region.primitives[0].dimensions[1];
        goal.tolerance[2] = position_constraint.constraint_region.primitives[0].dimensions[2];
        position_constraint_failed = false;
      }
    }
    bool orientation_constraint_failed = has_orientation_constraint;
    if (has_orientation_constraint)
    {
      const moveit_msgs::OrientationConstraint& orientation_constraint = goal_constraint.orientation_constraints[0];
      if (orientation_constraint.link_name != jmg_->getLinkModelNames().back())
      {
        warn_msg = "Orientation constraint does not apply to the endeffector";
      }
      if (orientation_constraint.header.frame_id != roadmap_.volume.base_frame)
      {
        warn_msg = "Frame of orientation constraint does not align with the roadmap frame";
      }
      else
      {
        geometry_msgs::Quaternion q = orientation_constraint.orientation;
        Eigen::Quaterniond orientation(q.w, q.x, q.y, q.z);
        Eigen::Vector3d euler_angles = orientation.toRotationMatrix().eulerAngles(0, 1, 2);
        goal.transform[3] = euler_angles[0];
        goal.transform[4] = euler_angles[1];
        goal.transform[5] = euler_angles[2];
        goal.tolerance[3] = orientation_constraint.absolute_x_axis_tolerance;
        goal.tolerance[4] = orientation_constraint.absolute_y_axis_tolerance;
        goal.tolerance[5] = orientation_constraint.absolute_z_axis_tolerance;
        orientation_constraint_failed = false;
      }
    }
    success = !(position_constraint_failed || orientation_constraint_failed);  // none of position/orientation failed
  }

  if (!success)
    ROS_WARN_STREAM_NAMED(LOGNAME, "Failed to process goal constraint - " << warn_msg);
  return success;
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
