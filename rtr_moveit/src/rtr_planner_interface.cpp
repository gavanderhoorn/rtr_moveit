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

#include <deque>
#include <string>
#include <vector>

#include <mutex>

#include <rtr_moveit/rtr_planner_interface.h>
#include <rtr_moveit/rtr_conversions.h>

namespace rtr_moveit
{
const std::string LOGNAME = "rtr_planner_interface";

RTRPlannerInterface::RTRPlannerInterface(const robot_model::RobotModelConstPtr& robot_model, const ros::NodeHandle& nh)
  : nh_(nh), robot_model_(robot_model)
{
}

RTRPlannerInterface::~RTRPlannerInterface()
{
  // TODO(henningkayser) implement destructor
}

bool RTRPlannerInterface::initialize()
{
  // check if hardware is connected
  if (!hardware_interface_.Connected())
  {
    ROS_ERROR_NAMED(LOGNAME, "Unable to initialize RapidPlan interface. Hardware is not connected.");
    return false;
  }

  // try to initialize hardware
  if (!hardware_interface_.Init())
  {
    ROS_ERROR_NAMED(LOGNAME, "Unable to initialize RapidPlan interface. Failed to initialize Hardware.");
    return false;
  }

  // perform handshake
  if (!hardware_interface_.Handshake())
  {
    ROS_ERROR_NAMED(LOGNAME, "Unable to initialize RapidPlan interface. Handshake failed.");
    return false;
  }

  ROS_INFO_NAMED(LOGNAME, "RapidPlan interface initialized.");
  return true;
}

bool RTRPlannerInterface::isReady() const
{
  // try handshake
  if (!hardware_interface_.Handshake())
  {
    ROS_WARN_NAMED(LOGNAME, "RapidPlan interface is not ready. Handshake failed.");
    return false;
  }

  ROS_DEBUG_NAMED(LOGNAME, "RapidPlan interface is ready.");
  return true;
}

bool RTRPlannerInterface::solve(const RoadmapSpecification& roadmap_spec, const moveit_msgs::RobotState& start_state,
                                const RapidPlanGoal& goal, const std::vector<rtr::Voxel>& occupancy_voxels,
                                robot_trajectory::RobotTrajectory& trajectory)
{
  // TODO(henningkayser): implement state search
  // Find closest existing configuration in roadmap that can be connected to the start state
  unsigned int start_id = 0;  // dummy

  // planning result
  int result = -1;
  std::deque<unsigned int> waypoints, edges;
  {  // SCOPED MUTEX LOCK
    std::lock_guard<std::mutex> scoped_lock(mutex_);

    // load roadmap and get roadmap index
    uint16_t roadmap_index;
    if (!prepareRoadmap(roadmap_spec, roadmap_index))
      return false;

    // run collision check with HardwareInterface
    std::vector<uint8_t> collisions;
    if (!hardware_interface_.CheckScene(occupancy_voxels, roadmap_index, collisions))
    {
      ROS_ERROR_NAMED(LOGNAME, "HardwareInterface failed to check collision scene.");
      return false;
    }

    // call PathPlanner
    if (goal.type == RapidPlanGoal::Type::TRANSFORM)
      result = planner_.FindPath(start_id, goal.transform, collisions, goal.tolerance, goal.weights, waypoints, edges);
    else if (goal.type == RapidPlanGoal::Type::STATE_IDS)
      result = planner_.FindPath(start_id, goal.state_ids, collisions, waypoints, edges);
  }  // SCOPED MUTEX UNLOCK

  // process result
  if (result == 0)
    processSolutionPath(waypoints, edges, trajectory);
  else
    ROS_ERROR_STREAM_NAMED(LOGNAME, "RapidPlan failed at finding a valid path - " << planner_.GetError(result));

  // TODO(henningkayser): Connect start and goal state to trajectory

  return result == 0;  // 0 == SUCESS
}

bool RTRPlannerInterface::prepareRoadmap(const RoadmapSpecification& roadmap_spec, uint16_t& roadmap_index)
{
  // save new roadmap if it is new
  std::string roadmap_id = roadmap_spec.roadmap_id;
  if (roadmaps_.find(roadmap_spec.roadmap_id) == roadmaps_.end())
    roadmaps_[roadmap_spec.roadmap_id] = roadmap_spec;

  // TODO(henningkayser): Only store *.og file paths, others will be deprecated with the next API
  RoadmapFiles files = roadmaps_[roadmap_id].files;

  // verify that the roadmap is loaded in the PathPlanner
  if (roadmap_id != loaded_roadmap_)
  {
    if (!planner_.LoadRoadmap(files.configs, files.edges, files.transforms))
    {
      ROS_ERROR_STREAM_NAMED(LOGNAME, "Failed to load roadmap '" << roadmap_id << "' to RapidPlan PathPlanner.");
      return false;
    }
  }

  // check if roadmap is already written to hardware
  if (!findRoadmapIndex(roadmap_id, roadmap_index))
  {
    // write roadmap and retrieve new roadmap index
    if (!hardware_interface_.WriteRoadmap(files.occupancy, roadmap_index))
    {
      ROS_ERROR_STREAM_NAMED(LOGNAME, "Failed to write roadmap '" << roadmap_id << "' to RapidPlan MPU.");
      return false;
    }
    roadmap_indices_[roadmap_index] = roadmap_id;
  }
  ROS_INFO_STREAM_NAMED(LOGNAME, "RapidPlan initialized with with roadmap '" << roadmap_id << "'");
  return true;
}

void RTRPlannerInterface::processSolutionPath(const std::deque<unsigned int>& waypoints,
                                              const std::deque<unsigned int>& edges,
                                              robot_trajectory::RobotTrajectory& trajectory) const
{
  // TODO(henningkayser): implement trajectory processing
  ROS_ASSERT_MSG(false, "Function not implemented.");
}
}  // namespace rtr_moveit
