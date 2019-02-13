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

#ifndef RTR_MOVEIT_RTR_PLANNER_INTERFACE_H
#define RTR_MOVEIT_RTR_PLANNER_INTERFACE_H

// temporarily disable RapidPlanInterface
#define RAPID_PLAN_INTERFACE_ENABLED 0

#include <deque>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <rtr-api/PathPlanner.hpp>
#include <rtr-occupancy/Voxel.hpp>

#if RAPID_PLAN_INTERFACE_ENABLED
#include <rtr-api/RapidPlanInterface.hpp>
#endif

#include <rtr_moveit/rtr_datatypes.h>

#include <moveit/macros/class_forward.h>

namespace rtr_moveit
{
MOVEIT_CLASS_FORWARD(RTRPlannerInterface);

// A RapidPlan goal specification
struct RapidPlanGoal
{
  // RapidPlan supports either ids of roadmap states or tool pose transforms as goals.
  // The third JOINT_STATE is handled by looking for nearby roadmap states and calling
  // the planner with corresponding STATE_IDS.
  enum Type
  {
    STATE_IDS,
    TRANSFORM,
    JOINT_STATE
  };
  Type type;

  // STATE_IDS: a list of target states in the roadmap
  std::vector<uint> state_ids;

  // TRANSFORM: an endeffector transform to look for a target state
  std::array<float, 6> transform;
  std::array<float, 6> tolerance;  // pose tolerance of the target state
  std::array<float, 6> weights;    // pose distance weights for ranking multiple solutions

  // JOINT_STATE
  std::vector<float> joint_state;
};

class RTRPlannerInterface
{
public:
  RTRPlannerInterface(const ros::NodeHandle& nh);
  virtual ~RTRPlannerInterface();

  /** \brief Initialize the RapidPlanInterface */
  bool initialize();

  /** \brief Check if the RapidPlanInterface is available and the planner can receive requests */
  bool isReady() const;

  /** \brief Run planning attempt and generate a solution path */
  bool solve(const RoadmapSpecification& roadmap_spec, const rtr::Config& start_config, const RapidPlanGoal& goal,
             const std::vector<rtr::Voxel>& occupancy_voxels, const double& timeout,
             std::vector<rtr::Config>& solution_path);

  /** \brief Run planning attempt and generate solution waypoints and edges */
  bool solve(const RoadmapSpecification& roadmap_spec, const rtr::Config& start_config, const RapidPlanGoal& goal,
             const std::vector<rtr::Voxel>& occupancy_voxels, const double& timeout,
             std::vector<rtr::Config>& roadmap_states, std::deque<unsigned int>& waypoints,
             std::deque<unsigned int>& edges);

private:
  /** \brief Initialize PathPlanner and RapidPlanInterface with a given roadmap identifier */
  bool prepareRoadmap(const RoadmapSpecification& roadmap_spec, uint16_t& roadmap_index);

  /** \brief Find the roadmap index for a given roadmap name */
  bool findRoadmapIndex(const std::string& roadmap_name, uint16_t& roadmap_index)
  {
    for (auto it = roadmap_indices_.begin(); it != roadmap_indices_.end(); it++)
    {
      if (it->second == roadmap_name)
      {
        roadmap_index = it->first;
        return true;
      }
    }
    return false;
  }

  ros::NodeHandle nh_;
  bool debug_ = false;

  // mutex lock for thread-safe RapidPlan calls
  std::mutex mutex_;

// RapidPlan interfaces
#if RAPID_PLAN_INTERFACE_ENABLED
  rtr::RapidPlanInterface rapidplan_interface_;
#endif

  rtr::PathPlanner planner_;

  // available roadmap specifications
  std::map<std::string, RoadmapSpecification> roadmaps_;
  // name of roadmap loaded by the planner
  std::string loaded_roadmap_;
  // indices of roadmaps written to the board
  std::map<uint16_t, std::string> roadmap_indices_;
};
}  // namespace rtr_moveit

#endif  // RTR_MOVEIT_RTR_PLANNER_INTERFACE_H
