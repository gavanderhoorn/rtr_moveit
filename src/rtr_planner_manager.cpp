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

#include <moveit/planning_interface/planning_interface.h>
#include <pluginlib/class_list_macros.hpp>

#include <rtr_moveit/rtr_planning_context.h>
#include <rtr_moveit/rtr_planner_interface.h>

namespace rtr_moveit
{
const std::string LOGNAME = "rtr_planner_manager";

class RTRPlannerManager : public planning_interface::PlannerManager
{
public:
  RTRPlannerManager() : planning_interface::PlannerManager(), planner_interface_(new RTRPlannerInterface())
  {
  }

  /** \brief Initializes the planner interface and load parameters */
  bool initialize(const robot_model::RobotModelConstPtr& model, const std::string& ns)
  {
    if (!planner_interface_->initialize())
    {
      ROS_ERROR_NAMED(LOGNAME, "RapidPlan interface could not be initialized!");
      return false;
    }

    // TODO(henningkayser@picknik.ai): handle groups in RobotModel
    return true;
  }

  /** \brief Verifies that the MotionPlanRequest can be handled by the planner */
  bool canServiceRequest(const moveit_msgs::MotionPlanRequest& req) const
  {
    // check if there is a roadmap configuration for the given group
    if (!planner_interface_->hasGroupConfig(req.group_name))
    {
      ROS_ERROR_STREAM_NAMED(LOGNAME, "No Planner configuration found for group " << req.group_name);
      return false;
    }

    // This version only supports single goal poses
    bool goal_constraints_valid =
        req.goal_constraints.size() == req.goal_constraints[0].position_constraints.size() ==
        req.goal_constraints[0].position_constraints[0].constraint_region.primitive_poses.size() == 1;

    if (!goal_constraints_valid)
    {
      ROS_ERROR_NAMED(LOGNAME, "Invalid goal constraints. Only single goal poses are supported!");
    }
    return goal_constraints_valid;
  }

  /** \brief Returns the planner description */
  std::string getDescription() const
  {
    return "RTR";
  }

  /** \brief Returns the names of available planning algorithms */
  void getPlanningAlgorithms(std::vector<std::string>& algs) const
  {
    algs.resize(1);
    algs[0] = "RapidPlan";
  }

  /** \brief Applies the given planner configuration to the planner */
  void setPlannerConfigurations(const planning_interface::PlannerConfigurationMap& pconfig)
  {
    // TODO(henningkayser@picknik.ai): implement setPlannerConfiguration()
    ROS_ASSERT_MSG(false, "function not implemented.");
  }

  /** \brief Returns a configured planning context for a given planning scene and motion plan request */
  planning_interface::PlanningContextPtr
  getPlanningContext(const planning_scene::PlanningSceneConstPtr& planning_scene,
                     const planning_interface::MotionPlanRequest& req, moveit_msgs::MoveItErrorCodes& error_code) const
  {
    RTRPlanningContext context("rtr_planning_context", req.group_name, planner_interface_);
    context.setMotionPlanRequest(req);
    context.setPlanningScene(planning_scene);
    return std::make_shared<RTRPlanningContext>(context);
  }

private:
  // The RapidPlan wrapper interface
  const RTRPlannerInterfacePtr planner_interface_;
};
}  // namespace rtr_moveit

PLUGINLIB_EXPORT_CLASS(rtr_moveit::RTRPlannerManager, planning_interface::PlannerManager);
