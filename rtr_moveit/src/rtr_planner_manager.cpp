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

#include <map>
#include <set>
#include <string>
#include <vector>

#include <moveit/planning_interface/planning_interface.h>
#include <pluginlib/class_list_macros.hpp>

#include <rtr_moveit/rtr_planning_context.h>
#include <rtr_moveit/rtr_planner_interface.h>

// ROS parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <ros/package.h>
#include <boost/filesystem.hpp>

namespace rtr_moveit
{
const std::string LOGNAME = "rtr_planner_manager";

class RTRPlannerManager : public planning_interface::PlannerManager
{
public:
  RTRPlannerManager() : planning_interface::PlannerManager(), nh_("~")
  {
  }

  /** \brief Initializes the planner interface and load parameters */
  bool initialize(const robot_model::RobotModelConstPtr& robot_model, const std::string& ns)
  {
    // load config
    loadRoadmapConfigurations(robot_model->getJointModelGroupNames());
    if (group_configs_.empty() || roadmaps_.empty())
    {
      ROS_ERROR_NAMED(LOGNAME, "Failed at loading any group configurations from config file.");
      return false;
    }

    // initialize planner
    planner_interface_.reset(new RTRPlannerInterface(robot_model, nh_));
    if (!planner_interface_->initialize())
    {
      ROS_ERROR_NAMED(LOGNAME, "RapidPlan interface could not be initialized!");
      return false;
    }

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

    // TODO(henningkayser): Move this (and getGoalPose() in planning context) to a an extra goal specification class
    // This version only supports single goal poses
    if (req.goal_constraints.size() != 1)
    {
      ROS_ERROR_NAMED(LOGNAME, "Received an invalid number of goal constraints. Should be 1.");
      return false;
    }
    if (req.goal_constraints[0].position_constraints.size() != 1)
    {
      ROS_ERROR_NAMED(LOGNAME, "Received an invalid number of posiiton constraints. Should be 1.");
      return false;
    }
    if (req.goal_constraints[0].position_constraints[0].constraint_region.primitive_poses.size() != 1)
    {
      ROS_ERROR_NAMED(LOGNAME, "Received an invalid number of goal poses. Should be 1.");
      return false;
    }

    return true;
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

  void loadRoadmapConfigurations(const std::vector<std::string>& group_names)
  {
    // load group configs
    group_configs_.clear();
    std::set<std::string> roadmap_ids;
    for (const std::string& group_name : group_names)
    {
      if (!nh_.hasParam(group_name))
      {
        ROS_INFO_STREAM_NAMED(LOGNAME, "No roadmap specification found for goup " << group_name);
        continue;
      }

      // create new group config
      GroupConfig config;
      config.group_name = group_name;
      std::vector<std::string> group_roadmap_ids;
      rosparam_shortcuts::get(LOGNAME, nh_, group_name + "/default_roadmap", config.default_roadmap_id);
      rosparam_shortcuts::get(LOGNAME, nh_, group_name + "/roadmaps", group_roadmap_ids);

      // check if default roadmap is set
      if (!config.default_roadmap_id.empty())
        config.roadmap_ids.insert(config.default_roadmap_id);
      else
        ROS_WARN_STREAM("No default roadmap specified for group " << group_name);

      // add specified roadmap names
      config.roadmap_ids.insert(group_roadmap_ids.begin(), group_roadmap_ids.end());

      // leave out group if no roadmap was found
      if (config.roadmap_ids.empty())
      {
        ROS_INFO_STREAM("Leaving out group " << group_name << ", no roadmaps are specified in the config file.");
        continue;
      }

      // add new roadmap ids
      roadmap_ids.insert(config.roadmap_ids.begin(), config.roadmap_ids.end());

      // add group config
      group_configs_[group_name] = config;
    }

    // load default configs
    std::string default_roadmaps_package;
    std::string default_roadmaps_directory;
    rosparam_shortcuts::get(LOGNAME, nh_, "default/roadmaps_package", default_roadmaps_package);
    rosparam_shortcuts::get(LOGNAME, nh_, "default/roadmaps_directory", default_roadmaps_directory);
    if (ros::package::getPath(default_roadmaps_package).empty())
      ROS_WARN_STREAM_NAMED(LOGNAME, "Unable to find default roadmaps package '" << default_roadmaps_package << "'");

    // load roadmap specifications
    roadmaps_.clear();
    for (const std::string& roadmap_id : roadmap_ids)
    {
      // look for non-default parameters
      std::string roadmap_config = "roadmaps/" + roadmap_id;
      std::string package = nh_.param(roadmap_config + "/package", default_roadmaps_package);
      std::string directory = nh_.param(roadmap_config + "/directory", default_roadmaps_directory);
      std::string filename = nh_.param(roadmap_config + "/filename", roadmap_id);

      // skip roadmap if package is invalid
      std::string package_path = ros::package::getPath(package);
      if (package_path.empty())
      {
        std::string skip_roadmap_message = "Skipping roadmap '" + roadmap_id + "': ";
        if (package != default_roadmaps_package)
          ROS_WARN_STREAM_NAMED(LOGNAME, skip_roadmap_message << "invalid package '" << package << "'");
        else
          ROS_WARN_STREAM_NAMED(LOGNAME, skip_roadmap_message << "invalid default package");
        continue;
      }

      // resolve roadmap file
      boost::filesystem::path roadmap_file(package_path);
      roadmap_file.append(directory);
      roadmap_file.append(filename);
      if (!roadmap_file.has_extension())
        roadmap_file.replace_extension(".og");

      // check if file exists
      if (!boost::filesystem::exists(roadmap_file))
      {
        ROS_WARN_STREAM_NAMED(LOGNAME, "Unable to locate file for roadmap '" << roadmap_id << "' at: " << roadmap_file);
        continue;
      }

      // add new roadmap spec
      RoadmapSpecification spec;
      spec.roadmap_id = roadmap_id;
      spec.files.occupancy = roadmap_file.c_str();
      roadmaps_[roadmap_id] = spec;
      ROS_INFO_STREAM_NAMED(LOGNAME, "Found roadmap '" << roadmap_id << "' at: " << roadmap_file);
    }
  }

  /** \brief Returns a configured planning context for a given planning scene and motion plan request */
  planning_interface::PlanningContextPtr getPlanningContext(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                            const planning_interface::MotionPlanRequest& req,
                                                            moveit_msgs::MoveItErrorCodes& error_code) const
  {
    // TODO(henningkayser): retrieve group config and roadmap specs and pass them to the planning context
    RTRPlanningContext context("rtr_planning_context", req.group_name, planner_interface_);
    context.setMotionPlanRequest(req);
    context.setPlanningScene(planning_scene);
    return std::make_shared<RTRPlanningContext>(context);
  }

private:
  ros::NodeHandle nh_;

  // The RapidPlan wrapper interface
  RTRPlannerInterfacePtr planner_interface_;

  // group and roadmap configurations
  std::map<std::string, GroupConfig> group_configs_;
  std::map<std::string, RoadmapSpecification> roadmaps_;
};
}  // namespace rtr_moveit

PLUGINLIB_EXPORT_CLASS(rtr_moveit::RTRPlannerManager, planning_interface::PlannerManager);
