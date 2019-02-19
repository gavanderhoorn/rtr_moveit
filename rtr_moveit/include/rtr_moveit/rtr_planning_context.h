/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, PickNik LLC
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
 * Desc: A planning context that handles MotionPlanRequests and runs planning attempts using the RTRPlanningInterface.
 */

#ifndef RTR_MOVEIT_RTR_PLANNING_CONTEXT_H
#define RTR_MOVEIT_RTR_PLANNING_CONTEXT_H

// C++
#include <string>

// MoveIt
#include <moveit/macros/class_forward.h>
#include <moveit/planning_interface/planning_interface.h>

// rtr_moveit
#include <rtr_moveit/rtr_planner_interface.h>
#include <rtr_moveit/rtr_datatypes.h>

namespace rtr_moveit
{
MOVEIT_CLASS_FORWARD(RTRPlanningContext);
MOVEIT_CLASS_FORWARD(RTRPlannerInterface);

class RTRPlanningContext : public planning_interface::PlanningContext
{
public:
  /** Constructor
   * @param planning_group - The name of the joint model group
   * @param roadmap_spec - Roadmap and region volume configuration for this context
   * @param planner_interface - The RTRPlannerInterface that handles RapidPlan collision checks and roadmap planning
   */
  RTRPlanningContext(const std::string& planning_group, const RoadmapSpecification& roadmap_spec,
                     const RTRPlannerInterfacePtr& planner_interface);

  /** Destructor */
  virtual ~RTRPlanningContext()
  {
  }

  /** Runs a planning attempt on the configured context and stores results in a MotionPlanResponse
   * @param  res - The MotionPlanResponse containing result code, solution trajectory and planning time
   * @return true on success
   */
  virtual bool solve(planning_interface::MotionPlanResponse& res);

  /** Runs a planning attempt on the configured context and stores results in a MotionPlanDetailedResponse
   * @param  res - The MotionPlanDetailedResponse containing result code, solution trajectory and descriptions
   *               and planning times of all planning steps
   * @return true on success
   */
  virtual bool solve(planning_interface::MotionPlanDetailedResponse& res);

  /** Configures the planning context which includes extracting a goal from the MotionPlanRequest
   * @param error_code - the result code
   */
  void configure(moveit_msgs::MoveItErrorCodes& error_code);

  /** Clear the planning context data */
  virtual void clear();

  /** Terminate the planning context - NOTE: this is not supported */
  virtual bool terminate();

private:
  /** Runs a planning attempt on the configured context and initializes results as RobotTrajectory and planning time
   * @param  trajectory - the result RobotTrajectory
   * @param  planning_time - the elapsed planning time
   * @return a MoveItErrorCodes message
   */
  moveit_msgs::MoveItErrorCodes solve(robot_trajectory::RobotTrajectoryPtr& trajectory, double& planning_time);

  /** Converts the given goal Constraints vector to a vector of valid RapidPlanGoals that can be used with the
   *  RTRPlannerInterface. Failed Constraints are left out of the result vector.
   * @param  goal_constraints - the Constraints vector
   * @param  goal - the RapidPlanGoal result vector
   * @return true on success, false if no RapidPlanGoals could be extracted from non-empty goal_constraints
   */
  bool initRapidPlanGoals(const std::vector<moveit_msgs::Constraints>& goal_constraints,
                          std::vector<RapidPlanGoal>& goals);

  /** Converts the given goal constraints to a valid RapidPlanGoal that can be used with the RTRPlannerInterface.
   *  If the RapidPlanGoal does not fully meet the constraints, the robot state goal_state is initialized as the
   *  actual goal.
   * @param  goal_constraints - the goal constraints to extract
   * @param  goal - the returned RapidPlanGoal
   * @param  goal_state - the robot state that fulfills the constraints in case the RapidPlanGoal doesn't
   * @return true on success
   */
  bool getRapidPlanGoal(const moveit_msgs::Constraints& goal_constraint, RapidPlanGoal& goal,
                        robot_state::RobotStatePtr& goal_state);

  /** Initializes start_config from the MotionPlanRequest. If the joint values in the MotionPlanRequest
   *  are not populated, the current state of the planning scene is used is used.
   *  @param start_config - the returned joint config
   *  @return true on success, false if the joint state inside the MotionPlanRequest is invalid
   */
  bool initStartState(rtr::Config& start_config);

  robot_state::RobotStatePtr start_state_;
  std::vector<robot_state::RobotStatePtr> goal_states_;

  const RTRPlannerInterfacePtr planner_interface_;
  const moveit::core::JointModelGroup* jmg_;
  std::vector<std::string> joint_model_names_;
  RoadmapSpecification roadmap_;
  std::vector<rtr::Config> roadmap_configs_;
  std::vector<rtr::ToolPose> roadmap_poses_;
  std::vector<RapidPlanGoal> goals_;
  bool configured_ = false;

  // parameters
  double allowed_joint_distance_;
  double allowed_position_distance_;
  int max_goal_states_;

  ros::Time terminate_plan_time_;
};
}  // namespace rtr_moveit

#endif  // RTR_MOVEIT_RTR_PLANNING_CONTEXT_H
