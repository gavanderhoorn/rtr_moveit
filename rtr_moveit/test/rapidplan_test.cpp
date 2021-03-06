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

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <rtr_moveit/rtr_planner_interface.h>
#include <rtr_moveit/roadmap_search.h>

TEST(TestSuite, testPlannerInterface)
{
  ros::NodeHandle nh;

  // roadmap spec without file
  rtr_moveit::RoadmapSpecification roadmap;
  roadmap.roadmap_id = "test_roadmap";

  // valid start
  const std::size_t start_id = 0;
  const std::size_t goal_id = 10;

  // valid goal
  rtr_moveit::RapidPlanGoal goal;
  goal.type = rtr_moveit::RapidPlanGoal::Type::STATE_IDS;
  goal.state_ids = { goal_id };

  // planner setup
  rtr_moveit::RTRPlannerInterface planner_(nh);
  double timeout = 5;  // seconds
  rtr_moveit::OccupancyData occupancy_dummy;
  occupancy_dummy.type = rtr_moveit::OccupancyData::Type::VOXELS;
  std::vector<std::vector<float>> solution;
  EXPECT_FALSE(planner_.solve(roadmap, start_id, goal, occupancy_dummy, timeout, solution)) << "Planning should not "
                                                                                               "work without a roadmap";
  solution.clear();

  // add roadmap file
  roadmap.roadmap_id = "test_roadmap_2";
  roadmap.og_file = ros::package::getPath("rtr_moveit") + "/test/test_roadmap.og";

  // this should work now
  std::vector<std::vector<float>> roadmap_states;
  std::deque<std::size_t> waypoints, edges;
  ASSERT_TRUE(planner_.solve(roadmap, start_id, goal, occupancy_dummy, timeout, roadmap_states, waypoints, edges))
      << "Planning with STATE_IDS goal should have been successful";

  // then it should work backwards as well
  goal.state_ids = { start_id };
  ASSERT_TRUE(planner_.solve(roadmap, goal_id, goal, occupancy_dummy, timeout, solution))
      << "Planning with JOINT_STATE goal should have been successful";

  ASSERT_FALSE(solution.empty()) << "Solution path is empty";

  // TODO(RTR-58): add TRANSFORM goal test

  // test state search
  EXPECT_TRUE(rtr_moveit::findClosestConfigId(roadmap_states[start_id], roadmap_states) == start_id);
  EXPECT_TRUE(rtr_moveit::findClosestConfigId(roadmap_states[goal_id], roadmap_states) == goal_id);
  EXPECT_FALSE(rtr_moveit::findClosestConfigId(roadmap_states[goal_id], roadmap_states) == start_id);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "rapidplan_test");
  return RUN_ALL_TESTS();
}
