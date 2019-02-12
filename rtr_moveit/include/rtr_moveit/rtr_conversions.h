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

#ifndef RTR_MOVEIT_RTR_CONVERSIONS_H
#define RTR_MOVEIT_RTR_CONVERSIONS_H

#include <array>
#include <cmath>
#include <deque>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include <ros/ros.h>
#include <ros/assert.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <moveit/planning_scene/planning_scene.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <moveit/robot_state/robot_state.h>
#include <geometric_shapes/shapes.h>

#include <moveit/collision_detection/world.h>
#include <moveit/collision_detection_fcl/collision_world_fcl.h>

#include <rtr_moveit/rtr_datatypes.h>

// RapidPlan
#include <rtr-occupancy/Voxel.hpp>

namespace rtr_moveit
{
namespace
{
inline void poseMsgToRtr(const geometry_msgs::Pose& pose, rtr::Transform& rtr_transform)
{
  // set position x/y/z
  rtr_transform.t[0] = pose.position.x;
  rtr_transform.t[1] = pose.position.y;
  rtr_transform.t[2] = pose.position.z;
  rtr_transform.R.Set(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
}

inline void poseRtrToMsg(const rtr::Transform& rtr_transform, geometry_msgs::Pose& pose)
{
  pose.position.x = rtr_transform.t[0];
  pose.position.y = rtr_transform.t[1];
  pose.position.z = rtr_transform.t[2];
  rtr::Vec3 euler_angles;
  rtr_transform.R.GetEuler(euler_angles);
  tf::Quaternion rotation;
  rotation.setEuler(euler_angles[2], euler_angles[1], euler_angles[0]);
  tf::quaternionTFToMsg(rotation, pose.orientation);
}

inline bool rtrTransformToRtrToolPose(const rtr::Transform& transform, std::array<float, 6>& tool_pose)
{
  rtr::Vec3 euler_angles;
  transform.R.GetEuler(euler_angles);
  tool_pose = { transform.t[0], transform.t[1], transform.t[2], euler_angles[0], euler_angles[1], euler_angles[2] };
}

inline void pathRtrToRobotTrajectory(const std::vector<rtr::Config>& path,
                                     const robot_state::RobotState& reference_state,
                                     const std::vector<std::string>& joint_names,
                                     robot_trajectory::RobotTrajectory& trajectory)
{
  ROS_ASSERT_MSG(joint_names.size() != path[0].size(), "Joint values don't match joint names");
  for (const rtr::Config& joint_state : path)
  {
    robot_state::RobotStatePtr robot_state(new robot_state::RobotState(reference_state));
    for (std::size_t i = 0; i < joint_names.size(); i++)
      robot_state->setJointPositions(joint_names[i], { (double)joint_state[i] });
    trajectory.addSuffixWayPoint(robot_state, 0.1);
  }
}

/* \brief Generates a list of occupancy boxes given a planning scene and target volume region */
inline void planningSceneToRtrCollisionVoxels(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                              const RoadmapVolume& volume, std::vector<rtr::Voxel>& voxels)
{
  // occupancy box id and dimensions
  // TODO(henningkayser): Check that box id is not present in planning scene - should be unique
  std::string box_id = "rapidplan_collision_box";
  double voxel_dimension = volume.voxel_dimension;
  double x_length = volume.dimensions.size[0];
  double y_length = volume.dimensions.size[1];
  double z_length = volume.dimensions.size[2];

  int x_voxels = x_length / voxel_dimension;
  int y_voxels = y_length / voxel_dimension;
  int z_voxels = z_length / voxel_dimension;

  // Compute transform: world->volume
  // world_to_volume points at the corner of the volume with minimal x,y,z
  auto world_to_base(planning_scene->getFrameTransform(volume.base_frame));
  Eigen::Translation3d base_to_volume(volume.center.x - 0.5 * x_length, volume.center.y - 0.5 * y_length,
                                      volume.center.z - 0.5 * z_length);
  auto world_to_volume = world_to_base * base_to_volume;

  // create collision world and add voxel box shape one step outside the volume grid
  collision_detection::CollisionWorldFCL world;
  shapes::Box box(voxel_dimension, voxel_dimension, voxel_dimension);
  double voxel_offset = -0.5 * voxel_dimension;
  world.getWorld()->addToObject(box_id, std::make_shared<const shapes::Box>(box),
                                world_to_volume * Eigen::Translation3d(voxel_offset, voxel_offset, voxel_offset));

  // collision request and result
  collision_detection::CollisionRequest request;
  collision_detection::CollisionResult result;

  // clear scene boxes vector
  voxels.resize(0);

  // x/y/z step transforms
  Eigen::Affine3d x_step(Eigen::Affine3d::Identity() * Eigen::Translation3d(voxel_dimension, 0, 0));
  Eigen::Affine3d y_step(Eigen::Affine3d::Identity() * Eigen::Translation3d(0, voxel_dimension, 0));
  Eigen::Affine3d z_step(Eigen::Affine3d::Identity() * Eigen::Translation3d(0, 0, voxel_dimension));

  // x/y reset transforms
  Eigen::Affine3d y_reset(Eigen::Affine3d::Identity() * Eigen::Translation3d(0, -y_voxels * voxel_dimension, 0));
  Eigen::Affine3d z_reset(Eigen::Affine3d::Identity() * Eigen::Translation3d(0, 0, -z_voxels * voxel_dimension));

  // Loop over X/Y/Z voxel positions and check for box collisions in the collision scene
  // NOTE: This implementation is a prototype and will be replaced by more efficient methods as described below
  // TODO(henningkayser): More efficient implementations:
  //                          * Iterate over collision objects and only sample local bounding boxes
  //                          * Use octree search, since boxes can have variable sizes
  // TODO(henningkayser): adjust grid to odd volume dimensions
  // TODO(henningkayser): Do we need extra Box padding here?
  for (uint16_t x = 0; x < x_voxels; x++)
  {
    world.getWorld()->moveObject(box_id, x_step);
    for (uint16_t y = 0; y < y_voxels; y++)
    {
      world.getWorld()->moveObject(box_id, y_step);
      for (uint16_t z = 0; z < z_voxels; z++)
      {
        world.getWorld()->moveObject(box_id, z_step);
        planning_scene->getCollisionWorld()->checkWorldCollision(request, result, world);
        if (result.collision)
        {
          voxels.push_back(rtr::Voxel(x, y, z));
          result.clear();  // TODO(henningkayser): Is this really necessary?
        }
      }
      // move object back to z start
      world.getWorld()->moveObject(box_id, z_reset);
    }
    // move object back to y start
    world.getWorld()->moveObject(box_id, y_reset);
  }
}
}  // namespace
}  // namespace rtr_moveit

#endif  // RTR_MOVEIT_RTR_CONVERSIONS_H
