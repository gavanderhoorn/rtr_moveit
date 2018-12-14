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
#include <vector>

#include <Eigen/Geometry>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <moveit/planning_scene/planning_scene.h>
#include <geometric_shapes/shapes.h>

#include <moveit/collision_detection/world.h>
#include <moveit/collision_detection_fcl/collision_world_fcl.h>

namespace rtr_moveit
{
static inline void poseMsgToRtr(const geometry_msgs::Pose& pose, std::array<float, 6>& rtr_transform)
{
  // set position x/y/z
  rtr_transform[0] = pose.position.x;
  rtr_transform[1] = pose.position.y;
  rtr_transform[2] = pose.position.z;

  // set orientation roll pitch yaw
  Eigen::Quaterniond rotation(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  Eigen::Vector3d euler_angles = rotation.toRotationMatrix().eulerAngles(0, 1, 2);
  rtr_transform[3] = euler_angles[0];
  rtr_transform[4] = euler_angles[1];
  rtr_transform[5] = euler_angles[2];
}

static inline void poseRtrToMsg(const std::array<float, 6>& rtr_transform, geometry_msgs::Pose& pose)
{
  pose.position.x = rtr_transform[0];
  pose.position.y = rtr_transform[1];
  pose.position.z = rtr_transform[2];
  tf::Quaternion rotation = tf::createQuaternionFromRPY(rtr_transform[3], rtr_transform[4], rtr_transform[5]);
  tf::quaternionTFToMsg(rotation, pose.orientation);
}

static inline void pathRtrToJointTrajectory(const std::vector<std::vector<float>>& roadmap_states,
                                            const std::deque<unsigned int>& path_indices,
                                            trajectory_msgs::JointTrajectory& trajectory)
{
  trajectory.points.resize(path_indices.size());
  int joints_num = roadmap_states[0].size();
  for (auto i : path_indices)
  {
    // fill joint values
    trajectory.points[i].positions.resize(joints_num);
    for (int j = 0; j < joints_num; j++)
      trajectory.points[i].positions[j] = roadmap_states[i][j];

    // TODO(henningkayser@picknik.ai): Initialize velocities, accelerations, effort, time_from-start
  }
}

/* \brief Generates a list of occupancy boxes given a planning scene and target volume region */
static inline void planningSceneToRtrCollisionBoxes(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                    const RoadmapVolume& volume,
                                                    std::vector<rtr::Box> scene_boxes)
{
  // occupancy box id and dimensions
  // TODO(henningkayser): Check that box id is not present in planning scene - should be unique
  std::string box_id = "rapidplan_collision_box";
  // TODO(henningkayser): parameterize box dimensions or generate from volume with accuracy parameter
  double box_x = 0.02;
  double box_y = 0.02;
  double box_z = 0.02;

  // Compute transform: world->volume
  // world_to_volume points at the corner of the volume with minimal x,y,z
  Eigen::Isometry3d world_to_base(planning_scene->getFrameTransform(volume.base_frame));
  Eigen::Translation3d base_to_volume(volume.center.x - 0.5 * volume.dimensions.size[0],
                                      volume.center.y - 0.5 * volume.dimensions.size[1],
                                      volume.center.z - 0.5 * volume.dimensions.size[2]);
  Eigen::Isometry3d world_to_volume = world_to_base * base_to_volume;

  // create collision world and add box shape
  collision_detection::CollisionWorldFCL world;
  shapes::Box box(box_x, box_y, box_z);
  world.getWorld()->addToObject(box_id, std::make_shared<const shapes::Box>(box), world_to_volume);

  // extract collision world from planning scene
  collision_detection::CollisionWorldConstPtr collision_world = planning_scene->getCollisionWorld();

  // collision request and result
  collision_detection::CollisionRequest request;
  collision_detection::CollisionResult result;

  // clear scene boxes vector
  scene_boxes.resize(0);

  // Loop over X/Y/Z coordinates and check for box collisions in the collision world
  // TODO(henningkayser): adjust grid to odd volume dimensions
  // TODO(henningkayser): More efficient implementations:
  //                          * Iterate over collision objects and only sample local bounding boxes
  //                          * Use octree search, since boxes can have variable sizes
  // TODO(henningkayser): Do we need extra Box padding here?
  for (double x = 0.0; x < volume.dimensions.size[0]; x += box_x)
    for (double y = 0.0; y < volume.dimensions.size[1]; y += box_y)
      for (double z = 0.0; z < volume.dimensions.size[2]; z += box_z)
      {
        world.getWorld()->moveObject(box_id, world_to_volume * Eigen::Translation3d(x, y, z));
        collision_world->checkWorldCollision(request, result, world);
        if (result.collision)
          scene_boxes.push_back(rtr::Box(x, y, z, x + box_x, y + box_y, z + box_z));
        result.clear();
      }
}
}  // namespace rtr_moveit

#endif  // RTR_MOVEIT_RTR_CONVERSIONS_H
