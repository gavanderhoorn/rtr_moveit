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
 * Desc: Generation of occupancy data from pcl or planning scenes
 */

#ifndef RTR_MOVEIT_OCCUPANCY_HANDLER_H
#define RTR_MOVEIT_OCCUPANCY_HANDLER_H

// C++ synchronization
#include <mutex>
#include <condition_variable>

// PCL
#include <pcl/pcl_base.h>

// MoveIt!
#include <moveit/planning_scene/planning_scene.h>

// rtr_moveit
#include <rtr_moveit/rtr_datatypes.h>

namespace rtr_moveit
{
class OccupancyHandler
{
public:
  /* @brief Constructor */
  OccupancyHandler();

  /* @brief Constructor
   * @param nh - The node handle
   */
  OccupancyHandler(const ros::NodeHandle& nh);

  /* @brief Constructor
   * @param nh - The node handle
   * @param pcl_topic - The pcl topic for point cloud data queries
   */
  OccupancyHandler(const ros::NodeHandle& nh, const std::string& pcl_topic);

  /* Enables/disables Marker publishers for result visualization
   * @param enabled - condition if visualization should be enabled
   */
  void setVisualizationEnabled(bool enabled);

  /* @brief Set the volume region for all occupancy data queries
   * @param  roadmap_volume  - The new region
   */
  void setVolumeRegion(const RoadmapVolume& roadmap_volume);

  /* @brief Set the point cloud topic for all pcl queries
   * @param  pcl_topic  - The new pcl topic
   */
  void setPointCloudTopic(const std::string& pcl_topic);

  /* @brief Initializes occupancy_data with an updated point cloud
   * @param  occupancy_data  - the result data including the point cloud
   * @return true on success
   */
  bool fromPCL(OccupancyData& occupancy_data);

  /* @brief Generates a list of occupancy voxels given a planning scene
   * @param  planning_scene  - the planning scene
   * @param  occupancy_data  - the result data including the voxels
   * @return true on success
   */
  bool fromPlanningScene(const planning_scene::PlanningSceneConstPtr& planning_scene,
                         OccupancyData& occupancy_data);
private:
  /* Visualizes the origin of the region volume and voxels positions
   * @param  frame_id  - the reference frame of the roadmap volume pose
   * @param  volume_pose - the origin pose of the region volume
   * @param  voxel_points - the position vector of all occupancy voxels
   * @param  voxel_dimensions - the voxel dimensions along the coordinate axes
   */
  void visualizeVoxels(const std::string& frame_id,  const geometry_msgs::Pose& volume_origin_pose,
                       const geometry_msgs::Pose& volume_center_pose, const std::array<float, 3> volume_dimensions,
                       const std::vector<geometry_msgs::Point>& voxel_points,
                       const std::array<float, 3> voxel_dimensions);

  /* Callback function for point cloud subscribers
   * @param  cloud_pcl2 - the pointer of a new sensed point cloud
   */
  void pclCallback(const pcl::PCLPointCloud2ConstPtr& cloud_pcl2);

  ros::NodeHandle nh_;
  RoadmapVolume volume_region_;
  std::string pcl_topic_;

  // PCL synchronization
  pcl::PointCloud<pcl::PointXYZ>::Ptr shared_pcl_ptr_;
  std::mutex pcl_mtx_;
  std::condition_variable pcl_condition_;
  bool pcl_ready_ = false;

  // visualization
  bool visualize_occupancy_data_ = false;
  ros::Publisher volume_pub_;
};
}  // namespace rtr_moveit

#endif  // RTR_MOVEIT_OCCUPANCY_HANDLER_H
