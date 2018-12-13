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

#include <rtrapi/HardwareInterface.h>

namespace rtr
{
HardwareInterface::HardwareInterface(){};
HardwareInterface::~HardwareInterface(){};

bool HardwareInterface::Init()
{
  return true;
};

bool HardwareInterface::WriteRoadmap(const std::string& og_fname, uint16_t& roadmap_index)
{
  roadmap_index = 0;
  return true;
};

bool HardwareInterface::HardwareInterface::PopRoadmap()
{
  return true;
};

bool HardwareInterface::HardwareInterface::ClearRoadmaps()
{
  return true;
};

bool HardwareInterface::HardwareInterface::NumRoadmaps(uint16_t& num_roadmaps) const
{
  num_roadmaps = 1;
  return true;
};

bool HardwareInterface::CheckScene(pcl::PointCloud<pcl::PointXYZ>::ConstPtr scene_cloud, uint16_t roadmap_index,
                                   std::vector<uint8_t>& collisions) const
{
  return true;
};

bool HardwareInterface::CheckScene(const std::vector<Voxel>& scene_voxels, uint16_t roadmap_index,
                                   std::vector<uint8_t>& collisions) const
{
  return true;
};

bool HardwareInterface::CheckScene(const std::vector<Box>& scene_boxes, uint16_t roadmap_index,
                                   std::vector<uint8_t>& collisions) const
{
  return true;
};

bool HardwareInterface::Connected() const
{
  return true;
};

bool HardwareInterface::Handshake() const
{
  return true;
};

bool HardwareInterface::Test()
{
  return true;
};

bool HardwareInterface::SaveSnapshot(const std::string& fname, pcl::PointCloud<pcl::PointXYZ>::ConstPtr scene_cloud,
                                     unsigned int roadmap_index) const
{
  return true;
};

bool HardwareInterface::SaveSnapshot(const std::string& fname, const std::vector<Voxel>& scene_voxels,
                                     unsigned int roadmap_index) const
{
  return true;
};

bool HardwareInterface::SaveSnapshot(const std::string& fname, const std::vector<Box>& scene_boxes,
                                     unsigned int roadmap_index) const
{
  return true;
};
}
