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

#include <rtrapi/PathPlanner.h>

namespace rtr
{
PathPlanner::PathPlanner(){};

// prevent any copying
PathPlanner::PathPlanner(const PathPlanner& r) : PathPlanner(){};
PathPlanner& PathPlanner::operator=(const PathPlanner& r)
{
  return *this;
};
PathPlanner::~PathPlanner(){};

bool PathPlanner::LoadRoadmap(const std::string& config_fn, const std::string& edge_fn, const std::string& transform_fn)
{
  return true;
};

bool PathPlanner::SetEdgeCost(PtrEdgeCostFunc f)
{
  return true;
};

int PathPlanner::FindPath(const uint& start, const Transform& destination, const std::vector<uint8_t>& collisions,
                          const Transform& tol, const Transform& weights, std::deque<uint>& path_waypoints,
                          std::deque<uint>& path_edges, float timeout) const
{
  return 0;
};

int PathPlanner::FindPath(const uint& start, const std::vector<uint>& destinations,
                          const std::vector<uint8_t>& collisions, std::deque<uint>& path_waypoints,
                          std::deque<uint>& path_edges, float timeout) const
{
  return 0;
};

std::string PathPlanner::GetError(const int errcode)
{
  return "ERROR";
};

uint PathPlanner::GetNumConfigs() const
{
  return 0;
};

uint PathPlanner::GetNumTransforms() const
{
  return 0;
};

uint PathPlanner::GetNumEdges() const
{
  return 0;
};

std::vector<Config> PathPlanner::GetConfigs() const
{
  return {};
};

std::vector<Edge> PathPlanner::GetEdges() const
{
  return {};
};

std::vector<Transform> PathPlanner::GetTransforms() const {};
}
