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
 * Desc: Helper functions for searching and sorting roadmap data
 */

#ifndef RTR_MOVEIT_ROADMAP_SEARCH_H
#define RTR_MOVEIT_ROADMAP_SEARCH_H

#include <string>
#include <vector>

#include <rtr-api/PathPlanner.hpp> // contains rtr::Config, rtr::ToolPose

namespace rtr_moveit
{
namespace
{
/** Compute the absolute distance between two joint state configurations
 * @param first, second - The pair of joint states as Config types
 * @return - The absolute joint state distance between first and second
 */
float getConfigDistance(const rtr::Config& first, const rtr::Config& second)
{
  float distance = 0.0;
  for (unsigned int i = 0; i < first.size(); ++i)
    distance += std::abs(first[i] - second[i]);
  return distance;
}

/** Compute the absolute position distance between two tool poses
 * @param first, second - The pair of tool poses as rtr::ToolPose types
 * @return - The absolute tool pose distance between first and second
 */
float getPositionDistance(const rtr::ToolPose& first, const rtr::ToolPose& second)
{
  float distance = 0.0;
  for (unsigned int i = 0; i < 3; ++i)
    distance += std::pow(first[i] - second[i], 2);
  return std::sqrt(distance);
}

/** Template definition of a distance function between two items of tye T
 * @param first, second - The pair of items of type T
 * @return - The absolute distance between first and second
 */
template <class T>
float getDistance(const T&, const T&);

/** Template implementation for computing the absolute distance between two joint state configurations
 * @param first, second - The pair of joint states as Config types
 * @return - The absolute joint state distance between first and second
 */
template <>
float getDistance<rtr::Config>(const rtr::Config& first, const rtr::Config& second)
{
  return getConfigDistance(first, second);
}

/** Template implementation for Computing the absolute position distance between two tool poses
 * @param first, second - The pair of tool poses as rtr::ToolPose types
 * @return - The absolute tool pose distance between first and second
 */
template <>
float getDistance<rtr::ToolPose>(const rtr::ToolPose& first, const rtr::ToolPose& second)
{
  return getPositionDistance(first, second);
}

/** Template implementation of a distance-based search of item type T using getDistance<T>(T, T).
* Find indices and distances of n closest items within a position distance threshold to a given item.
* @param item - The item to compare
* @param item - The list of items to search in
* @param result_ids - The indices of the closest items with distances in increasing order
* @param result_distances - The distances of the result items in increasing order
* @param max_results - The maximum size of the result set
* @param distance_threshold - The allowed distance of result items from item
*/
template <class T>
void findClosest(const T& item, const std::vector<T>& items,
                  std::vector<unsigned int>& result_ids, std::vector<float>& result_distances,
                  const unsigned int max_results = 1, const float& distance_threshold = FLT_MAX)
{
  result_ids.clear();
  result_distances.clear();
  if (!items.empty() && max_results != 0 && distance_threshold > 0.0)
  {
    // iterate items
    for (unsigned int item_id = 0; item_id < items.size(); ++item_id)
    {
      float distance = getDistance<T>(item, items[item_id]);
      int insert_position = 0;
      bool add_to_results = result_ids.empty(); // if empty, we don't need to compare
      if (!add_to_results)
      {
        for (insert_position = result_distances.size(); insert_position > 0; insert_position--)
        {
          if (distance < result_distances[insert_position - 1])
            add_to_results = true;
          else
            break;
        }
      }
      // add to results
      if (add_to_results && distance < distance_threshold)
      {
        result_distances.insert(result_distances.begin() + insert_position, distance);
        result_ids.insert(result_ids.begin() + insert_position, item_id);
        if (result_distances.size() > max_results)
        {
          result_ids.pop_back();
          result_distances.pop_back();
        }
      }
    }
  }
}

/** Find indices and distances of n closest tool poses within a position distance threshold to a given pose.
* @param pose - The tool pose to compare
* @param poses - The list of tool poses to search in
* @param result_ids - The indices of the closest poses with distances in increasing order
* @param result_distances - The distances of the result poses in increasing order
* @param max_results - The maximum size of the result set
* @param distance_threshold - The allowed distance of result poses from pose
*/
void findClosestPositions(const rtr::ToolPose& pose, const std::vector<rtr::ToolPose>& poses,
                          std::vector<unsigned int>& result_ids, std::vector<float>& result_distances,
                          const unsigned int max_results = 1, const float& distance_threshold = FLT_MAX)
{
  findClosest<rtr::ToolPose>(pose, poses, result_ids, result_distances, max_results, distance_threshold);
}

/** Find indices and distances of n closest tool poses within a position distance threshold to a given pose.
* @param pose - The tool pose to compare
* @param poses - The list of tool poses to search in
* @param result_ids - The indices of the closest poses with distances in increasing order
* @param max_results - The maximum size of the result set
* @param distance_threshold - The allowed distance of result poses from pose
*/
void findClosestPositions(const rtr::ToolPose& pose, const std::vector<rtr::ToolPose>& poses,
                          std::vector<unsigned int>& result_ids, const unsigned int max_results = 1,
                          const float& distance_threshold = FLT_MAX)
{
  std::vector<float> result_distances;
  findClosest<rtr::ToolPose>(pose, poses, result_ids, result_distances, max_results, distance_threshold);
}

/** Find indices and distances of n closest elements in configs within a distance threshold to a given joint config.
*   If dimension of config and elements in configs dont fit, result_ids and result_distances are empty.
* @param config - The joint state config to compare
* @param configs - The list of configs to search in
* @param result_ids - The indices of the closest configs with distances in increasing order
* @param result_distances - The distances of the result configs in increasing order
* @param max_results - The maximum size of the result set
* @param distance_threshold - The allowed distance of result elements from config
*/
void findClosestConfigs(const rtr::Config& config, const std::vector<rtr::Config>& configs,
                        std::vector<unsigned int>& result_ids, std::vector<float>& result_distances,
                        const unsigned int max_results = 1, const float& distance_threshold = FLT_MAX)
{
  findClosest<rtr::Config>(config, configs, result_ids, result_distances, max_results, distance_threshold);
}

/** Find indices and distances of all elements in configs within a distance threshold to a given joint config
*   If dimension of config and elements in configs dont fit, result_ids and result_distances are empty.
* @param config - The joint state config to compare
* @param configs - The list of configs to search in
* @param result_ids - The indices of the closest configs with distances in increasing order
* @param distance_threshold - The allowed distance of result elements from config
*/
void findClosestConfigs(const rtr::Config& config, const std::vector<rtr::Config>& configs,
                        std::vector<unsigned int>& result_ids, std::vector<float>& result_distances,
                        const float& distance_threshold = FLT_MAX)
{
  findClosestConfigs(config, configs, result_ids, result_distances, configs.size(), distance_threshold);
}

/** Find indices and distances of all elements in configs within a distance threshold to a given joint config
*   If dimension of config and elements in configs dont fit, result_ids and result_distances are empty.
* @param config - The joint state config to compare
* @param configs - The list of configs to search in
* @param result_ids - The indices of the closest configs with distances in increasing order
* @param result_distances - The distances of the result configs in increasing order
* @param distance_threshold - The allowed distance of result elements from config
*/
void findClosestConfigs(const rtr::Config& config, const std::vector<rtr::Config>& configs,
                        std::vector<unsigned int>& result_ids, const float& distance_threshold = FLT_MAX)
{
  std::vector<float> result_distances;
  findClosestConfigs(config, configs, result_ids, result_distances, configs.size(), distance_threshold);
}


/** Find and return the index of the element in configs with the minimal distance to a given joint state config
 * @param config - The joint state config to compare
 * @param configs - The list of configs to search in
 * @return - The index of the closest element in configs, -1 if configs is empty or config sizes don't match
 */
unsigned int findClosestConfigId(const rtr::Config& config, const std::vector<rtr::Config>& configs,
                                 const float& distance_threshold = FLT_MAX)
{
  std::vector<unsigned int> result_ids;
  std::vector<float> result_distances;
  findClosestConfigs(config, configs, result_ids, result_distances, 1, distance_threshold);
  return result_ids.empty() ? -1 : result_ids[0];
}
}  // namespace
}  // namespace rtr_moveit

#endif  // RTR_MOVEIT_ROADMAP_SEARCH_H
