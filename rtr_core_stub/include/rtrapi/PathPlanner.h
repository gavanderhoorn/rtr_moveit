/* Copyright (C) Realtime Robotics, Inc - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef RTRAPI_PATHPLANNER_H
#define RTRAPI_PATHPLANNER_H

#include <boost/geometry.hpp>
#include <chrono>
#include <deque>
#include <map>
#include <queue>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

namespace rtr {

/*!***********************************************************************
 * Configuration struct contains a single robot pose, defining a value in
 * radians for each moveable joint.
 *************************************************************************/
typedef std::vector<float> Config;

/*!***********************************************************************
 * Edge struct indicates undirected connection between two configuration
 * indices.
 *************************************************************************/
typedef std::array<uint, 2> Edge;

/*!***********************************************************************
 * Forward kinematics of a specific pose. Order is: {x, y, z, r, p, y}
 * x, y, z in meters, r, p, y in radians
 *************************************************************************/
typedef std::array<float, 6> Transform;

/*!***********************************************************************
 * Function pointer type that users of this class must provide.
 * This function will be used to calculate the cost of moving between two
 * configurations.
 * return val assumed range: [0, +infinity)
 *************************************************************************/
typedef float (*PtrEdgeCostFunc)(const Config& t1, const Config& t2);

// TODO: remove this typedef
typedef unsigned int uint;
class PathPlannerPrivate;

/*!***********************************************************************
 * Class to perform the runtime operation of finding a path from a
 * starting config to a goal.
 *
 * User first calls the LoadRoadmap method to populate the class with
 * needed config, edge and tranform data. The transform data is generated
 * by the Sprint Configuration Toolkit. The formats of the config
 * and edge files are discussed in the User Manual, and examples are
 * generated in the Roadmap Generation Tutorial.
 *
 * After LoadRoadmap(), the user must supply an implementation of a edge
 * traversal cost function via a call to the SetEdgeCost() method. This is
 * used in the path search functionality to appropriately weight the edges.
 *
 * After calling LoadRoadmap() and SetEdgeCost(), FindPath() can be called
 * repeatedly to obtain paths. There are two versions of FindPath().
 *
 * The first version takes a goal pose in cartesian space (x, y, z, r, p, y)
 * along with associated tolerances.
 *
 * The second version takes a goal configuration index, and will only return
 * paths to that specific index. This is useful in bringing a robot to an
 * exact desired waypoint as part of a complex task, or returning to a home
 * position in between cycles.
 *************************************************************************/
class PathPlanner {
 public:
  PathPlanner();
  virtual ~PathPlanner();

  /*!************************************************************************
   * Loads the specified roadmap files.
   * returns True for success, False otherwise
   *************************************************************************/
  bool LoadRoadmap(const std::string& config_fn, const std::string& edge_fn,
                   const std::string& transform_fn);

  /*!************************************************************************
   * Set the edge cost function. The function provided should have the
   * signature float(const Config&, const Config&)
   *
   * LoadRoadmap() must have been successfully called previously.
   * Returns True on success, False otherwise
   *************************************************************************/
  bool SetEdgeCost(PtrEdgeCostFunc f);

  /*!************************************************************************
   * Function to return a path from a specified starting configuration
   * to a desired location. Arguments:
   *
   * start: starting configuration index.
   *
   * destination: desired pose of the end effector-link with respect to
   *              the base-link. Both these links are defined in the
   *              Configuration Toolkit
   *
   * collisions: vector of length Num_Edges indicating if each corresponding
   *             edge is safe to use in the current scene. This vector is
   *             generated by a call to CheckScene() with a HardwareAPI Object
   *
   * tolerance: Acceptable deviation in each transform dimension for a
   *            configuration to be considered a valid goal. Node i is
   *            rejected as a goal if:
   *            abs(transforms[i][j] - destination[j]) > tol[j]
   *            for j = [0, 5]
   *
   * weights: A 6-tuple of dimensionless floats. If multiple configurations
   *          meet the specified tolerances, this is used to rank potential
   *          goals. A simple weighted linear combination of deviation is
   *          used. As an example, if x, y, z, r, p all have the same
   *          importance to the application, but yaw is irrelevant then
   *          the tuple {1, 1, 1, 1, 1, 0} could be provided.
   *
   * path_waypoints: The resulting path in the form of a list of
   *                 configuration indices.
   *
   * path_edges: The resulting path in the form of a list of edge indices
   *
   * timeout:  optional time in milliseconds to cutoff a search
   *
   * returns: 0: Success
   *         -1: Collision vector not correct size
   *         -2: No configurations in roadmap meet tolerances
   *         -3: Due to collisions, no path could be found
   *************************************************************************/
  int FindPath(const uint& start, const Transform& destination,
               const std::vector<uint8_t>& collisions, const Transform& tol,
               const Transform& weights, std::deque<uint>& path_waypoints,
               std::deque<uint>& path_edges,
               float timeout = std::numeric_limits<float>::max()) const;

  /*!***********************************************************************
   * Similar to the above version, but simpler. Instead of representing the
   * goal as an end effector pose, a vector of acceptable configuration
   * indices are explicitly provided.
   *
   * This is useful in bringing a robot to an exact desired waypoint as part
   * of a complex task, or returning to a home position in between cycles.
   *
   * returns: 0: Success
   *         -3: Due to collisions, no path could be found
   *************************************************************************/
  int FindPath(const uint& start, const std::vector<uint>& destinations,
               const std::vector<uint8_t>& collisions, std::deque<uint>& path_waypoints,
               std::deque<uint>& path_edges,
               float timeout = std::numeric_limits<float>::max()) const;

  /*!***********************************************************************
   * Provides a string description of the error code provided by FindPath
   *************************************************************************/
  std::string GetError(const int errcode);

  /*!***********************************************************************
   * Get the number of robot configurations in the roadmap.
   *************************************************************************/
  uint GetNumConfigs() const;

  /*!***********************************************************************
   * Get the number of transforms loaded up. Should equal the number of
   * configurations.
   *************************************************************************/
  uint GetNumTransforms() const;

  /*!***********************************************************************
   * Get the number of edges in the roadmap.
   *************************************************************************/
  uint GetNumEdges() const;

  /*!***********************************************************************
   * Get the configs that were loaded previously.
   *************************************************************************/
  std::vector<Config> GetConfigs() const;

  /*!***********************************************************************
   * Get the edges that were loaded previously.
   *************************************************************************/
  std::vector<Edge> GetEdges() const;

  /*!***********************************************************************
   * Get the transforms that were loaded previously.
   *************************************************************************/
  std::vector<Transform> GetTransforms() const;

 private:
  // prevent any copying
  PathPlanner(const PathPlanner& r);
  PathPlanner& operator=(const PathPlanner& r);
  PathPlannerPrivate* planner_;
};
}  // namespace rtr

#endif  // RTRAPI_PATHPLANNER_H
