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

#ifndef HARDWARE_INTERFACE_H_
#define HARDWARE_INTERFACE_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rtr_occupancy/Box.h>
#include <rtr_occupancy/Voxel.h>
#include <string>
#include <vector>

namespace rtr {
class HardwareInterfacePrivate;

/*!***********************************************************************
 * The class for interfacing with RapidPlan's Motion Planning Accelerator
 ************************************************************************/
class HardwareInterface {
 public:
  HardwareInterface();
  virtual ~HardwareInterface();

  /*!***********************************************************************
   * Initializes the hardware (takes ~2 seconds).
   * Returns true if the hardware is already initialized or if the hardware
   * is initialized successfully.
   * Successful Init() is required before further functionality can be used
   ****************************************************************************/
  bool Init();

  /*!***************************************************************************
   * Loads a roadmap from the given file and writes it to the board. The files
   * given in the first arguments is created by the configuration toolkit.
   * Roadmap_index is set to the index to which this roadmap was written.
   * The roadmap_index returned should be saved and used for later calls to
   * CheckScene. Returns true if the roadmap was successfully written
   ****************************************************************************/
  bool WriteRoadmap(const std::string& og_fname, uint16_t& roadmap_index);

  /*!***************************************************************************
   * Removes the roadmap at the last index.
   * Returns true if successful.
   * Returns false if there are no roadmaps to pop or the board is not connected.
   ****************************************************************************/
  bool PopRoadmap();

  /*!***************************************************************************
   * Removes all of the roadmaps from the board.
   * Returns true if successful
   * Returns false if there are no roadmaps to clear or the board is not connected
   ****************************************************************************/
  bool ClearRoadmaps();

  /*!***************************************************************************
   * Queries the board for the number of roadmaps programmed.
   * Returns true if successful.
   ****************************************************************************/
  bool NumRoadmaps(uint16_t& num_roadmaps) const;

  /*!***************************************************************************
   * Computes a vector of collision results for the roadmap at the given index.
   * Returns true if successful.
   * Sensor data can be provided in one of three formats:
   * 1: PCL Point Clouds (merged and transformed to the correct frame)
   * 2: A list of voxels in the correct frame
   * 3: A list of aligned "voxel boxes" representing AABBs in the voxelspace
   *
   * For documentation and example code on correctly creating lists of voxels
   * and voxel boxes, go to rtr.ai/support.
   *****************************************************************************/
  bool CheckScene(pcl::PointCloud<pcl::PointXYZ>::ConstPtr scene_cloud, uint16_t roadmap_index,
                  std::vector<uint8_t>& collisions) const;

  bool CheckScene(const std::vector<Voxel>& scene_voxels, uint16_t roadmap_index,
                  std::vector<uint8_t>& collisions) const;

  bool CheckScene(const std::vector<Box>& scene_boxes, uint16_t roadmap_index,
                  std::vector<uint8_t>& collisions) const;

  /*!***************************************************************************
   * Returns true if the board is connected.
   ****************************************************************************/
  bool Connected() const;

  /*!***************************************************************************
   * Returns true if the board is initialized
   ****************************************************************************/
  bool Handshake() const;

  /*!***************************************************************************
   * Clears the existing roadmaps, runs a test and diffs the results against
   * an expected output.
   * Returns true if the test results match the expected.
   ****************************************************************************/
  bool Test();

  /*!***************************************************************************
   * Saves the current state of the world for later analysis
   *****************************************************************************/
  bool SaveSnapshot(const std::string& fname, pcl::PointCloud<pcl::PointXYZ>::ConstPtr scene_cloud,
                    unsigned int roadmap_index) const;

  bool SaveSnapshot(const std::string& fname, const std::vector<Voxel>& scene_voxels,
                    unsigned int roadmap_index) const;

  bool SaveSnapshot(const std::string& fname, const std::vector<Box>& scene_boxes,
                    unsigned int roadmap_index) const;

 private:
  // prevent any copying
  HardwareInterface(const HardwareInterface& rhs);
  HardwareInterface& operator=(const HardwareInterface& rhs);

  HardwareInterfacePrivate* hw_;
};

}  // namespace rtr

#endif  // HARDWARE_INTERFACE_H_
