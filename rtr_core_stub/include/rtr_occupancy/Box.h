#ifndef BOX_H_
#define BOX_H_

#include <iostream>
#include <set>
#include <tuple>
#include <rtr_occupancy/Voxel.h>

namespace rtr {

class Box {
public:
  Box();
  Box(const Box& b);
  Box(uint16_t _x_min, uint16_t _y_min, uint16_t _z_min,
      uint16_t _x_max, uint16_t _y_max, uint16_t _z_max);
  Box(const Voxel& v);
  Box(const Voxel& v1, const Voxel& v2);

  inline bool operator == (const Box& b) const;
  inline bool operator != (const Box& b) const;
  inline bool operator < (const Box &b) const;
  inline const Box& operator = (const Box& b);
  inline void Set(const Box& b);
  inline void Set(uint16_t _x_min, uint16_t _y_min, uint16_t _z_min,
                  uint16_t _x_max, uint16_t _y_max, uint16_t _z_max);
  inline void Set(const Voxel& v);
  inline void Set(const Voxel& v1, const Voxel& v2);

  int Volume() const;
  int OverlapVolume(const Box& b) const;
  void VoxelsEnclosed(std::vector<Voxel>& voxels) const;

  inline bool Contains(const Box& b) const; // this box encloses a superset of the other box
  inline bool Intersects(const Voxel& v) const;
  inline bool Intersects(const Box& b) const;
  inline void Merge(const Box& box, Box& merged) const;

  uint16_t x_min, y_min, z_min;
  uint16_t x_max, y_max, z_max;
};

std::ostream& operator << (std::ostream& out, const Box& b);
////////////////////////////////////////////////////////////////////////////////
// Functions
////////////////////////////////////////////////////////////////////////////////

inline bool Box::operator == (const Box& b) const {
  return b.x_min == x_min && b.x_max == x_max &&
         b.y_min == y_min && b.y_max == y_max &&
         b.z_min == z_min && b.z_max == z_max;
}


inline bool Box::operator != (const Box& b) const {
  return b.x_min != x_min || b.x_max != x_max ||
         b.y_min != y_min || b.y_max != y_max ||
         b.z_min != z_min || b.z_max != z_max;
}


inline bool Box::operator < (const Box &b) const {
  return std::tie(x_min, x_max, y_min, y_max, z_min, z_max) <
         std::tie(b.x_min, b.x_max, b.y_min, b.y_max, b.z_min, b.z_max);
}


inline const Box& Box::operator = (const Box& b) {
  Set(b);
  return *this;
}


inline void Box::Set(const Box& b) {
  x_min = b.x_min;  y_min = b.y_min;  z_min = b.z_min;
  x_max = b.x_max;  y_max = b.y_max;  z_max = b.z_max;
}


inline void Box::Set(uint16_t _x_min, uint16_t _y_min, uint16_t _z_min,
                     uint16_t _x_max, uint16_t _y_max, uint16_t _z_max) {
  x_min = _x_min;  y_min = _y_min;  z_min = _z_min;
  x_max = _x_max;  y_max = _y_max;  z_max = _z_max;
}


inline void Box::Set(const Voxel& v) {
  x_min = x_max = v.x;
  y_min = y_max = v.y;
  z_min = z_max = v.z;
}


inline void Box::Set(const Voxel& v1, const Voxel& v2) {
  x_min = std::min(v1.x, v2.x);  x_max = std::max(v1.x, v2.x);
  y_min = std::min(v1.y, v2.y);  y_max = std::max(v1.y, v2.y);
  z_min = std::min(v1.z, v2.z);  z_max = std::max(v1.z, v2.z);
}


inline bool Box::Contains(const Box& b) const {
  return b.x_min >= x_min && b.x_max <= x_max &&
         b.y_min >= y_min && b.y_max <= y_max &&
         b.z_min >= z_min && b.z_max <= z_max;
}


inline bool Box::Intersects(const Voxel& v) const {
  return v.x >= x_min && v.x <= x_max &&
         v.y >= y_min && v.y <= y_max &&
         v.z >= z_min && v.z <= z_max;
}


inline bool Box::Intersects(const Box& b) const {
  return !(x_max < b.x_min || b.x_max < x_min ||
           y_max < b.y_min || b.y_max < y_min ||
           z_max < b.z_min || b.z_max < z_min);
}


inline void Box::Merge(const Box& box, Box& merged) const {
  merged.x_min = std::min(x_min, box.x_min);
  merged.x_max = std::max(x_max, box.x_max);
  merged.y_min = std::min(y_min, box.y_min);
  merged.y_max = std::max(y_max, box.y_max);
  merged.z_min = std::min(z_min, box.z_min);
  merged.z_max = std::max(z_max, box.z_max);
}


} // namespace rtr

#endif // BOX_H_
