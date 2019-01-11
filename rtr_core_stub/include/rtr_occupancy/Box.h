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
  Box(uint16_t x_min, uint16_t y_min, uint16_t z_min,
      uint16_t x_max, uint16_t y_max, uint16_t z_max);
  Box(const Voxel& v);
  Box(const Voxel& v1, const Voxel& v2);

  inline bool operator == (const Box& b) const;
  inline bool operator != (const Box& b) const;
  inline bool operator < (const Box &b) const;
  inline const Box& operator = (const Box& b);
  inline void Set(const Box& b);
  inline void Set(uint16_t x_min, uint16_t y_min, uint16_t z_min,
                  uint16_t x_max, uint16_t y_max, uint16_t z_max);
  inline void Set(const Voxel& v);
  inline void Set(const Voxel& v1, const Voxel& v2);

  int Volume() const;
  int OverlapVolume(const Box& b) const;
  void VoxelsEnclosed(std::vector<Voxel>& voxels) const;

  inline bool Contains(const Box& b) const; // this box encloses a superset of the other box
  inline bool Intersects(const Voxel& v) const;
  inline bool Intersects(const Box& b) const;
  inline void Merge(const Box& box, Box& merged) const;

  uint16_t x_min_, y_min_, z_min_;
  uint16_t x_max_, y_max_, z_max_;
};

std::ostream& operator << (std::ostream& out, const Box& b);
////////////////////////////////////////////////////////////////////////////////
// Functions
////////////////////////////////////////////////////////////////////////////////

inline bool Box::operator == (const Box& b) const {
  return b.x_min_ == x_min_ && b.x_max_ == x_max_ &&
         b.y_min_ == y_min_ && b.y_max_ == y_max_ &&
         b.z_min_ == z_min_ && b.z_max_ == z_max_;
}


inline bool Box::operator != (const Box& b) const {
  return b.x_min_ != x_min_ || b.x_max_ != x_max_ ||
         b.y_min_ != y_min_ || b.y_max_ != y_max_ ||
         b.z_min_ != z_min_ || b.z_max_ != z_max_;
}


inline bool Box::operator < (const Box &b) const {
  return std::tie(x_min_, x_max_, y_min_, y_max_, z_min_, z_max_) <
         std::tie(b.x_min_, b.x_max_, b.y_min_, b.y_max_, b.z_min_, b.z_max_);
}


inline const Box& Box::operator = (const Box& b) {
  Set(b);
  return *this;
}


inline void Box::Set(const Box& b) {
  x_min_ = b.x_min_;  y_min_ = b.y_min_;  z_min_ = b.z_min_;
  x_max_ = b.x_max_;  y_max_ = b.y_max_;  z_max_ = b.z_max_;
}


inline void Box::Set(uint16_t x_min, uint16_t y_min, uint16_t z_min,
                     uint16_t x_max, uint16_t y_max, uint16_t z_max) {
  x_min_ = x_min;  y_min_ = y_min;  z_min_ = z_min;
  x_max_ = x_max;  y_max_ = y_max;  z_max_ = z_max;
}


inline void Box::Set(const Voxel& v) {
  x_min_ = x_max_ = v.x_;
  y_min_ = y_max_ = v.y_;
  z_min_ = z_max_ = v.z_;
}


inline void Box::Set(const Voxel& v1, const Voxel& v2) {
  x_min_ = std::min(v1.x_, v2.x_);  x_max_ = std::max(v1.x_, v2.x_);
  y_min_ = std::min(v1.y_, v2.y_);  y_max_ = std::max(v1.y_, v2.y_);
  z_min_ = std::min(v1.z_, v2.z_);  z_max_ = std::max(v1.z_, v2.z_);
}


inline bool Box::Contains(const Box& b) const {
  return b.x_min_ >= x_min_ && b.x_max_ <= x_max_ &&
         b.y_min_ >= y_min_ && b.y_max_ <= y_max_ &&
         b.z_min_ >= z_min_ && b.z_max_ <= z_max_;
}


inline bool Box::Intersects(const Voxel& v) const {
  return v.x_ >= x_min_ && v.x_ <= x_max_ &&
         v.y_ >= y_min_ && v.y_ <= y_max_ &&
         v.z_ >= z_min_ && v.z_ <= z_max_;
}


inline bool Box::Intersects(const Box& b) const {
  return !(x_max_ < b.x_min_ || b.x_max_ < x_min_ ||
           y_max_ < b.y_min_ || b.y_max_ < y_min_ ||
           z_max_ < b.z_min_ || b.z_max_ < z_min_);
}


inline void Box::Merge(const Box& box, Box& merged) const {
  merged.x_min_ = std::min(x_min_, box.x_min_);
  merged.x_max_ = std::max(x_max_, box.x_max_);
  merged.y_min_ = std::min(y_min_, box.y_min_);
  merged.y_max_ = std::max(y_max_, box.y_max_);
  merged.z_min_ = std::min(z_min_, box.z_min_);
  merged.z_max_ = std::max(z_max_, box.z_max_);
}


} // namespace rtr

#endif // BOX_H_
