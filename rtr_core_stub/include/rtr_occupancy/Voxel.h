#ifndef VOXEL_H_
#define VOXEL_H_

#include <iostream>
#include <tuple>
#include <vector>

namespace rtr {

class Voxel {
public:
  Voxel();
  Voxel(const Voxel& v);
  Voxel(uint16_t x, uint16_t y, uint16_t z);
  inline bool operator == (const Voxel& v) const;
  inline bool operator != (const Voxel& v) const;
  inline bool operator < (const Voxel& v) const;
  inline const Voxel& operator = (const Voxel& v);
  inline void Set(const Voxel& v);
  inline void Set(uint16_t x, uint16_t y, uint16_t z);
  inline Voxel EntrywiseMin(const Voxel& v);
  inline Voxel EntrywiseMax(const Voxel& v);

  uint16_t x_;
  uint16_t y_;
  uint16_t z_;
};

std::ostream& operator << (std::ostream& out, const Voxel& v);

////////////////////////////////////////////////////////////////////////////////
// Functions
////////////////////////////////////////////////////////////////////////////////
inline bool Voxel::operator == (const Voxel& v) const { return v.x_ == x_ && v.y_ == y_ && v.z_ == z_; }
inline bool Voxel::operator != (const Voxel& v) const { return v.x_ != x_ || v.y_ != y_ || v.z_ != z_; }
inline bool Voxel::operator < (const Voxel& v) const { return std::tie(x_, y_, z_) < std::tie(v.x_, v.y_, v.z_); }
inline const Voxel& Voxel::operator = (const Voxel& v) { Set(v); return *this; }
inline void Voxel::Set(const Voxel& v) { x_ = v.x_; y_ = v.y_; z_ = v.z_; }
inline void Voxel::Set(uint16_t x, uint16_t y, uint16_t z) { x_ = x; y_ = y; z_ = z; }
inline Voxel Voxel::EntrywiseMin(const Voxel& v) { return Voxel(std::min(x_, v.x_), std::min(y_, v.y_), std::min(z_, v.z_)); }
inline Voxel Voxel::EntrywiseMax(const Voxel& v) { return Voxel(std::max(x_, v.x_), std::max(y_, v.y_), std::max(z_, v.z_)); }

} // namespace rtr

#endif // VOXEL_H_
