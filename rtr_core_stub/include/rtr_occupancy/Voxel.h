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
  Voxel(uint16_t _x, uint16_t _y, uint16_t _z);
  inline bool operator == (const Voxel& v) const;
  inline bool operator != (const Voxel& v) const;
  inline bool operator < (const Voxel& v) const;
  inline const Voxel& operator = (const Voxel& v);
  inline void Set(const Voxel& v);
  inline void Set(uint16_t _x, uint16_t _y, uint16_t _z);
  inline Voxel EntrywiseMin(const Voxel& v);
  inline Voxel EntrywiseMax(const Voxel& v);

  uint16_t x;
  uint16_t y;
  uint16_t z;
};

std::ostream& operator << (std::ostream& out, const Voxel& v);

////////////////////////////////////////////////////////////////////////////////
// Functions
////////////////////////////////////////////////////////////////////////////////
inline bool Voxel::operator == (const Voxel& v) const { return v.x == x && v.y == y && v.z == z; }
inline bool Voxel::operator != (const Voxel& v) const { return v.x != x || v.y != y || v.z != z; }
inline bool Voxel::operator < (const Voxel& v) const { return std::tie(x, y, z) < std::tie(v.x, v.y, v.z); }
inline const Voxel& Voxel::operator = (const Voxel& v) { Set(v); return *this; }
inline void Voxel::Set(const Voxel& v) { x = v.x; y = v.y; z = v.z; }
inline void Voxel::Set(uint16_t _x, uint16_t _y, uint16_t _z) { x = _x; y = _y; z = _z; }
inline Voxel Voxel::EntrywiseMin(const Voxel& v) { return Voxel(std::min(x, v.x), std::min(y, v.y), std::min(z, v.z)); }
inline Voxel Voxel::EntrywiseMax(const Voxel& v) { return Voxel(std::max(x, v.x), std::max(y, v.y), std::max(z, v.z)); }

} // namespace rtr

#endif // VOXEL_H_
