#ifndef CRITSANDVALUES_HPP
#define CRITSANDVALUES_HPP

#include "BezierGeometryGlobal.hpp"
#include "StaticVector.hpp"
#include <vector>

namespace bezier_geometry {
template <std::size_t MAX_SIZE> struct CritsAndValues {
  bool startIsCrit;
  bool endIsCrit;
  StaticVector<std::pair<RealNum, RealNum>, MAX_SIZE> critsAndValues;
};
} // namespace bezier_geometry

#endif // CRITSANDVALUES_HPP
