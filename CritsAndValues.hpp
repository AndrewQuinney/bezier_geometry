#ifndef CRITSANDVALUES_HPP
#define CRITSANDVALUES_HPP

#include <vector>

#include "Point2D.hpp"

namespace bezier_geometry {
struct CritsAndValues {
  bool startIsCrit;
  bool endIsCrit;
  std::vector<std::pair<RealNum, RealNum>> critsAndValues;
};
} // namespace bezier_geometry

#endif // CRITSANDVALUES_HPP
