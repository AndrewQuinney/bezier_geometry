#ifndef BEZIERGEOMETRYGLOBAL_HPP
#define BEZIERGEOMETRYGLOBAL_HPP

#include <ostream>

namespace bezier_geometry {
typedef double RealNum;

std::ostream &debug_out();
} // namespace bezier_geometry

#ifdef NDEBUG
#define debug                                                                  \
  while (false)                                                                \
  bezier_geometry::debug_out
#else
#define debug bezier_geometry::debug_out
#endif

#endif
