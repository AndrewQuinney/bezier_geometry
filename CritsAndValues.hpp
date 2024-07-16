#ifndef CRITSANDVALUES_HPP
#define CRITSANDVALUES_HPP

#include "BezierGeometryGlobal.hpp"
#include "StaticVector.hpp"
#include <vector>

namespace bezier_geometry {
/**
 * \brief Stores 'critical points' and their associated values.
 *
 * A critical point refers to a parameter for which a continuous value changes
 * direction, meaning that as the parameter increases, the associated value
 * stops increasing and starts decreasing (or vice versa). These values can
 * refer to coordinates, perpendicular magnitudes of a curve, curve distance
 * from a fixed point, etc.
 *
 * When referring to any critical values on a Bezier curve, the start and end
 * points of the curve are also considered to be critical points.
 */
template <std::size_t MAX_SIZE> struct CritsAndValues {
  bool startIsCrit; /**< True if the first entry is a critical point. Necessary
                       because 'start' points can often ben included whether or
                       not they are acutally critical points. */
  bool endIsCrit;   /**< True if the last entry is a critical point. Necessary
                       because 'end' points can often ben included whether or not
                       they are acutally critical points. */
  StaticVector<std::pair<RealNum, RealNum>, MAX_SIZE>
      critsAndValues; /**< The set of critical points and their associated
                         values. */
};
} // namespace bezier_geometry

#endif // CRITSANDVALUES_HPP
