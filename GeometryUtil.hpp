#ifndef GeometryUtil_HPP_
#define GeometryUtil_HPP_

#include "BezierGeometryGlobal.hpp"

namespace bezier_geometry {
extern const RealNum ACCEPTABLE_ERROR_MARGIN;
bool sufficientlySmall(const RealNum &input);
bool sufficientlyClose(const RealNum &input1, const RealNum &input2);
bool sufficientlyCloseSlopes(const RealNum &slope1, const RealNum &slope2);
void sufficientlyDifferentSlopes(const RealNum &slope, RealNum &outputSlope1,
                                 RealNum &outputSlope2);
RealNum rotateSlope(const RealNum &slope, const RealNum &rotationAngle);
bool fuzzyEquals(const RealNum &input1, const RealNum &input2);
bool fuzzyIsNull(const RealNum &input);
RealNum radiansToDegrees(const RealNum &radians);
RealNum degreesToRadians(const RealNum &degrees);
std::string toString(const RealNum &input);
} // namespace bezier_geometry

#endif
