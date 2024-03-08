#ifndef Point2D_HPP_
#define Point2D_HPP_

#include <string>

#include "BezierGeometryGlobal.hpp"

namespace bezier_geometry {
class Point2D {
public:
  Point2D();
  Point2D(const RealNum &x, const RealNum &y);
  const RealNum &getX() const;
  const RealNum &getY() const;
  RealNum distanceFrom(const Point2D &input) const;
  bool operator==(const Point2D &input) const;
  bool operator!=(const Point2D &input) const;
  Point2D rotate(const Point2D &fulcrum, const RealNum &angle) const;
  Point2D shift(const RealNum &distance, const RealNum &slope, bool right,
                bool up) const;
  std::string toString() const;
  RealNum getCWVerticalAngle(const Point2D &fulcrum);

  static RealNum getAngleBetween(const Point2D &first, const Point2D &second,
                                 const Point2D &fulcrum, bool clockwise);
  static RealNum getSlopeBetween(const Point2D &first, const Point2D &second);
  static bool colinear(const Point2D &p1, const Point2D &p2, const Point2D &p3);
  static RealNum getPerpendicularMagnitude(const Point2D &point,
                                           const RealNum &slope);

private:
  RealNum xCoord;
  RealNum yCoord;

  static const RealNum twoPi;
};

std::ostream &operator<<(std::ostream &os, const Point2D &input);
} // namespace bezier_geometry

#endif
