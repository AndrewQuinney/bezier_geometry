#include "Point2D.hpp"

#include <cmath>

#include "GeometryUtil.hpp"

namespace bezier_geometry {
const RealNum Point2D::twoPi = 2.0 * M_PI;

Point2D::Point2D() : Point2D(0, 0) {}

Point2D::Point2D(const RealNum &x, const RealNum &y) : xCoord(x), yCoord(y) {
  if (std::isinf(x) || std::isnan(x) || std::isinf(y) || std::isnan(y)) {
    throw std::string("Invalid point - (") + ::bezier_geometry::toString(x) +
        ", " + ::bezier_geometry::toString(y) + ")";
  }
}

const RealNum &Point2D::getX() const { return xCoord; }

const RealNum &Point2D::getY() const { return yCoord; }

RealNum Point2D::distanceFrom(const Point2D &input) const {
  return std::sqrt(std::pow(input.getX() - getX(), 2) +
                   std::pow(input.getY() - getY(), 2));
}

bool Point2D::operator==(const Point2D &input) const {
  if (!sufficientlyClose(getX(), input.getX()) ||
      !sufficientlyClose(getY(), input.getY())) {
    return false;
  }
  return sufficientlySmall(distanceFrom(input));
}

bool Point2D::operator!=(const Point2D &input) const {
  return !(*this == input);
}

Point2D Point2D::rotate(const Point2D &fulcrum, const RealNum &angle)
    const { // Positive angles represent a counterclockwise rotation.
  const RealNum xDiff = getX() - fulcrum.getX();
  const RealNum yDiff = getY() - fulcrum.getY();
  const RealNum cosAngle = std::cos(degreesToRadians(angle));
  const RealNum sinAngle = std::sin(degreesToRadians(angle));
  return Point2D((xDiff * cosAngle) - (yDiff * sinAngle) + fulcrum.getX(),
                 (xDiff * sinAngle) + (yDiff * cosAngle) + fulcrum.getY());
}

/*
Returns the point resulting from moving this point 'distance' units in the
direction specified by 'slope'.

-slope - the angle of the direction of movement
-right - when moving along the slope, whether or not this point is moving to the
right -up - when moving along the slope, whether or not this point is moving up;
only used if the slope is very large and there would be very little right/left
movement.
*/
Point2D Point2D::shift(const RealNum &distance, const RealNum &slope,
                       bool right, bool up) const {
  const RealNum aTanSlope = std::atan(slope);
  RealNum xDiff = distance * std::cos(aTanSlope);
  RealNum yDiff = distance * std::sin(aTanSlope);
  if (sufficientlyCloseSlopes(slope,
                              std::numeric_limits<RealNum>::infinity())) {
    if (up != (yDiff >= 0)) {
      xDiff *= -1.0;
      yDiff *= -1.0;
    }
  } else if (right != (xDiff >= 0)) {
    xDiff *= -1.0;
    yDiff *= -1.0;
  }
  return Point2D(getX() + xDiff, getY() + yDiff);
}

std::string Point2D::toString() const {
  return "(" + ::bezier_geometry::toString(getX()) + std::string(", ") +
         ::bezier_geometry::toString(getY()) + ")";
}

RealNum Point2D::getCWVerticalAngle(const Point2D &fulcrum) {
  return Point2D::getAngleBetween(Point2D(fulcrum.getX(), fulcrum.getY() + 100),
                                  *this, fulcrum, true);
}

/*
Returns the angle that 'first' must be rotated about 'fulcrum' in a
clockwise/counterclockwise direction to be colinear with 'second'.
*/
RealNum Point2D::getAngleBetween(const Point2D &first, const Point2D &second,
                                 const Point2D &fulcrum, bool clockwise) {
  RealNum firstCCWAngleWithPositiveHorizontal =
      std::atan2(first.getY() - fulcrum.getY(), first.getX() - fulcrum.getX());
  firstCCWAngleWithPositiveHorizontal =
      firstCCWAngleWithPositiveHorizontal < 0.0
          ? firstCCWAngleWithPositiveHorizontal + twoPi
          : firstCCWAngleWithPositiveHorizontal;
  RealNum secondCCWAngleWithPositiveHorizontal = std::atan2(
      second.getY() - fulcrum.getY(), second.getX() - fulcrum.getX());
  secondCCWAngleWithPositiveHorizontal =
      secondCCWAngleWithPositiveHorizontal < 0.0
          ? secondCCWAngleWithPositiveHorizontal + twoPi
          : secondCCWAngleWithPositiveHorizontal;
  if (clockwise) {
    return radiansToDegrees(firstCCWAngleWithPositiveHorizontal -
                            secondCCWAngleWithPositiveHorizontal +
                            (firstCCWAngleWithPositiveHorizontal >=
                                     secondCCWAngleWithPositiveHorizontal
                                 ? 0.0
                                 : twoPi));
  } else {
    return radiansToDegrees(secondCCWAngleWithPositiveHorizontal -
                            firstCCWAngleWithPositiveHorizontal +
                            (firstCCWAngleWithPositiveHorizontal <=
                                     secondCCWAngleWithPositiveHorizontal
                                 ? 0.0
                                 : twoPi));
  }
}

bool Point2D::colinear(const Point2D &p1, const Point2D &p2,
                       const Point2D &p3) {
  if (p1 == p2 || p1 == p3 || p2 == p3) {
    return true;
  }
  const RealNum testSlope = getSlopeBetween(p1, p2);
  return sufficientlyCloseSlopes(testSlope, getSlopeBetween(p1, p3)) &&
         sufficientlyClose(getPerpendicularMagnitude(p3, testSlope),
                           getPerpendicularMagnitude(p1, testSlope));
}

RealNum Point2D::getSlopeBetween(const Point2D &first, const Point2D &second) {
  return (second.getY() - first.getY()) / (second.getX() - first.getX());
}

/*
Return the point's minimum distance (positive or negative) from a line that
passes through the origin having the input slope.
*/
RealNum Point2D::getPerpendicularMagnitude(const Point2D &point,
                                           const RealNum &slope) {
  if (std::isinf(slope)) { // Just to prevent positive and negative infinity
                           // from returning different results.
    return point.getX();
  }
  // The rotation angle to make the input slope 0;
  const RealNum reverseAngleWithPositiveHorizontal = std::atan(slope) * (-1.0);
  // The y-coordinate portion of a point rotation matrix about the fulcrum.
  return (point.getX() * std::sin(reverseAngleWithPositiveHorizontal)) +
         (point.getY() * std::cos(reverseAngleWithPositiveHorizontal));
}

std::ostream &operator<<(std::ostream &os, const Point2D &input) {
  os << input.toString();
  return os;
}
} // namespace bezier_geometry
