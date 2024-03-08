#include "CircleArc.hpp"

#include <cmath>

#include "GeometryUtil.hpp"

namespace bezier_geometry {
namespace {
Point2D getPositiveVerticalPoint(const Point2D &circleFulcrum,
                                 const RealNum &circleRadius) {
  return Point2D(circleFulcrum.getX(), circleFulcrum.getY() + circleRadius);
}

Point2D getArcPoint(const Point2D &circleFulcrum, const RealNum &circleRadius,
                    const RealNum &PVCWAngle) {
  return getPositiveVerticalPoint(circleFulcrum, circleRadius)
      .rotate(circleFulcrum, (-1.0) * PVCWAngle);
}

void checkParameter(const RealNum &parameter) {
  if (std::isnan(parameter) || parameter < 0 || parameter > 1) {
    throw std::string("Invalid parameter: ") + toString(parameter);
  }
}
} // namespace

CircleArc::CircleArc(const Point2D &circleFulcrum, const RealNum &circleRadius,
                     const RealNum &startPVCWAngle, const RealNum &endPVCWAngle)
    : circleFulcrum(circleFulcrum), circleRadius(circleRadius),
      fullCircle(getArcPoint(circleFulcrum, circleRadius, startPVCWAngle) ==
                 getArcPoint(circleFulcrum, circleRadius, endPVCWAngle)),
      startPVCWAngle(fullCircle ? 0.0 : startPVCWAngle),
      endPVCWAngle(fullCircle ? 360.0 : endPVCWAngle),
      angleMagnitude(fullCircle ? 360.0
                                : Point2D::getAngleBetween(
                                      getArcStartPoint(), getArcEndPoint(),
                                      circleFulcrum, true)) {
  if (startPVCWAngle < 0 || startPVCWAngle > 360 || endPVCWAngle < 0 ||
      endPVCWAngle > 360) {
    throw std::string("Invalid positive vertical clockwise angles: ") +
        toString(startPVCWAngle) + ", " + toString(endPVCWAngle);
  }
}

const Point2D &CircleArc::getFulcrum() const { return circleFulcrum; }

const RealNum &CircleArc::getRadius() const { return circleRadius; }

bool CircleArc::isFullCircle() const { return fullCircle; }

Point2D CircleArc::getArcStartPoint() const {
  return fullCircle ? getPositiveVerticalPoint(circleFulcrum, circleRadius)
                    : getArcPoint(circleFulcrum, circleRadius, startPVCWAngle);
}

Point2D CircleArc::getArcEndPoint() const {
  return fullCircle ? getPositiveVerticalPoint(circleFulcrum, circleRadius)
                    : getArcPoint(circleFulcrum, circleRadius, endPVCWAngle);
}

CircleArc::PointInArc CircleArc::liesWithinArc(const Point2D &input)
    const { // Currently does NOT validate distance from the fulcrum.
  return fullCircle                    ? PointInArc::INSIDE
         : getArcStartPoint() == input ? PointInArc::START
         : getArcEndPoint() == input   ? PointInArc::END
         : angleMagnitude > Point2D::getAngleBetween(getArcStartPoint(), input,
                                                     circleFulcrum, true)
             ? PointInArc::INSIDE
             : PointInArc::NOT_IN_ARC;
}

CircleArc CircleArc::rotate(const Point2D &fulcrum,
                            const RealNum &angle) const {
  const Point2D circleFulcrumRotated(circleFulcrum.rotate(fulcrum, angle));
  if (fullCircle) {
    return CircleArc(circleFulcrumRotated, circleRadius, 0, 360.0);
  } else {
    const Point2D rotatedPositiveVertical(
        getPositiveVerticalPoint(circleFulcrumRotated, circleRadius));
    return CircleArc(
        circleFulcrumRotated, circleRadius,
        Point2D::getAngleBetween(rotatedPositiveVertical,
                                 getArcStartPoint().rotate(fulcrum, angle),
                                 circleFulcrumRotated, true),
        Point2D::getAngleBetween(rotatedPositiveVertical,
                                 getArcEndPoint().rotate(fulcrum, angle),
                                 circleFulcrumRotated, true));
  }
}

CircleArc CircleArc::shift(const RealNum &distance, const RealNum &slope,
                           bool right, bool up) const {
  return CircleArc(circleFulcrum.shift(distance, slope, right, up),
                   circleRadius, startPVCWAngle, endPVCWAngle);
}

/*
 * The curve parameters represent the angle from the positive vertical in the
 * range [0, 1].
 * */
CritsAndValues
CircleArc::getPerpendicularMagnitudeCritsAndValues(const RealNum &slope) const {
  return getCritsAndValues(slope, [&slope](const Point2D &input) -> RealNum {
    return Point2D::getPerpendicularMagnitude(input, slope);
  });
}

CritsAndValues
CircleArc::getDistanceCritsAndValues(const Point2D &fulcrum) const {
  if (fulcrum == circleFulcrum) {
    CritsAndValues result;
    result.startIsCrit = true;
    result.endIsCrit = true;
    result.critsAndValues = {{0, circleRadius}, {1, circleRadius}};
    return result;
  } else {
    return getCritsAndValues(
        (-1.0) / Point2D::getSlopeBetween(circleFulcrum, fulcrum),
        [&fulcrum](const Point2D &input) -> RealNum {
          return fulcrum.distanceFrom(input);
        });
  }
}

/*
 * A parameter is treated as the clockwise angle from the start of this arc
 * scaled so the parameter for the end of the arc has a parameter value of 1.
 * For full circles, the 'start' is considered to be the positive vertical
 * point.
 * */
RealNum CircleArc::getParamForPoint(
    const Point2D &input) const { // No validation on the input's distance from
  // the fulcrum is performed.
  const RealNum cwAngleFromStart =
      Point2D::getAngleBetween(getArcStartPoint(), input, circleFulcrum, true);
  if (cwAngleFromStart > angleMagnitude) {
    if (sufficientlyClose(cwAngleFromStart, angleMagnitude)) {
      return 1;
    } else if (sufficientlyClose(cwAngleFromStart, 360)) {
      return 0;
    } else {
      return std::numeric_limits<RealNum>::quiet_NaN();
    }
  } else {
    return cwAngleFromStart / angleMagnitude;
  }
}

Point2D CircleArc::valueAt(const RealNum &parameter) const {
  checkParameter(parameter);
  return getArcPoint(circleFulcrum, circleRadius,
                     startPVCWAngle + (angleMagnitude * parameter));
}

template <typename T>
CritsAndValues CircleArc::getCritsAndValues(const RealNum &slope,
                                            const T &valueCalc) const {
  CritsAndValues result;
  const RealNum perpSlope = (-1.0) / slope;
  const Point2D critPoint1(
      circleFulcrum.shift(circleRadius, perpSlope, true, true));
  const Point2D critPoint2(
      circleFulcrum.shift(circleRadius, perpSlope, false, false));
  const Point2D arcStart(getArcStartPoint());
  const Point2D arcEnd(getArcEndPoint());
  const bool critPoint1IsStart = critPoint1 == arcStart;
  const bool critPoint1IsEnd = critPoint1 == arcEnd;
  const bool critPoint2IsStart = critPoint2 == arcStart;
  const bool critPoint2IsEnd = critPoint2 == arcEnd;
  result.startIsCrit = critPoint1IsStart || critPoint2IsStart;
  result.endIsCrit = critPoint1IsEnd || critPoint2IsEnd;
  result.critsAndValues.push_back({0, valueCalc(arcStart)});
  RealNum critPoint1Param = getParamForPoint(critPoint1);
  RealNum critPoint2Param = getParamForPoint(critPoint2);
  if (!std::isnan(critPoint1Param) && !std::isnan(critPoint2Param)) {
    if (critPoint1Param < critPoint2Param) {
      if (!critPoint1IsStart) {
        result.critsAndValues.push_back(
            {critPoint1Param, valueCalc(critPoint1)});
      }
      if (!critPoint2IsEnd) {
        result.critsAndValues.push_back(
            {critPoint2Param, valueCalc(critPoint2)});
      }
    } else {
      if (!critPoint2IsStart) {
        result.critsAndValues.push_back(
            {critPoint2Param, valueCalc(critPoint2)});
      }
      if (!critPoint1IsEnd) {
        result.critsAndValues.push_back(
            {critPoint1Param, valueCalc(critPoint1)});
      }
    }
  } else if (!std::isnan(critPoint1Param)) {
    if ((!result.startIsCrit || critPoint1Param > 0.25) &&
        (!result.endIsCrit || critPoint1Param < 0.75)) {
      result.critsAndValues.push_back({critPoint1Param, valueCalc(critPoint1)});
    }
  } else if (!std::isnan(critPoint2Param)) {
    if ((!result.startIsCrit || critPoint2Param > 0.25) &&
        (!result.endIsCrit || critPoint2Param < 0.75)) {
      result.critsAndValues.push_back({critPoint2Param, valueCalc(critPoint2)});
    }
  }
  result.critsAndValues.push_back({1, valueCalc(arcEnd)});
  return result;
}
} // namespace bezier_geometry
