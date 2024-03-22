#ifndef CIRCLEARC_H
#define CIRCLEARC_H

#include "CritsAndValues.hpp"
#include "Point2D.hpp"

namespace bezier_geometry {
class CircleArc {
public:
  enum class PointInArc { START, END, INSIDE, NOT_IN_ARC };

  CircleArc(const Point2D &circleFulcrum, const RealNum &circleRadius,
            const RealNum &startPVCWAngle, const RealNum &endPVCWAngle);
  const Point2D &getFulcrum() const;
  const RealNum &getRadius() const;
  bool isFullCircle() const;
  Point2D getArcStartPoint() const;
  Point2D getArcEndPoint() const;
  PointInArc liesWithinArc(const Point2D &input) const;
  CircleArc rotate(const Point2D &fulcrum, const RealNum &angle) const;
  CircleArc shift(const RealNum &distance, const RealNum &slope, bool right,
                  bool up) const;
  Point2D valueAt(const RealNum &parameter) const;
  CritsAndValues<4>
  getPerpendicularMagnitudeCritsAndValues(const RealNum &slope) const;
  CritsAndValues<4> getDistanceCritsAndValues(const Point2D &fulcrum) const;

private:
  const Point2D circleFulcrum;
  const RealNum circleRadius;
  const bool fullCircle;
  const RealNum startPVCWAngle;
  const RealNum endPVCWAngle;
  const RealNum angleMagnitude;

  RealNum getParamForPoint(const Point2D &input) const;
  template <typename T>
  CritsAndValues<4> getCritsAndValues(const RealNum &slope,
                                      const T &valueCalc) const;
};
} // namespace bezier_geometry

#endif // CIRCLEARC_H
