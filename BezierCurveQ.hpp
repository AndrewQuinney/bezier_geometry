#ifndef BezierCurveQ_HPP_
#define BezierCurveQ_HPP_

#include "BezierGeometryGlobal.hpp"
#include "CircleArc.hpp"
#include "CritsAndValues.hpp"
#include "Point2D.hpp"
#include "PolynomialFunction.hpp"

namespace bezier_geometry {

class BezierCurveQ {
public:
  struct ShiftAgainstResult {
    RealNum distance;
    RealNum param;
    RealNum inputParam;
    RealNum blockedCWVerticalAngleStart;
    RealNum blockedCWVerticalAngleEnd;
    bool infiniteInitialIntersection;
    bool atMyCrit;
    bool atInputCrit;
  };
  struct RotateAgainstResult {
    RealNum angle;
    RealNum param;
    RealNum inputParam;
    RealNum blockedCWVerticalAngleStart;
    RealNum blockedCWVerticalAngleEnd;
    bool infiniteInitialIntersection;
    bool atMyCrit;
    bool atInputCrit;
  };
  struct CWAngleInterval { // endParam always indicates a point clockwise of
                           // startParam about a fulcrum.
    RealNum startParam;
    RealNum startCWVerticalAngle;
    RealNum endParam;
    RealNum endCWVerticalAngle;
  };

  BezierCurveQ(const Point2D &s, const Point2D &e, const Point2D &c);

  BezierCurveQ rotate(const Point2D &fulcrum, const RealNum &angle) const;
  BezierCurveQ shift(const RealNum &distance, const RealNum &slope, bool right,
                     bool up) const;
  bool operator==(const BezierCurveQ &input) const;
  RealNum maxDistance(const Point2D &input) const;
  RealNum maxDistance(const Point2D &input, const RealNum &minParam,
                      const RealNum &maxParam) const;
  const RealNum &getMaxXExtent() const;
  const RealNum &getMaxXPara() const;
  const RealNum &getMaxYExtent() const;
  const RealNum &getMaxYPara() const;
  RealNum minDistance(const Point2D &input) const;
  RealNum minDistance(const Point2D &input, const RealNum &minParam,
                      const RealNum &maxParam) const;
  const RealNum &getMinXExtent() const;
  const RealNum &getMinXPara() const;
  const RealNum &getMinYExtent() const;
  const RealNum &getMinYPara() const;
  RealNum rateOfChangeAtParam(const RealNum &parameter) const;
  void rotateAgainst(const BezierCurveQ &input, const Point2D &fulcrum,
                     bool clockwise, RotateAgainstResult &output) const;
  void shiftAgainst(const BezierCurveQ &input, const RealNum &slope, bool right,
                    bool up, ShiftAgainstResult &output) const;
  StaticVector<std::pair<RealNum, RealNum>, 4>
  pointsOfIntersection(const BezierCurveQ &input) const;
  const Point2D &getControl() const;
  Point2D valueAt(const RealNum &parameter) const;
  std::string toString() const;
  Point2D getTestPointForEndIntersection(bool start) const;
  Point2D getTestPointForEndIntersection(const RealNum &myParam,
                                         const Point2D &myPoint,
                                         const Point2D &myStart,
                                         const RealNum &myRate) const;
  CritsAndValues<5> getDistanceCritsAndValues(const Point2D &input) const;
  Point2D getDirectionIndicator(const RealNum &parameter) const;
  RealNum getConcavitySlope(const RealNum &parameter) const;
  Point2D getConcavityIndicator(const RealNum &parameter) const;
  CritsAndValues<3>
  getPerpendicularMagnitudeCritsAndValues(const RealNum &slope) const;
  void getCWAngleRangeWithPositiveVertical(const Point2D &fulcrum,
                                           RealNum &outputStart1,
                                           RealNum &outputEnd1,
                                           RealNum &outputStart2,
                                           RealNum &outputEnd2) const;
  void getRangeAlongSlope(const RealNum &slope, RealNum &outputStart,
                          RealNum &outputEnd) const;
  void rotateAgainstCircleArc(const CircleArc &circleArc,
                              const Point2D &fulcrum, bool clockwise,
                              RealNum &outputAngle, RealNum &outputParam,
                              Point2D &outputCirclePoint) const;
  void shiftAgainstCircleArc(const CircleArc &circleArc, const RealNum &slope,
                             bool right, bool up, RealNum &outputDistance,
                             RealNum &outputParam,
                             Point2D &outputCirclePoint) const;
  bool curveIntersectionBlocksShift(const RealNum &myParam,
                                    const RealNum &slope, bool right, bool up,
                                    const RealNum &inputParam,
                                    const BezierCurveQ &input,
                                    const bool tolerateSmallMagnitudeOverlap,
                                    RealNum &outputBlockedCWVerticalAngleStart,
                                    RealNum &outputBlockedCWVerticalAngleEnd,
                                    bool &outputAtMyCrit,
                                    bool &outputAtInputCrit) const;
  bool curveIntersectionBlocksRotate(
      const RealNum &myParam, const CritsAndValues<5> &myDistanceCrits,
      const BezierCurveQ &input, const RealNum &inputParam,
      const CritsAndValues<5> &inputDistanceCrits, const Point2D &fulcrum,
      const bool clockwise, RealNum &outputBlockedCWVerticalAngleStart,
      RealNum &outputBlockedCWVerticalAngleEnd, bool &outputAtMyCrit,
      bool &outputAtInputCrit) const;
  RealNum getDistanceConcavity(const RealNum &param,
                               const Point2D &fulcrum) const;
  StaticVector<RealNum, 2>
  pointShiftAgainstParams(const Point2D &input, const RealNum &slope,
                          bool skipIntersections) const;
  StaticVector<RealNum, 4>
  getCurveDistanceParams(const Point2D &fulcrum, const RealNum &distance,
                         const RealNum &startParam,
                         const RealNum &endParam) const;
  void
  getCWAngleIntervals(const Point2D &fulcrum,
                      StaticVector<CWAngleInterval, 4> &outputFirstSet,
                      StaticVector<CWAngleInterval, 4> &outputSecondSet) const;
  bool sufficientlyCloseAlongCurve(const RealNum &curveParam,
                                   const RealNum &testParam) const;
  RealNum paramForPoint(const Point2D &input) const;

  static BezierCurveQ longStraightLine(const RealNum &slope,
                                       const Point2D &point);
  static BezierCurveQ longStraightLine(const Point2D &start,
                                       const Point2D &point);
  static BezierCurveQ straightLine(const Point2D &s, const Point2D &e);
  static bool isIntersectionInfinite(
      const StaticVector<std::pair<RealNum, RealNum>, 4> &intersection);

  template <std::size_t EDGES>
  static typename std::enable_if<(EDGES == 1)>::type
  validateAngle(const RealNum &cwAngleSize) {
    if (cwAngleSize > 180 || sufficientlyClose(cwAngleSize, 180)) {
      throw std::string("Angle sizes greater than or equal to 180 degrees "
                        "require at least 2 edges.");
    }
  }

  template <std::size_t EDGES>
  static typename std::enable_if<(EDGES == 2)>::type
  validateAngle(const RealNum &cwAngleSize) {
    if (sufficientlyClose(cwAngleSize, 360)) {
      throw std::string("Full circles require at least 3 edges.");
    }
  }

  template <std::size_t EDGES>
  static typename std::enable_if<(EDGES > 2)>::type
  validateAngle(const RealNum &cwAngleSize) {}

  /*
   * The first entry in the returned result are the endpoints, the second entry
   * are the control points.
   *
   * The first entry always has one more entry than the second, since it also
   * encapsulates the end of the last edge.
   * */
  template <std::size_t EDGES>
  static typename std::enable_if<(EDGES > 1),
                                 std::pair<StaticVector<Point2D, EDGES + 1>,
                                           StaticVector<Point2D, EDGES>>>::type
  circleArc(const Point2D &fulcrum, const RealNum &radius,
            const RealNum &startCWPositiveVerticalAngle,
            const RealNum &cwAngleSize) {
    if (cwAngleSize <= 0 || cwAngleSize > 360) {
      throw std::string("Invalid arc angle size: ") +
          std::to_string(cwAngleSize);
    }
    validateAngle<EDGES>(cwAngleSize);
    const RealNum angleIncrement = (-1.0) * cwAngleSize / RealNum(EDGES);
    Point2D currentControl;
    {
      Point2D startPoint(fulcrum.shift(radius, 0, true, true));
      Point2D endPoint(startPoint.rotate(fulcrum, angleIncrement));
      RealNum endTangentSlope =
          (-1.0) / Point2D::getSlopeBetween(endPoint, fulcrum);
      currentControl =
          Point2D(startPoint.getX(),
                  endPoint.getY() +
                      (endTangentSlope * (startPoint.getX() - endPoint.getX())))
              .rotate(fulcrum, 90 - startCWPositiveVerticalAngle);
    }
    Point2D currentEndpoint =
        Point2D(fulcrum.getX(), fulcrum.getY() + radius)
            .rotate(fulcrum, (-1.0) * startCWPositiveVerticalAngle);
    std::pair<StaticVector<Point2D, EDGES + 1>, StaticVector<Point2D, EDGES>>
        result;
    for (int i = 0; i < EDGES; i++) {
      result.first.push_back(currentEndpoint);
      currentEndpoint = currentEndpoint.rotate(fulcrum, angleIncrement);
      result.second.push_back(currentControl);
      currentControl = currentControl.rotate(fulcrum, angleIncrement);
    }
    result.first.push_back(currentEndpoint);
    return result;
  }

private:
  RealNum maxX;
  RealNum maxXPara;
  RealNum maxY;
  RealNum maxYPara;
  RealNum minX;
  RealNum minXPara;
  RealNum minY;
  RealNum minYPara;
  Point2D control;
  PolynomialFunction<3>
      paraFormX; // The parametric form of this curve for the x-coordinate.
  PolynomialFunction<3>
      paraFormY; // The parametric form of this curve for the y-coordinate.

  static const StaticVector<std::pair<RealNum, RealNum>, 4>
      INFINITE_INTERSECTION;

  RealNum minMaxDistance(const Point2D &input, bool max,
                         const RealNum &minParam,
                         const RealNum &maxParam) const;
  PolynomialFunction<5> getCurveDistanceSquared(const Point2D &fulcrum) const;
  const PolynomialFunction<3> &getParaFormX() const;
  const PolynomialFunction<3> &getParaFormY() const;
  bool straightLine() const;
  bool straightLine(RealNum &outputSlope) const;
  void populateInfiniteIntersectionShiftBlockValues(
      const RealNum &slope, bool right, bool up,
      RealNum &outputBlockedCWVerticalAngleStart,
      RealNum &outputBlockedCWVerticalAngleEnd) const;
  bool overlapBlocksEverything(const RealNum &myParam,
                               const BezierCurveQ &input,
                               const RealNum &inputParam,
                               const bool testMineOnly) const;
  bool differentSlopesIntersectionConcavityBlocks(
      const RealNum &slope, bool right, bool up, const RealNum &myParam,
      const BezierCurveQ &input, const RealNum &inputParam) const;
  bool differentSlopesEndpointIntersectionBlocks(
      const RealNum &slope, bool right, bool up, const RealNum &myParam,
      bool atMyStart, const BezierCurveQ &input,
      const RealNum &inputParam) const;
  bool movesTowardsFromEndpoint(bool atStart, const RealNum &slope, bool right,
                                bool up) const;
  void endpointRotationConditions(
      const CritsAndValues<5> &myDistanceCrits, const BezierCurveQ &input,
      const CritsAndValues<5> &inputDistanceCrits, const Point2D &fulcrum,
      bool clockwise, BezierCurveQ::RotateAgainstResult &output) const;
  BezierCurveQ getTestCircleArc(const RealNum &parameter,
                                const Point2D &circleFulcrum) const;
  BezierCurveQ getTestCircleArcForEndpoint(const RealNum &parameter,
                                           const Point2D &circleFulcrum,
                                           bool clockwiseFromEndpoint) const;
  RealNum tangentSpeed(const RealNum &parameter) const;
  RealNum paramForSlope(const RealNum &slope) const;
  RealNum straightLineParamForPoint(const RealNum &lineSlope,
                                    const Point2D &targetPoint) const;
  RealNum getRotationTangentSpeed(const Point2D &fulcrum,
                                  const RealNum &param) const;
  RealNum getDistanceSlope(const Point2D &fulcrum, const RealNum &param,
                           const RealNum &rotationTangentSpeed) const;

  static void checkParam(const RealNum &parameter);
};

std::ostream &operator<<(std::ostream &os, const BezierCurveQ &input);
} // namespace bezier_geometry

#endif
