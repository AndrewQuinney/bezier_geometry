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
  std::vector<std::pair<RealNum, RealNum>>
  pointsOfIntersection(const BezierCurveQ &input) const;
  const Point2D &getControl() const;
  Point2D valueAt(const RealNum &parameter) const;
  std::string toString() const;
  Point2D getTestPointForEndIntersection(bool start) const;
  Point2D getTestPointForEndIntersection(const RealNum &myParam,
                                         const Point2D &myPoint,
                                         const Point2D &myStart,
                                         const RealNum &myRate) const;
  CritsAndValues getDistanceCritsAndValues(const Point2D &input) const;
  Point2D getDirectionIndicator(const RealNum &parameter) const;
  RealNum getConcavitySlope(const RealNum &parameter) const;
  Point2D getConcavityIndicator(const RealNum &parameter) const;
  CritsAndValues
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
      const RealNum &myParam, const CritsAndValues &myDistanceCrits,
      const BezierCurveQ &input, const RealNum &inputParam,
      const CritsAndValues &inputDistanceCrits, const Point2D &fulcrum,
      const bool clockwise, RealNum &outputBlockedCWVerticalAngleStart,
      RealNum &outputBlockedCWVerticalAngleEnd, bool &outputAtMyCrit,
      bool &outputAtInputCrit) const;
  RealNum getDistanceConcavity(const RealNum &param,
                               const Point2D &fulcrum) const;
  std::vector<RealNum> pointShiftAgainstParams(const Point2D &input,
                                               const RealNum &slope,
                                               bool skipIntersections) const;
  std::vector<RealNum> getCurveDistanceParams(const Point2D &fulcrum,
                                              const RealNum &distance,
                                              const RealNum &startParam,
                                              const RealNum &endParam) const;
  void getCWAngleIntervals(const Point2D &fulcrum,
                           std::vector<CWAngleInterval> &outputFirstSet,
                           std::vector<CWAngleInterval> &outputSecondSet) const;
  bool sufficientlyCloseAlongCurve(const RealNum &curveParam,
                                   const RealNum &testParam) const;
  RealNum paramForPoint(const Point2D &input) const;

  static BezierCurveQ longStraightLine(const RealNum &slope,
                                       const Point2D &point);
  static BezierCurveQ longStraightLine(const Point2D &start,
                                       const Point2D &point);
  static BezierCurveQ straightLine(const Point2D &s, const Point2D &e);
  static bool isIntersectionInfinite(
      const std::vector<std::pair<RealNum, RealNum>> &intersection);
  static std::pair<std::vector<Point2D>, std::vector<Point2D>>
  circleArc(const Point2D &fulcrum, const RealNum &radius,
            const RealNum &startCWPositiveVerticalAngle,
            const RealNum &cwAngleSize, int edges);

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

  static const std::vector<std::pair<RealNum, RealNum>> INFINITE_INTERSECTION;

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
      const CritsAndValues &myDistanceCrits, const BezierCurveQ &input,
      const CritsAndValues &inputDistanceCrits, const Point2D &fulcrum,
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
std::ostream &operator<<(std::ostream &os,
                         const std::vector<std::pair<RealNum, RealNum>> &input);
template <typename T>
std::ostream &operator<<(std::ostream &os, const std::vector<T> &input);
} // namespace bezier_geometry

#endif
