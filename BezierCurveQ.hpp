#ifndef BezierCurveQ_HPP_
#define BezierCurveQ_HPP_

#include "BezierGeometryGlobal.hpp"
#include "CircleArc.hpp"
#include "CritsAndValues.hpp"
#include "Point2D.hpp"
#include "PolynomialFunction.hpp"

namespace bezier_geometry {
/**
 * \brief A quadratic Bezier Curve.
 *
 * Represents a single quadratic Bezier Curve, meaning that it specifies a
 * start, end, and single control point. The quadratic version of Bezier Curves
 * never intersect themselves, are always convex, and only have at most a single
 * point at a particular slope (with the exception of straight lines).
 *
 * This implementation supports several functions that are important to a
 * geometry processing system, such as:
 * * Bounding boxes.
 * * Intersection.
 * * Collision detection with other curves, for both rotations and straight line
 * shifts.
 * * Relevant interval calculation for different types of translations.
 *
 * A curve consists of two polynomial functions - one for the X-coordinate and
 * one for the Y-coordinate, derived from the input points and using a single
 * common parameter. These equations are considered defined for parameter ranges
 * [0, 1], with a parameter value of 0 yielding the star point and a value of 1
 * yielding the end; every other value between these represents a point along
 * the curve.
 */
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

  /**
   * \brief Creates an instance of a curve using the standard parameters.
   *
   * A quadratic Bezier Curve is created by supplying a start, end and control
   * point.
   */
  BezierCurveQ(const Point2D &s, /**< The start point. */
               const Point2D &e, /**< The end point. */
               const Point2D &c  /**< The control point. */
  );
  /**
   * \brief Calculate and return the curve that results from rotating this
   * curve.
   *
   * The new curve is derived by simply constructing a new curve with this
   * curve's start, end and control points rotated by the input parameters. \see
   * Point2D::rotate \see ClockwiseLogic
   *
   * @return The curve resulting from the input rotation.
   */
  BezierCurveQ
  rotate(const Point2D &fulcrum, /**< The fulcrum about which the rotation is to
                                    be calculated. */
         const RealNum
             &angle /**< The angle (in degrees) to be rotated; an angle that is
                    a multiple of 360 (including 0) results in a curve
                    equivalent to this one. Keeping with the behaviour of a
                    rotation matrix, negative angles are 'clockwise' and
                    positive angles are 'counterclockwise'. */
  ) const;
  /**
   * \brief Calculate and return the curve that results from moving this curve
   * by a given distance and direction.
   *
   * The new curve is derived by simply constructing a new curve with this
   * curve's start, end and control points shifted by the input parameters. \see
   * Point2D::shift \see \ref DirectionalLogic
   *
   * @return The curve resulting from the input shift.
   */
  BezierCurveQ shift(
      const RealNum &distance, /**< The straight-line distance to be moved. */
      const RealNum &
          slope, /**< The slope along which the movement is to be calculated. */
      bool right, /**< The direction along the input slope. */
      bool up /**< The direction along the input slope, only used if the slope
                 is very large. */
  ) const;
  /**
   * \brief Tests if this curve is sufficiently close to the input curve to be
   * considered equivalent.
   *
   * Equivalence is determined by comparing each curve's start, end, and control
   * points using the conditions for point equivalence. \see Point2D::operator==
   *
   * @return True if this curve and the input curve are equal.
   */
  bool operator==(const BezierCurveQ &input /**< The curve that is to be
                                               compared against this curve. */
  ) const;
  /**
   * \brief Get the point on this curve that is farthest from the input point.
   *
   * This is the same as calling BezierCurveQ::maxDistance(input, 0, 1).
   *
   * @return The parameter value for this curve that yields a point farther from
   * the input point than any other on this curve. Always in the range [0, 1].
   */
  RealNum
  maxDistance(const Point2D &input /**< The point from which the farthest
                                      distance is to be calculated. */
  ) const;
  /**
   * \brief Get the point on this curve that is farthest from the input point
   * within a parameter range.
   *
   * The farthest point on the section of this curve specified by the input
   * parameters is calculated.
   *
   * @return The parameter value for this curve that yields a point farther from
   * the input point than any other on this curve within the specified parameter
   * range. Always in the range [minParam, maxParam].
   */
  RealNum maxDistance(
      const Point2D &input, /**< The point from which the farthest distance is
                               to be calculated. */
      const RealNum &minParam, /**< The parameter indicating the start of the
                                  segment of this curve to perform the
                                  calculation on. Must be in the range [0, 1] */
      const RealNum
          &maxParam /**< The parameter indicating the end of the segment of this
                       curve to perform the calculation on. Must be in the range
                       [minParam, 1]. */
  ) const;
  /**
   * \brief Get the highest X-coordinate that any point on this curve occupies.
   *
   * @return The highest X-coordinate that this curve reaches.
   */
  const RealNum &getMaxXExtent() const;
  /**
   * \brief Get the parameter for the highest X-coordinate that any point on
   * this curve occupies.
   *
   * @return The parameter for the highest X-coordinate that this curve reaches.
   * Always in the range [0, 1].
   */
  const RealNum &getMaxXPara() const;
  /**
   * \brief Get the highest Y-coordinate that any point on this curve occupies.
   *
   * @return The highest Y-coordinate that this curve reaches.
   */
  const RealNum &getMaxYExtent() const;
  /**
   * \brief Get the parameter for the highest Y-coordinate that any point on
   * this curve occupies.
   *
   * @return The parameter for the highest Y-coordinate that this curve reaches.
   * Always in the range [0, 1].
   */
  const RealNum &getMaxYPara() const;
  /**
   * \brief Get the point on this curve that is closest to the input point.
   *
   * This is the same as calling BezierCurveQ::minDistance(input, 0, 1).
   *
   * @return The parameter value for this curve that yields a point closer to
   * the input point than any other on this curve. Always in the range [0, 1].
   */
  RealNum minDistance(const Point2D &input /**< The point to which the closest
                                              distance is to be calculated. */
  ) const;
  /**
   * \brief Get the point on this curve that is closest to the input point
   * within a parameter range.
   *
   * The closest point on the section of this curve specified by the input
   * parameters is calculated.
   *
   * @return The parameter value for this curve that yields a point closer to
   * the input point than any other on this curve within the specified parameter
   * range. Always in the range [minParam, maxParam].
   */
  RealNum minDistance(
      const Point2D &input, /**< The point to which the closest distance is to
                               be calculated. */
      const RealNum &minParam, /**< The parameter indicating the start of the
                                  segment of this curve to perform the
                                  calculation on. Must be in the range [0, 1] */
      const RealNum
          &maxParam /**< The parameter indicating the end of the segment of this
                       curve to perform the calculation on. Must be in the range
                       [minParam, 1]. */
  ) const;
  /**
   * \brief Get the lowest X-coordinate that any point on this curve occupies.
   *
   * @return The lowest X-coordinate that this curve reaches.
   */
  const RealNum &getMinXExtent() const;
  /**
   * \brief Get the parameter for the lowest X-coordinate that any point on
   * this curve occupies.
   *
   * @return The parameter for the lowest X-coordinate that this curve reaches.
   * Always in the range [0, 1].
   */
  const RealNum &getMinXPara() const;
  /**
   * \brief Get the lowest Y-coordinate that any point on this curve occupies.
   *
   * @return The lowest Y-coordinate that this curve reaches.
   */
  const RealNum &getMinYExtent() const;
  /**
   * \brief Get the parameter for the lowest Y-coordinate that any point on
   * this curve occupies.
   *
   * @return The parameter for the lowest Y-coordinate that this curve reaches.
   * Always in the range [0, 1].
   */
  const RealNum &getMinYPara() const;
  /**
   * \brief get this curve's slope at a parameter.
   *
   * \see \ref DirectionalLogic
   *
   * @return This curve's slope at the given parameter.
   */
  RealNum rateOfChangeAtParam(
      const RealNum &parameter /**< The curve parameter to evaluate the slope
                                  at. Must be in the range [0, 1]. */
  ) const;
  /**
   * \brief Calculate the angle that this curve can be rotated until it 'hits'
   * the input curve.
   *
   * Given a fulcrum and rotation direction, calculate how much of an angle this
   * curve can be rotated until it is 'blocked' by the input curve. This method
   * also calculates other potentially useful values, such as the resulting
   * intersection parameters for both curves and the other directions that would
   * also be blocked if such a rotation were performed and subsequent moves were
   * attempted.
   *
   * A 'block' is not necessarily a place where this curve would touch/intersect
   * after a rotation. Any such touch must sufficiently overlap the direction of
   * rotation to ensure that a 'graze' would not cause a block, such as one
   * curve simply moving tangentially to another.
   *
   * \see ClockwiseLogic
   * \see SufficientlyClose
   */
  void rotateAgainst(
      const BezierCurveQ &input, /**< The curve that a rotation against will be
                                    checked for 'blocks'. */
      const Point2D &fulcrum,    /**< The fulcrum of the rotation. */
      bool clockwise,            /**< The direction of the rotation. */
      RotateAgainstResult
          &output /**< The destination for the calculated results. */
  ) const;
  /**
   * \brief Calculate the distance that this curve can be shifted until it
   * 'hits' the input curve.
   *
   * Given a shift direction, calculate how far this
   * curve can be moved in a straight line until it is 'blocked' by the input
   * curve. This method also calculates other potentially useful values, such as
   * the resulting intersection parameters for both curves and the other
   * directions that would also be blocked if such a shift were performed and
   * subsequent moves were attempted.
   *
   * A 'block' is not necessarily a place where this curve would touch/intersect
   * after a rotation. Any such touch must sufficiently overlap the direction of
   * rotation to ensure that a 'graze' would not cause a block, such as one
   * curve simply moving tangentially to another.
   *
   * \see DirectionalLogic
   * \see SufficientlyClose
   */
  void shiftAgainst(
      const BezierCurveQ &input, /**< The curve that a shift against will be
                                    checked for 'blocks'. */
      const RealNum &
          slope, /**< The slope along which the movement is to be calculated. */
      bool right, /**< The direction along the input slope. */
      bool up, /**< The direction along the input slope, only used if the slope
                     is very large. */
      ShiftAgainstResult
          &output /**< The destination for the calculated results. */
  ) const;
  /**
   * \brief Calculate the points of intersection for this and another curve.
   *
   * Two quadratic Bezier curves cannot intersect at more than 4 points.
   *
   * @return A list of parameter pairs (each numeric value in the range [0, 1]).
   * The first entry in each pair is a parameter to this curve, the second is a
   * parameter to the input curve; each pair representing a point at which these
   * curves intersect. There is a special case for curves that infinitely
   * intersect. \see isIntersectionInfinite
   */
  StaticVector<std::pair<RealNum, RealNum>, 4> pointsOfIntersection(
      const BezierCurveQ &input /**< The other curve to test for intersection
                                   with this curve. */
  ) const;
  /**
   * \brief Get the single control point for this curve.
   *
   * @return This curve's Bezier control point.
   */
  const Point2D &getControl() const;
  /**
   * \brief Calculate the point on this curve at the specified parameter.
   *
   * Bezier curves are continuous, so there is a point for every valid
   * parameter.
   *
   * @return The point on this curve for the given parameter.
   */
  Point2D valueAt(
      const RealNum &parameter /**< The parameter for this curve to evaluate the
                                  point at; must be in the range [0, 1]. */
  ) const;
  /**
   * \brief Get a string representation of this curve.
   *
   * @return A string consisting of START-CONTROL-END.
   */
  std::string toString() const;
  /**
   * \brief Get this curve's distance intervals from a point.
   *
   * For a given point, moving along a curve (increasing its parameter) will
   * either result in points closer to or farther from the point. This
   * calculates every point where a curve stops getting closer and starts
   * getting farther from the point and vice versa. The result also includes
   * this curve's end points and their respective distances.
   *
   * @return A list of parameters for this curve (all in the interval [0, 1])
   * and their respective distances from the input point where the 'direction of
   * distance' changes relative to this curve's parameter. A quadratic Bezier
   * curve may have no more than 3 parameters for which its 'distance direction'
   * changes; this fact, paired with the inclusion of the endpoints totals no
   * more than 5 possible critical points.
   */
  CritsAndValues<5> getDistanceCritsAndValues(
      const Point2D &input /**< The point that distances on this curve are
                              measured from. */
  ) const;
  /**
   * \brief Get a point that is on a tangential straight line from this curve at
   * a parameter.
   *
   * At any valid parameter, a curve evaluates to a single point. Also, at this
   * point the curve has a slope and may be considered to have a direction,
   * which refers to the direction along its current slope it will move if the
   * parameter were to increase.
   *
   * This returns a point that is this curve's value at the input parameter,
   * shifted by a constant distance in the curve's direction.
   *
   * @return A point representing this curve's slope and direction.
   */
  Point2D getDirectionIndicator(
      const RealNum &parameter /**< The parameter for this curve to evaluate the
                                  direction at; must be in the range [0, 1]. */
  ) const;
  /**
   * \brief Get a numeric value indicating how 'curved' this curve is at a
   * given point.
   *
   * This is a ratio that represents the degree of this curve's concavity. The
   * ratio is not against this curve's parameter, instead, it is relative to the
   * coordinate space that the curve exists in. This means that any curve with a
   * higher value from this function is more concave, or 'curved' at the given
   * parameter than any other curve with a lower value.
   *
   * The result is always positive and not affected by the direction of a
   * curve's concavity. Therefore, the lowest possible return value for this
   * function would be to call it on a straight line.
   *
   * @return A numeric value indicating how concave/curved this curve is at the
   * input parameter. Always positive, always higher for more real curvature,
   * consistent across curves.
   */
  RealNum getConcavitySlope(
      const RealNum &parameter /**< The parameter to evaluate this curve's
                                  concavity. Must be in the range [0, 1]. */
  ) const;
  /**
   * \brief Get a point that indicates the direction of this curve's concavity.
   *
   * If a curve is not a straight line, it curves in a direction at a given
   * parameter. This function will return a point that results from shifting the
   * curve's value at the input parameter by a slope perpendicular to this
   * curve's slope at the parameter by a constant distance in the direction of
   * its concavity.
   *
   * For example, if a curve were to make a vertically symmetrical 'U' shape,
   * then its concavity indicator at parameter 0.5 would be the central point in
   * the curve shifted straight up.
   *
   * @return A point indicating the direction that this curve is bending in. If
   * this curve is a straight line, the value at the parameter is simply
   * returned with no shift.
   */
  Point2D getConcavityIndicator(
      const RealNum &parameter /**< The parameter to evaluate this curve's
                                  concavity. Must be in the range [0, 1]. */
  ) const;
  /**
   * \brief Get the ranges of 'perpendicular magnitudes' that this curve exists
   * in for a given slope.
   *
   * For a given slope, a Quadratic Bezier curve's 'perpendicular magnitude'
   * changes direction (goes from increasing to decreasing or vice versa)
   * relative to the curve's parameter at most once. The magnitude of this
   * curve's endpoints is also calculated, meaning that there can be at most 3
   * points of interest.
   *
   * \see Point2D::getPerpendicularMagnitude
   *
   * @return The list of Bezier parameters for endpoints and changes in
   * direction for this curve's 'perpendicular magnitude' for the input slope,
   * along with the associated 'perpendicular magnitude' values for each
   * parameter.
   */
  CritsAndValues<3> getPerpendicularMagnitudeCritsAndValues(
      const RealNum
          &slope /**< The slope to evaluate perpendicular magnitudes for. */
  ) const;
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
  /**
   * \brief test
   * @param intersection
   * @return
   */
  static bool isIntersectionInfinite(
      const StaticVector<std::pair<RealNum, RealNum>, 4> &intersection);
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
  Point2D getTestPointForEndIntersection(bool start) const;
  Point2D getTestPointForEndIntersection(const RealNum &myParam,
                                         const Point2D &myPoint,
                                         const Point2D &myStart,
                                         const RealNum &myRate) const;
  RealNum getDistanceConcavity(const RealNum &param,
                               const Point2D &fulcrum) const;

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
  static void checkParam(const RealNum &parameter);
  static RealNum getCWVerticalAngleForEndpoint(const BezierCurveQ &input,
                                               bool start, bool intoCurve);
};

std::ostream &operator<<(std::ostream &os, const BezierCurveQ &input);
} // namespace bezier_geometry

#endif
