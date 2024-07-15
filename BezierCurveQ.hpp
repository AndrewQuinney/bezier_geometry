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
  /**
   * \brief The result of a 'shift against' operation.
   *
   * This contains the result of the calculation as well as other useful data
   * that can be helpful for caching.
   *
   * \see BezierCurveQ::shiftAgainst
   */
  struct ShiftAgainstResult {
    RealNum distance; /**< The straight line distance that the curve can move
                         before being 'blocked' by the curve being moved
                         against. -1 if there is no block. */
    RealNum param; /**< The parameter for the shifting curve that represents the
                      point at which it is blocked. -1 if there is no block. */
    RealNum inputParam; /**< The parameter for the stationary curve that
                           represents the point where it blocks the shifting
                           curve. -1 if there is no block. */
    RealNum blockedCWVerticalAngleStart; /**< An angle in degrees with an upward
                                            vertical line starting at the
                                            blocking point that represents the
                                            start of the other directions where
                                            shifts would also be blocked by this
                                            point. -1 if there is no block. */
    RealNum
        blockedCWVerticalAngleEnd;    /**< An angle in degrees with an upward
                                         vertical line starting at the blocking
                                         point that represents the end of the other
                                         directions where shifts would also be
                                         blocked by this point. This is to be
                                         interpreted as clockwise of the start
                                         angle, though it may not be a larger
                                         number. -1 if there is no block. */
    bool infiniteInitialIntersection; /**< True if the shifting and the
                                         stationary curves have an infinite
                                         intersection before any move is
                                         attempted. \see
                                         BezierCurveQ::isIntersectionInfinite */
    bool atMyCrit;    /**< True if the blocking point is at a 'perpendicular
                         magnitude' critical point for the shift slope on the
                         shifting curve. \see Point2D::perpendicularMagnitude */
    bool atInputCrit; /**< True if the blocking point is at a 'perpendicular
                         magnitude' critical point for the shift slope on the
                         stationary curve. \see Point2D::perpendicularMagnitude
                       */
  };
  /**
   * \brief The result of a 'rotate against' operation.
   *
   * This contains the result of the calculation as well as other useful data
   * that can be helpful for caching.
   *
   * \see BezierCurveQ::rotateAgainst
   */
  struct RotateAgainstResult {
    RealNum angle; /**< The angle about the input fulcrum that the moving curve
                      could rotate before being 'blocked' by the stationary
                      curve. -1 if there is no block. */
    RealNum param; /**< The parameter for the rotating curve that represents the
                      point at which it is blocked. -1 if there is no block. */
    RealNum inputParam; /**< The parameter for the stationary curve that
                           represents the point where it blocks the rotating
                           curve. -1 if there is no block. */
    RealNum blockedCWVerticalAngleStart; /**< An angle in degrees with an upward
                                            vertical line starting at the
                                            blocking point that represents the
                                            start of the other directions where
                                            shifts would also be blocked by this
                                            point. -1 if there is no block. */
    RealNum
        blockedCWVerticalAngleEnd;    /**< An angle in degrees with an upward
                                         vertical line starting at the blocking
                                         point that represents the end of the other
                                         directions where shifts would also be
                                         blocked by this point. This is to be
                                         interpreted as clockwise of the start
                                         angle, though it may not be a larger
                                         number. -1 if there is no block. */
    bool infiniteInitialIntersection; /**< True if the rotating and the
                                         stationary curves have an infinite
                                         intersection before any move is
                                         attempted. */
    bool atMyCrit; /**< True if the blocking point is at a distance crit for the
                      rotating curve. \see Point2D::distanceFrom */
    bool atInputCrit; /**< True if the blocking point is at a distance crit for
                         the stationary curve. \see Point2D::distanceFrom */
  };
  /**
   * \brief A single interval on a curve representing a direction of rotation
   * about a fulcrum.
   *
   * As the Bezier parameter increases a curve may move in a clockwise direction
   * about a given point, counterclockwise, or neither in the case of a straight
   * line pointing at the fulcrum. An instance of this represents a single
   * interval along a curve where the curve moves in a single direction about a
   * point relative to its parameter. \see BezierCurveQ::getCWAngleIntervals
   */
  struct CWAngleInterval {
    RealNum startParam; /**< The Bezier parameter for the start of this
                           interval. Always in the range [0, 1] */
    RealNum startCWVerticalAngle; /**< The clockwise angle (in degrees) with a
                                     vertical line starting at the fulcrum where
                                     the interval starts. */
    RealNum
        endParam; /**< The Bezier parameter for the end of this interval; this
                     always indicates a point clockwise of the point represented
                     by the start parameter. Always in the range [0, 1] */
    RealNum endCWVerticalAngle; /**< The clockwise angle (in degrees) with a
                                   vertical line starting at the fulcrum where
                                   the interval ends. To be interpreted as
                                   clockwise of the start angle */
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
  /**
   * \brief Calculate the angle that this curve can be rotated until it 'hits'
   * the input arc.
   *
   * Given a fulcrum and rotation direction, calculate how much of an angle this
   * curve can be rotated until it is 'blocked' by the input arc.
   *
   * A 'block' is not necessarily a place where this curve would touch/intersect
   * after a rotation. Any such touch must sufficiently overlap the direction of
   * rotation to ensure that a 'graze' would not cause a block, such as one
   * curve simply moving tangentially to another.
   *
   * \see ClockwiseLogic
   * \see SufficientlyClose
   */
  void rotateAgainstCircleArc(
      const CircleArc &circleArc, /**< The arc that angles and blocking points
                                     are to be calculated against. */
      const Point2D &fulcrum,     /**< The fulcrum of the rotation. */
      bool clockwise,             /**< The direction of the rotation. */
      RealNum &outputAngle, /**< Destination for the calculated angle that this
                               curve could rotate until it is blocked by the
                               input arc. -1 if this rotation is not blocked. */
      RealNum &outputParam, /**< Destination for the parameter for this curve
                               that represents the blocking point. In the range
                               [0, 1] if there is a block; -1 if this rotation
                               is not blocked. */
      Point2D &outputCirclePoint /**< Destination for point on the input circle
                                    arc that blocks. This is not assigned if
                                    there is no block. */
  ) const;
  /**
   * \brief Calculate the distance that this curve can be shifted until it
   * 'hits' the input arc.
   *
   * Given a shift direction, calculate how far this
   * curve can be moved in a straight line until it is 'blocked' by the input
   * arc.
   *
   * A 'block' is not necessarily a place where this curve would touch/intersect
   * after a rotation. Any such touch must sufficiently overlap the direction of
   * rotation to ensure that a 'graze' would not cause a block, such as one
   * curve simply moving tangentially to another.
   *
   * \see DirectionalLogic
   * \see SufficientlyClose
   */
  void shiftAgainstCircleArc(
      const CircleArc &circleArc, /**< The arc that distances and blocking
                                     points are to be calculated against. */
      const RealNum &
          slope, /**< The slope along which the movement is to be calculated. */
      bool right, /**< The direction along the input slope that the shift is to
                     be calculated. */
      bool up, /**< The direction along the input slope, only used if the slope
                  is very large. */
      RealNum &outputDistance, /**< Destination for the calculated distance
                                  until this curve is blocked. -1 if this
                                  rotation is not blocked. */
      RealNum
          &outputParam, /**< Destination for this curve's parameter where it
                           would be blocked by the input arc. In the range [0,
                           1] if there is a block; -1 if there is no block. */
      Point2D &outputCirclePoint /**< Destination for the point on the input
                                    circle arc where the block will occur. This
                                    is not assigned if there is no block. */
  ) const;
  /**
   * \brief Determine if an intersection will block a straight line movement.
   *
   * Generally, if the input curve 'crosses' this curve at the input point there
   * is always a block. A 'cross' refers to when the curves touch, they do not
   * have 'sufficiently close' slopes, and the intersection is also not at
   * either curves' endpoint.
   *
   * However, if the curves just 'touch' (the reverse of the above 'cross'
   * conditions) at the input point, there may be no block, depending on the
   * input direction. For example, if one curve just touches a straight vertical
   * line at at it's leftmost point, then a shift anywhere to the right would
   * not be blocked by such an intersection. This is helpful when these curves
   * are part of a 2d spline shape and physical bodies are to be simulated.
   *
   * If the intersection is found to block, other useful data is returned,
   * including the 'blocked angle interval'. These values represent the other
   * directions that would also be blocked by the intersection. Useful for
   * result caching.
   *
   * \see DirectionalLogic
   * \see SufficientlyClose
   * \see ClockwiseLogic
   * \see curveIntersectionBlocksRotate
   * \see Point2D::getPerpendicularMagnitude
   *
   * @return True if the input curve blocks a straight line move in the input
   * direction at the input intersection point.
   */
  bool curveIntersectionBlocksShift(
      const RealNum &myParam, /**< The parameter for this curve where the
                                 intersection takes place. */
      const RealNum &
          slope, /**< The slope along which the movement is to be calculated. */
      bool right, /**< The direction along the input slope that the shift is to
                     be calculated. */
      bool up, /**< The direction along the input slope, only used if the slope
                  is very large. */
      const RealNum
          &inputParam, /**< The parameter for the input curve where the
                          intersection takes place. It is assumed that the input
                          curve's value at this parameter matches tha of this
                          curve at this curve's parameter. */
      const BezierCurveQ
          &input, /**< The other curve that is intersecting this curve. */
      const bool
          tolerateSmallMagnitudeOverlap, /**< Flag for considering a
                                            'sufficiently small' overlap in
                                            these curves' 'perpendicular
                                            magnitude' for the input slope to
                                            not block. Generally this should
                                            always be true except for when this
                                            method is recycled as a part of
                                            calculating a rotation block, where
                                            tolerances must be tighter. */
      RealNum
          &outputBlockedCWVerticalAngleStart, /**< Destination for the start of
                                                 the blocked interval if the
                                                 intersection is found to be
                                                 blocking. This is an angle (in
                                                 degrees) with a vertical line
                                                 starting at the intersection
                                                 point and extending 'upwards'.
                                                 -1 if this intersection is not
                                                 blocking. */
      RealNum
          &outputBlockedCWVerticalAngleEnd, /**< Destination for the end of the
                                               blocked interval if the
                                               intersection is found to be
                                               blocking. This is an angle (in
                                               degrees) with a vertical line
                                               starting at the intersection
                                               point and extending 'upwards'. -1
                                               if this intersection is not
                                               blocking. */
      bool &outputAtMyCrit,   /**< Destination for the flag indicating that the
                                 intersection point is at a 'perpendicular
                                 magnitude' critical point (change from increasing
                                 to decreasing or vice versa) on this curve. */
      bool &outputAtInputCrit /**< Destination for the flag indicating that the
                                 intersection point is at a 'perpendicular
                                 magnitude' critical point (change from
                                 increasing to decreasing or vice versa) on the
                                 input curve. */
  ) const;
  /**
   * \brief Determine if an intersection will block a rotation.
   *
   * Generally, if the input curve 'crosses' this curve at the input point there
   * is always a block. A 'cross' refers to when the curves touch, they do not
   * have 'sufficiently close' slopes, and the intersection is also not at
   * either curves' endpoint.
   *
   * However, if the curves just 'touch' (the reverse of the above 'cross'
   * conditions) at the input point, there may be no block, depending on the
   * input direction. For example, if one curve just touches a straight vertical
   * line at at it's leftmost point, then a clockwise rotation about a point far
   * below either curve would not be blocked by such an intersection. This is
   * helpful when these curves are part of a 2d spline shape and physical bodies
   * are to be simulated.
   *
   * If the intersection is found to block, other useful data is returned,
   * including the 'blocked angle interval'. These values represent the other
   * directions that would also be blocked by the intersection. Useful for
   * result caching.
   *
   * \see SufficientlyClose
   * \see ClockwiseLogic
   *
   * @return True if the input curve blocks a rotation about the input fulcrum
   * in the input direction at the input intersection point.
   */
  bool curveIntersectionBlocksRotate(
      const RealNum &myParam, /**< The parameter for this curve where the
                                 intersection takes place. */
      const CritsAndValues<5>
          &myDistanceCrits, /**< The parameters and associated distances between
                               this curve and the fulcrum where the rate of
                               change of this curve's distance changes
                               direction. */
      const BezierCurveQ &input, /**< The curve intersecting this curve. */
      const RealNum &inputParam, /**< The parameter for the input curve where
                                    the intersection takes place. */
      const CritsAndValues<5>
          &inputDistanceCrits, /**< The parameters and associated distances
                                  between the input curve and the fulcrum where
                                  the rate of change of the input curve's
                                  distance changes direction. */
      const Point2D &fulcrum,  /**< The fulcrum for the rotation. */
      const bool clockwise,    /**< The direction of the rotation.. */
      RealNum
          &outputBlockedCWVerticalAngleStart, /**< Destination for the start of
                                                 the blocked interval if the
                                                 intersection is found to be
                                                 blocking. This is an angle (in
                                                 degrees) with a vertical line
                                                 starting at the intersection
                                                 point and extending 'upwards'.
                                                 -1 if this intersection is not
                                                 blocking. */
      RealNum
          &outputBlockedCWVerticalAngleEnd, /**< Destination for the end of the
                                               blocked interval if the
                                               intersection is found to be
                                               blocking. This is an angle (in
                                               degrees) with a vertical line
                                               starting at the intersection
                                               point and extending 'upwards'. -1
                                               if this intersection is not
                                               blocking. */
      bool &outputAtMyCrit,   /**< Destination for the flag indicating that the
                                 intersection point is at a distance critical
                                 point (change from increasing to decreasing or
                                 vice versa) on this curve. */
      bool &outputAtInputCrit /**< Destination for the flag indicating that the
                                 intersection point is at a distance critical
                                 point (change from increasing to decreasing or
                                 vice versa) on the input curve. */
  ) const;
  /**
   * \brief Get the parameters for this curve that are along a slope from a
   * point.
   *
   * Given a point and slope, calculate the parameters for this curve that will
   * yield points that form the input slope with the input point. In other
   * words. If the input point were to be shifted along the input slope, find
   * the parameters for this curve where it could intersect. There may be no
   * intersections.
   *
   * \see DirectionalLogic
   *
   * @return A list of parameters for this curve all in the range [0, 1] where
   * the input point can match along the input slope.
   */
  StaticVector<RealNum, 2> pointShiftAgainstParams(
      const Point2D
          &input, /**< The point for which a shift is being calculated. */
      const RealNum &slope,  /**< The slope of the shift. */
      bool skipIntersections /**< The input point may lie along this curve. If
                                this is the case, setting this flag to true will
                                not include parameters for the input point. */
  ) const;
  /**
   * \brief Get the parameters for this curve for a given distance from a point.
   *
   * Given a fulcrum point and a desired distance, calculate the parameters for
   * this curve that result in points at the desired distance.
   *
   * @return A list of parameters, each in the range [startParam, endParam], for
   * this curve that yield points at the desired distance. Possibly empty.
   */
  StaticVector<RealNum, 4> getCurveDistanceParams(
      const Point2D &fulcrum,  /**< The point from which the desired distance is
                                  calculated. */
      const RealNum &distance, /**< The desired distance. */
      const RealNum &startParam, /**< Calculate the desired distance over a
                                    subset of this curve, starting at this
                                    value. Must be in the range [0, 1]. */
      const RealNum &endParam /**< Calculate the desired distance over a subset
                                 of this curve, ending at this value. Must be in
                                 the range [startParam, 1]. */
  ) const;
  /**
   * \brief Get a set of intervals for this curve for its angle ranges about a
   * fulcrum.
   *
   * Relative to a fulcrum, this curve will 'move', meaning that as its
   * parameter increases it generates points that are clockwise of previous
   * points, counterclockwise of previous points, or neither (in the case of
   * straight lines pointing at the fulcrum).
   *
   * This function gets these intervals, meaning that for every interval found
   * the parameter ranges and the angle values are added to the result. \see
   * ClockwiseLogic
   */
  void getCWAngleIntervals(
      const Point2D &fulcrum, /**< The fulcrum that the angles about are to be
                                 calculated. */
      StaticVector<CWAngleInterval, 4>
          &outputFirstSet, /**< Destination for the list of results. */
      StaticVector<CWAngleInterval, 4> &
          outputSecondSet /**< Destination for the second list of results. This
                             is only populated if the fulcrum is a point on this
                             curve and in that case, this represents the angle
                             intervals on the opposite side of the fulcrum. */
  ) const;
  /**
   * \brief Determine if two parameters on a curve represent points that have a
   * sufficiently small distance along the curve between them.
   *
   * Sufficiently close parameters are not necessarily parameter values that are
   * sufficiently close, but a function of the difference in the parameters and
   * the 'speed' at which a curve's calculated points change relative to the
   * parameters. \see SufficientlyClose
   *
   * @return True if the 2 parameters represent points with a sufficiently small
   * 'arc distance' along the curve between them.
   */
  bool sufficientlyCloseAlongCurve(
      const RealNum &curveParam, /**< The first parameter for this curve. */
      const RealNum &testParam   /**< The second parameter for this curve. */
  ) const;
  /**
   * \brief Get the Bezier parameter for this curve that will result in a given
   * point.
   *
   * This is effectively the reverse of \ref BezierCurveQ::valueAt.
   *
   * @return The parameter for this curve that results in the given point. In
   * the range [0, 1] if the input point is found along this curve, -1 if it is
   * not.
   */
  RealNum
  paramForPoint(const Point2D &input /**< The point that a parameter for this
                                        curve is to be found for. */
  ) const;

  /**
   * \brief Create a 'long' straight line.
   *
   * The resulting curve represents a straight line that is 50000 units long
   * with the given slope and the given point in the middle.
   * \see DirectionalLogic
   *
   * @return A straight line.
   */
  static BezierCurveQ longStraightLine(
      const RealNum &slope, /**< The desired slope of the resulting line. */
      const Point2D &point /**< The mid-point of the resulting straight line. */
  );
  /**
   * \brief Create a 'long' straight line.
   *
   * The resulting curve represents a straight line that is 25000 units long..
   *
   * @return A straight line, starting at a given point and extending in a
   * direction that causes it to cross the other given point.
   */
  static BezierCurveQ longStraightLine(
      const Point2D &start, /**< The start point of the desired line. */
      const Point2D &point  /**< Another point that lies along the line, not
                               necessarily an endpoint. */
  );
  /**
   * \brief Create a straight line.
   *
   * @return A straight line between the two input points.
   */
  static BezierCurveQ
  straightLine(const Point2D &s, /**< The start point for the desired curve. */
               const Point2D &e  /**< The end point for the desired curve. */
  );
  /**
   * \brief Determine if a result returned from \ref
   * BezierCurveQ::pointsOfIntersection represents an infinite intersection.
   *
   * An infinite intersection results from determining the intersection between
   * 2 curves that overlap and therefore intersect at an interval of parameters
   * instead of a set of points.
   *
   * An infinite intersection's first entry is a pair of NaN values, but its
   * second and third entries are the start and end parameters for the
   * overlapping interval between the two curves.
   *
   * @return True if the input intersection represents an infinite intersection.
   */
  static bool
  isIntersectionInfinite(const StaticVector<std::pair<RealNum, RealNum>, 4>
                             &intersection /**< The intersection to be tested
                                              for an infinite intersection. */
  );

  /**
   * \brief Get a spline of Bezier curves to approximate a circle arc.
   *
   * Bezier curves cannot model a circle arc, but approximations can be made by
   * using start and end points at equal distances from a fulcrum, and modeling
   * control points to ensure that a set of such curves would be 'continuous' at
   * each endpoint.
   * \see ClockwiseLogic
   *
   * @return A collection of points representing the Bezier curve input points
   * to create the arc approximation. The first entry is the curve endpoints,
   * the second entry are the curve control points. The first entry always has
   * one more entry than the second, since it also  encapsulates the end of the
   * last edge.
   */
  template <std::size_t EDGES>
  static typename std::enable_if<(EDGES > 1),
                                 std::pair<StaticVector<Point2D, EDGES + 1>,
                                           StaticVector<Point2D, EDGES>>>::type
  circleArc(const Point2D
                &fulcrum, /**< The center point for the desired circle arc. */
            const RealNum &radius, /**< The radius of the desirec circle arc. */
            const RealNum
                &startCWPositiveVerticalAngle, /**< The clocwise angle with a
                                                  vertical line that the circle
                                                  arc will start at. */
            const RealNum
                &cwAngleSize /**< The angle about the fulcrum that the circle
                                arc will extend in the clockwise direction. */
  ) {
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
