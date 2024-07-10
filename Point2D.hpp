#ifndef Point2D_HPP_
#define Point2D_HPP_

#include <string>

#include "BezierGeometryGlobal.hpp"

/**
 * \defgroup ClockwiseLogic Clockwise Terminology
 *
 * The module often uses the terms 'clockwise' and 'counterclockwise' to
 * describe the direction of various rotation actions. Since these tend to
 * visual terms, they may be misinterpreted in different coordinate systems.
 * 'Clockwise' here refers to the visual term when increasing X-coordinates are
 * considered 'right' and increasing Y-coordinates are considered 'up'.
 * Conversely, decreasing X-coordinates are considered 'left' and decreasing
 * Y-coordinates are considered 'down'.
 *
 * More formally a clockwise rotation means:
 * * An increasing X-coordinate where the Y-coordinate is greater than that of
 * the rotation fulcrum.
 * * An increasing Y-coordinate where the X-coordinate is less than that of the
 * rotation fulcrum.
 * * The converse of the above.
 * @{
 */
/**@}*/
/**
 * \defgroup DirectionalLogic Directional Terminology
 *
 * The terms 'slope', 'right', and 'up' are used in relation to straight-line
 * directional translations of objects. Since these are visual terms, they may
 * be misinterpreted for different coordinate systems. To clarify, a module
 * considers these terms in relation to the coordinate system:
 * * 'Right' refers to an increasing X-coordinate.
 * * 'Up' refers to an increasing Y-coordinate.
 * * 'Slope' is the up/down change divided by the right/left change:
 * (y2-y1)/(x2-x1). This means that a slope of 0 means that there is no up/down
 * movement and a slope of infinity (or negative infinity) means that there is
 * no left/right movement.
 * @{
 */
/**@}*/
namespace bezier_geometry {
/**
 * \brief A single point in 2-dimensional space.
 *
 * A 2D cartesian point, representing an X and Y coordinate relative to an
 * origin.
 */
class Point2D {
public:
  /**
   * \brief Creates a point at the origin (0, 0).
   */
  Point2D();
  /**
   * \brief Creates a point at the input coordinates.
   */
  Point2D(const RealNum &x, /**< X-coordinate. */
          const RealNum &y /**< Y-coordinate. */);
  /**
   * \brief Returns the point's X-coordinate.
   *
   * @return The point's X-coordinate.
   */
  const RealNum &getX() const;
  /**
   * \brief Returns the point's Y-coordinate.
   *
   * @return The point's Y-coordinate.
   */
  const RealNum &getY() const;
  /**
   * \brief Get the distance between this point an the input point.
   *
   * The distance is calculated using the pythagorean theorem.
   *
   * @return The distance between this and the input point.
   */
  RealNum distanceFrom(const Point2D &input /**< The point from which this point's distance is to be calculated. */) const;
  /**
   * \brief Determines if this and the input point are approximately the same.
   *
   * Two points are considered 'equal' if the distance between them is
   * sufficiently small.
   * \see \ref SufficientlyClose
   * \see distanceFrom
   *
   * \return True if this and the input point are approximately the same.
   */
  bool operator==(
      const Point2D &input /**< The point that this is to be compared to. */)
      const;
  /**
   * \brief Determines if this and the input point are not approximately the
   * same.
   *
   * This is effectively the inverse of the == operator.
   * \see ::operator==
   *
   * \return True if this and the input point are approximately the same.
   */
  bool operator!=(const Point2D &input) const;
  /**
   * \brief Get the result of rotating this point about another point.
   *
   * Given an input point, calculate the point that would result from rotating
   * this about the input fulcrum. This is implemented via a 2D rotation matrix.
   * \see ClockwiseLogic
   *
   * \return A new point that would result from the input rotation parameters.
   */
  Point2D
  rotate(const Point2D &fulcrum, /**< The rotation fulcrum about which the
                                    rotation is to be calculated. */
         const RealNum
             &angle /**< The angle (in degrees) to be rotated; an angle that is
                       a multiple of 360 (including 0) results in a point
                       equivalent to this point. Keeping with the behaviour of a
                       rotation matrix, negative angles are 'clockwise' and
                       positive angles are 'counterclockwise'. */
  ) const;
  /**
   * \brief Get the result of moving this point by a given distance and
   * direction.
   *
   * Given a distance and direction, calculate the point that would result from
   * moving this point in a straight line in the given direction by the given
   * distance. \see \ref DirectionalLogic
   *
   * @return The point that would result from the given distance and direction.
   */
  Point2D shift(
      const RealNum &distance, /**< The straight-line distance to be moved. */
      const RealNum &
          slope, /**< The slope along which the movement is to be calculated. */
      bool right, /**< The direction along the input slope. */
      bool up /**< The direction along the input slope, only used if the slope
                 is very large. */
  ) const;
  /**
   * \brief Get a string representation of this point.
   *
   * The numeric values are truncated to a precision of 20 decimal places.
   *
   * @return This point in the form (X, Y).
   */
  std::string toString() const;
  /**
   * \brief Calculate the angle formed by this point, the fulcrum, a point
   * vertical of the fulcrum.
   *
   * Given an input fulcrum, calculate another point with the same X-coordinate
   * of fulcrum but with a higher Y-coordinate. The angle for which this
   * 'vertical' point must be rotated about the fulcrum (clockwise) to be
   * colinear with this point and the fulcrum is then calculated.
   * \see ClockwiseLogic
   * \see colinear
   *
   * @return The clockwise angle in degrees between a vertical line starting at
   * the fulcrum and the line formed between this point and the fulcrum. Never
   * negative.
   */
  RealNum
  getCWVerticalAngle(const Point2D &fulcrum /**< The fulcrum of rotation for
                                               which the angle is returned. */
  );

  /**
   * \brief Get the angle formed by 3 points.
   *
   * Given 3 points, determine the angle between them. Specifically, the angle
   * that one point must be rotated about another point in order to be colinear
   * with a third point. \see colinear
   * \see ClockwiseLogic
   *
   * @return The angle in radians that 'first' must be rotated about 'fulcrum'
   * to be colinear with 'second'.
   */
  static RealNum getAngleBetween(
      const Point2D &first, /**< The point for which the angle of rotation is to
                               be calculated. */
      const Point2D &second,  /**< The target point that will be colinear with
                                 the other two after the first's rotation. */
      const Point2D &fulcrum, /**< The fulcrum of rotation. */
      bool clockwise /**< True if the calculated angle must be for a clockwise
                        rotation; otherwise the counterclockwise angle is
                        calculated. */
  );
  /**
   * \brief Calculate the slope between 2 points.
   *
   * The order of parameters will not affect the resulting output.
   * \see DirectionalLogic
   *
   * @return The slope between the 2 input points.
   */
  static RealNum getSlopeBetween(const Point2D &first, /**< The first point. */
                                 const Point2D &second /**< The first point. */
  );
  /**
   * \brief Determine if 3 points are 'colinear'.
   *
   * Formally, 2D points are colinear if they all exist on any straight line.
   * This function implements this logic with a margin of error, meaning that 3
   * points are colinear if one of the following is true:
   * * Any pair of the input points are considered 'equal'.
   * * The slopes between 2 of the input points are sufficiently close AND the
   * perpendicular magnitude of the 3rd point is sufficiently close to that of
   * the other 2 along the slope formed by the other 2. This implementation
   * allows for a margin of error while also not creating exceptions for points
   * that are particularly close to one another vs those that are very far
   * apart. The ordering of the input parameters will not affect the outcome.
   * \see SufficientlyClose
   * \see DirectionalLogic
   * \see Point2D::operator==
   * \see getPerpendicularMagnitude
   *
   * @return True if the 3 input points are colinear.
   */
  static bool colinear(const Point2D &p1, /**< The first point. */
                       const Point2D &p2, /**< The second point. */
                       const Point2D &p3  /**< The third point. */
  );
  /**
   * \brief Get a point's minimum distance from a straight line going through
   * the origin with a given slope.
   *
   * It is often necessary to determine changes in distance in directions that
   * are not always aligned with the coordinate system (i.e. straight up/down,
   * or left/right). For example, a point whose 'perpendicular magnitude' is
   * very close to that of another along a slope could be considered to be a
   * potential collision for a straight-line shift; similarly a major difference
   * implies the opposite. The same can be said for intervals between points
   * when calculating areas of interest in collision detection.
   *
   * This uses a rotation matrix for the smallest angle necessary to make the
   * input slope 0. Vertical (infinite) slopes simply result in the input
   * point's X-coordinate.
   *
   * @return The closest distance between an input point and a straight line
   * going through the origin with a given slope.
   */
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
