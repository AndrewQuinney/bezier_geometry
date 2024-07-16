#ifndef Shape_HPP_
#define Shape_HPP_

#include "BezierCurveQ.hpp"
#include "BezierGeometryGlobal.hpp"

namespace bezier_geometry {
/**
 * \brief A two dimensional shape.
 *
 * A shape consists of at least 2 Bezier curves that form a closed loop, where
 * the start of the first curve equals the end of the last curve. This is the
 * only valid point of intersection between a shape's edges. Much like the
 * curves that make up its edges, a shape can perform move calculation and
 * collision detection, but with a spline of curves instead of individual
 * curves.
 */
class Shape {
public:
  /**
   * \brief A subset of a shape's edges.
   *
   * The edges contained are continuous on the perimeter of a shape.
   */
  struct EdgeSection {
    std::vector<const BezierCurveQ *> edges; /**< The edges in this subset. */
    RealNum
        startParam;   /**< The parameter for the first edge that represents the
                         start of this subset. Must be in the range [0, 1] */
    RealNum endParam; /**< The parameter for the last edge that represents the
                         end of this subset. Must be in the range [0, 1] */
  };
  /**
   * \brief The shape that represents an overlap between 2 other shapes.
   *
   * Any overlap between shapes is effectively another shape with a subset of
   * the edges from each. This represents only a single contiguous overlap,
   * meaning that if shapes overlap as more than one distinct 'overlap shape',
   * an instance of this represents only one.
   */
  class ShapeOverlap {
  public:
    /**
     * \brief Create an overlap.
     */
    ShapeOverlap(const std::vector<EdgeSection>
                     &sections /**< The subsets of the 2 overlapping shapes
                                  representing the consecutive edges from each
                                  shape that are inside the other shape. */
    );
    /**
     * \brief Get the edge sections for this overlap.
     *
     * @return The edge sections for this overlap.
     */
    const std::vector<EdgeSection> &getSections() const;

  private:
    std::vector<EdgeSection> sections;
  };
  /**
   * \brief The result of a 'shift against' operation.
   *
   * When one shape's shift result is calculated against another shape and it is
   * found that the other shape 'blocks' the shift, this contains the relevant
   * details.
   *
   * \see Shape::shiftAgainst
   * \see BezierCurveQ::shiftAgainst
   */
  struct ShiftAgainstResult {
    const BezierCurveQ
        *movingEdge; /**< The edge in this shape where it is blocked by the
                        input shape. nullptr if there is no block. */
    const BezierCurveQ
        *stationaryEdge; /**< The edge in the shape where it blocks this shape.
                            nullptr if there is no block. */
    BezierCurveQ::ShiftAgainstResult
        movingEdgeResult; /**< The block details from the shift result of the
                             two edges in this structure. */
  };
  /**
   * \brief The result of a 'rotate against' operation.
   *
   * When one shape's rotate result is calculated against another shape and it
   * is found that the other shape 'blocks' the rotation, this contains the
   * relevant details.
   *
   * \see Shape::rotateAgainst
   * \see BezierCurveQ::rotateAgainst
   */
  struct RotateAgainstResult {
    const BezierCurveQ
        *movingEdge; /**< The edge in this shape where it is blocked by the
                        input shape. nullptr if there is no block. */
    const BezierCurveQ
        *stationaryEdge; /**< The edge in the shape where it blocks this shape.
                            nullptr if there is no block. */
    BezierCurveQ::RotateAgainstResult
        movingEdgeResult; /**< The block details from the rotate result of the
                             two edges in this structure. */
  };

  /**
   * \brief Create a shape as a set of Bezier curves.
   *
   * The input points are interpreted to form a spline of curves in a closed
   * loop. If the curves that result from this intersect one another anywhere
   * except for the start of the first and the end of the last, it is an invalid
   * use case.
   */
  Shape(const std::vector<Point2D>
            &verticies, /**< The set of curve endpoints. Each entry is
                           considered both the end of a curve and the start of
                           the next curve. There must be at least 2 entries in
                           this set. */
        const std::vector<Point2D>
            &controlPoints /**< The set of curve control points. This must be
                              the same size as the verticies set. */
  );
  /**
   * \brief Get the highest X-coordinate this shape occupies.
   *
   * In a coordinate system where higher X-coordinates are 'right', this would
   * be the right side of the shape's bounding box.
   *
   * @return The highest X-coordinate this shape occupies.
   */
  const RealNum &getMaxX() const;
  /**
   * \brief Get the lowest X-coordinate this shape occupies.
   *
   * In a coordinate system where higher X-coordinates are 'right', this would
   * be the left side of the shape's bounding box.
   *
   * @return The lowest X-coordinate this shape occupies.
   */
  const RealNum &getMinX() const;
  /**
   * \brief Get the highest Y-coordinate this shape occupies.
   *
   * In a coordinate system where higher Y-coordinates are 'up', this would be
   * the top of the shape's bounding box.
   *
   * @return The highest Y-coordinate this shape occupies.
   */
  const RealNum &getMaxY() const;
  /**
   * \brief Get the lowest Y-coordinate this shape occupies.
   *
   * In a coordinate system where higher Y-coordinates are 'up', this would be
   * the bottom of the shape's bounding box.
   *
   * @return The lowest Y-coordinate this shape occupies.
   */
  const RealNum &getMinY() const;
  /**
   * \brief Get a version of this shape resulting from a rotation.
   *
   * Creates a rotated version of this shape by simply rotating all of the
   * edges' end and control points. \see Point2D::rotate
   *
   * @return Another shape that results from rotating this shape.
   */
  Shape
  rotate(const Point2D &fulcrum, /**< The point about which the rotation is to
                                    be performed. */
         const RealNum
             &angle /**< The angle (in degrees) to be rotated; an angle that is
                       a multiple of 360 (including 0) results in a point
                       equivalent to this point. Keeping with the behaviour of a
                       rotation matrix, negative angles are 'clockwise' and
                       positive angles are 'counterclockwise'. */
  ) const;
  /**
   * \brief Get a version of this shape resulting from a straight line shift.
   *
   * \see Point2D::shift
   *
   * @return Another shape that results from moving this shape in a straight
   * line.
   */
  Shape shift(
      const RealNum &distance, /**< The straight-line distance to be moved. */
      const RealNum &
          slope, /**< The slope along which the movement is to be calculated. */
      bool right, /**< The direction along the input slope. */
      bool up /**< The direction along the input slope, only used if the slope
                 is very large. */
  ) const;
  /**
   * \brief Calculate the overlap between this and another shape.
   *
   * 2 shapes that overlap can have their overlap represented as a set of other
   * shapes, which represent the area common to both shapes. Each entry in such
   * a set would consist of a contiguous set of edges from one shape and a
   * contiguous set of edges from the input shape, each representing the edges
   * from one curve inside the other.
   *
   * @return A set of overlaps, each representing an area that both shapes
   * occupy. Empty if there is no overlap.
   */
  std::vector<ShapeOverlap> getOverlap(
      const Shape
          &input /**< The shape to be checked for overlaps with this shape. */
  ) const;
  /**
   * \brief Tests if a given point lies within this shape.
   *
   * A point is considered to lie within this shape, on the edge of this shape,
   * or outside of this shape.
   *
   * @return 1 if the input point lies within this shape; 0 if the input point
   * lies on the edge of this shape; -1 if the input point is outside this
   * shape.
   */
  int pointInPolygon(const Point2D &testPoint /**< The point to check if it is
                                                 within this shape. */
  ) const;
  /**
   * \brief Get the edges for this shape.
   *
   * @return The set of edges that this shape consists of.
   */
  const std::vector<BezierCurveQ> &getEdges() const;
  /**
   * \brief Calculate the distance that this shape can be shifted until it
   * 'hits' the input shape.
   *
   * Given a shift direction, calculate how far this shape can be moved in a
   * straight line until it is 'blocked' by the input shape. This is effectively
   * an aggregate of the same method for individual curves, with some
   * optimization around relevant intervals for a given direction. \see
   * BezierCurveQ::shiftAgainst
   */
  void shiftAgainst(const Shape &input, /**< The shape that a shift against will
                                           be checked for 'blocks'. */
                    const RealNum &slope, /**< The slope along which the
                                             movement is to be calculated. */
                    bool right, /**< The direction along the input slope. */
                    bool up, /**< The direction along the input slope, only used
                                if the slope is very large. */
                    ShiftAgainstResult &output /**< The destination for the
                                                  calculated results. */
  ) const;
  /**
   * \brief Calculate the angle that this shape can be rotated until it 'hits'
   * the input shape.
   *
   * Given a fulcrum and rotation direction, calculate how much of an angle this
   * curve can be rotated until it is 'blocked' by the input curve. This is
   * effectively an aggregate of the same method for individual curves, with
   * some optimization around relevant intervals for a given direction. \see
   * BezierCurveQ::rotateAgainst
   */
  void
  rotateAgainst(const Shape &input, /**< The shape that a rotation against will
                                       be checked for 'blocks'. */
                const Point2D &fulcrum, /**< The fulcrum of the rotation. */
                bool clockwise,         /**< The direction of the rotation. */
                RotateAgainstResult
                    &output /**< The destination for the calculated results. */
  ) const;
  /**
   * \brief Simulate being 'pushed out of the way' by another shape's shift.
   *
   * This is a very specialized function related to a particular physics
   * simulation implementation.
   *
   * If this shape was moved out of the way due to another shape's movement and
   * the distance that this shape was moved was a guess, calculate how far to
   * reverse the guess after the the other shape has completed its movement. The
   * result of compensating by the amount indicated by this function is that
   * this shape would appear to have been moved sufficiently to allow for the
   * input shape's movement by a perfect amount.
   * \see DirectionalLogic
   *
   * @return The distance to reverse a previous shift.
   */
  RealNum shiftAgainstAfterShifting(
      const Shape
          &inputBeforeMove, /**< The pushing shape before it started the
                               movement for which this shape was pushed. */
      const RealNum &inputShiftDistance, /**< The distance that the pushing
                                            shape will move. */
      const RealNum
          &inputShiftSlope, /**< The slope that the pushing shape will move. */
      bool inputShiftRight, /**< The direction that the pushing shape will move
                               along its slope. */
      bool inputShiftUp,    /**< The direction that the pushing shape will move
                               along its slope. Only used if the slope is very
                               large. */
      const RealNum &myShiftSlope, /**< The slope that this shape was 'pushed'
                                      to move out of the way.*/
      bool myShiftRight, /**< The direction along the slope that this shape was
                            'pushed' to move out of the way.*/
      bool myShiftUp     /**< The direction along the slope that this shape was
                            'pushed' to move out of the way. Only used if the slope
                            is very large. */
  ) const;
  /**
   * \brief Simulate being 'pushed out of the way' by another shape's rotation.
   *
   * \see Shape::shiftAgainstAfterShifting
   * \see DirectionalLogic
   * \see ClockwiseLogic
   *
   * @return The distance to reverse a previous shift.
   */
  RealNum shiftAgainstAfterRotating(
      const Shape
          &inputBeforeMove, /**< The pushing shape before it started the
                               movement for which this shape was pushed. */
      const RealNum
          &inputRotationAngle, /**< The angle (in degrees) that the pushing
                                  shape will rotate. In the range [0, 360]. */
      const Point2D &inputRotationFulcrum, /**< The fulcrum that the pushing
                                              curve will rotate around. */
      bool inputRotationClockwise, /**< True if the pushing curve is rotating
                                      clockwise. */
      const RealNum &myShiftSlope, /**< The slope that this shape was 'pushed'
                                      to move out of the way.*/
      bool myShiftRight, /**< The direction along the slope that this shape was
                            'pushed' to move out of the way.*/
      bool myShiftUp     /**< The direction along the slope that this shape was
                            'pushed' to move out of the way. Only used if the slope
                            is very large. */
  ) const;
  /**
   * \brief Simulate being 'pushed out of the way' by another shape's shift.
   *
   * \see Shape::shiftAgainstAfterShifting
   * \see DirectionalLogic
   * \see ClockwiseLogic
   *
   * @return The angle to reverse a previous rotation.
   */
  RealNum rotateAgainstAfterShifting(
      const Shape
          &inputBeforeMove, /**< The pushing shape before it started the
                               movement for which this shape was pushed. */
      const RealNum &inputShiftDistance, /**< The distance that the pushing
                                            shape will move. */
      const RealNum
          &inputShiftSlope, /**< The slope that the pushing shape will move. */
      bool inputShiftRight, /**< The direction that the pushing shape will move
                               along its slope. */
      bool inputShiftUp,    /**< The direction that the pushing shape will move
                               along its slope. Only used if the slope is very
                               large. */
      const Point2D &myRotationFulcrum, /**< The point that this curve was
                                           rotated around as it was 'pushed'. */
      bool myRotationClockwise /**< True if this curve was rotated clockwise as
                                  it was 'pushed'. */
  ) const;
  /**
   * \brief Simulate being 'pushed out of the way' by another shape's rotation.
   *
   * \see Shape::shiftAgainstAfterShifting
   * \see DirectionalLogic
   * \see ClockwiseLogic
   *
   * @return The angle to reverse a previous rotation.
   */
  RealNum rotateAgainstAfterRotating(
      const Shape
          &inputBeforeMove, /**< The pushing shape before it started the
                               movement for which this shape was pushed. */
      const RealNum
          &inputRotationAngle, /**< The angle (in degrees) that the pushing
                                  shape will rotate. In the range [0, 360]. */
      const Point2D &inputRotationFulcrum, /**< The fulcrum that the pushing
                                              curve will rotate around. */
      bool inputRotationClockwise, /**< True if the pushing curve is rotating
                                      clockwise. */
      const Point2D &myRotationFulcrum, /**< The point that this shape was
                                           rotated about as it was 'pushed'. */
      bool myRotationClockwise /**< True if this shape was 'pushed' in a
                                  clockwise direction. */
  ) const;
  /**
   * \brief Get a string representation of this shape.
   *
   * \see BezierCurveQ::toString
   *
   * @return A string containing a comma-separated list of the edges in this
   * shape.
   */
  std::string toString() const;

  /**
   * \brief Create a shape that approximates a circle.
   *
   * Increasing the EDGES value will create a closer approximation of a circle.
   *
   * \see BezierCurveQ::circleArc
   *
   * @return A new shape with the specified number of edges that models a circle
   * as closely as a possible.
   */
  template <std::size_t EDGES>
  static Shape approximateCircle(
      const Point2D &centre, /**< The centre of the desired circle. */
      const RealNum &radius  /**< The radius of the desired circle. */
  ) {
    std::pair<std::vector<Point2D>, std::vector<Point2D>> circleArcPoints(
        BezierCurveQ::circleArc<EDGES>(centre, radius, 0, 360));
    circleArcPoints.first.pop_back();
    return Shape(circleArcPoints.first, circleArcPoints.second);
  }

private:
  std::vector<BezierCurveQ> edges;
  RealNum maxX;
  RealNum minX;
  RealNum maxY;
  RealNum minY;

  void addEdge(const Point2D &start, const Point2D &end,
               const Point2D &control);

  class EdgeTouch {
  public:
    enum TouchType { TOUCH, CROSS, PARALLEL_START };

    EdgeTouch(const TouchType &tt, const BezierCurveQ &myEdge,
              const RealNum &myEdgeParam, const BezierCurveQ &inputEdge,
              const RealNum &inputEdgeParam);
    const BezierCurveQ *getEdge(bool getMine, bool getFirst) const;
    const RealNum &getEdgeParam(bool getMine, bool getFirst) const;
    const TouchType &getType() const;
    void setSecondSet(const BezierCurveQ &mySecondEdge,
                      const RealNum &mySecondEdgeParam,
                      const BezierCurveQ &inputSecondEdge,
                      const RealNum &inputSecondEdgeParam);

  private:
    TouchType type;
    const BezierCurveQ *myEdge;
    RealNum myEdgeParam;
    const BezierCurveQ *inputEdge;
    RealNum inputEdgeParam;
    const BezierCurveQ *mySecondEdge;
    RealNum mySecondEdgeParam;
    const BezierCurveQ *inputSecondEdge;
    RealNum inputSecondEdgeParam;
  };
  typedef struct {
    const std::vector<BezierCurveQ> *sourceEdgeList;
    bool getMine;
    bool operator()(const EdgeTouch &touch1, const EdgeTouch &touch2) const;
  } TouchSort;
  static int addEdgeSet(const std::vector<BezierCurveQ> &shapeEdges,
                        int currentEdgeIdx, const EdgeTouch &startTouch,
                        const EdgeTouch &endTouch,
                        std::vector<EdgeSection> &result, bool getMine);
  static int PiPTestBetweenTouches(const EdgeTouch &touch1,
                                   const EdgeTouch &touch2,
                                   const Shape &inputShape, bool getMine);
  static bool crossesAtIntersection(
      const BezierCurveQ &shape1Edge1, const RealNum &shape1Edge1Param,
      const BezierCurveQ &shape1Edge2, const RealNum &shape1Edge2Param,
      const BezierCurveQ &shape2Edge1, const RealNum &shape2Edge1Param,
      const BezierCurveQ &shape2Edge2, const RealNum &shape2Edge2Param);
  static RealNum getTestParam(const BezierCurveQ &testCurve,
                              const RealNum &testCurveParam, bool goingTowards,
                              const BezierCurveQ &input);
  static bool nearStart(const BezierCurveQ &inputCurve,
                        const Point2D &inputPoint);
  static bool nearEnd(const BezierCurveQ &inputCurve,
                      const Point2D &inputPoint);
  static std::vector<EdgeSection>
  getInsideEdges(const Shape &testShape, const Shape &destinationShape,
                 std::vector<EdgeTouch> &touches, bool getMine);
};

std::ostream &operator<<(std::ostream &os, const Shape &input);
} // namespace bezier_geometry

#endif
