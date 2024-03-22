#ifndef Shape_HPP_
#define Shape_HPP_

#include "BezierCurveQ.hpp"
#include "BezierGeometryGlobal.hpp"

namespace bezier_geometry {
class Shape { // Any continuous, closed, 2-dimensional shape.
private:
  struct EdgeSection {
    std::vector<const BezierCurveQ *> edges;
    RealNum startParam;
    RealNum endParam;
  };

public:
  class ShapeOverlap {
  public:
    ShapeOverlap(const std::vector<EdgeSection> &sections);
    const std::vector<EdgeSection> &getSections() const;

  private:
    std::vector<EdgeSection> sections;
  };
  struct ShiftAgainstResult {
    const BezierCurveQ *movingEdge;
    const BezierCurveQ *stationaryEdge;
    BezierCurveQ::ShiftAgainstResult movingEdgeResult;
  };
  struct RotateAgainstResult {
    const BezierCurveQ *movingEdge;
    const BezierCurveQ *stationaryEdge;
    BezierCurveQ::RotateAgainstResult movingEdgeResult;
  };

  Shape(const std::vector<Point2D> &verticies,
        const std::vector<Point2D> &controlPoints);
  const RealNum &getMaxX() const;
  const RealNum &getMinX() const;
  const RealNum &getMaxY() const;
  const RealNum &getMinY() const;
  Shape rotate(const Point2D &fulcrum, const RealNum &angle) const;
  Shape shift(const RealNum &distance, const RealNum &slope, bool right,
              bool up) const;
  std::vector<ShapeOverlap> getOverlap(const Shape &input) const;
  int pointInPolygon(const Point2D &testPoint) const;
  const std::vector<BezierCurveQ> &getEdges() const;
  void shiftAgainst(const Shape &input, const RealNum &slope, bool right,
                    bool up, ShiftAgainstResult &output) const;
  void rotateAgainst(const Shape &input, const Point2D &fulcrum, bool clockwise,
                     RotateAgainstResult &output) const;
  RealNum shiftAgainstAfterShifting(const Shape &inputBeforeMove,
                                    const RealNum &inputShiftDistance,
                                    const RealNum &inputShiftSlope,
                                    bool inputShiftRight, bool inputShiftUp,
                                    const RealNum &myShiftSlope,
                                    bool myShiftRight, bool myShiftUp) const;
  RealNum shiftAgainstAfterRotating(const Shape &inputBeforeMove,
                                    const RealNum &inputRotationAngle,
                                    const Point2D &inputRotationFulcrum,
                                    bool inputRotationClockwise,
                                    const RealNum &myShiftSlope,
                                    bool myShiftRight, bool myShiftUp) const;
  RealNum rotateAgainstAfterShifting(const Shape &inputBeforeMove,
                                     const RealNum &inputShiftDistance,
                                     const RealNum &inputShiftSlope,
                                     bool inputShiftRight, bool inputShiftUp,
                                     const Point2D &myRotationFulcrum,
                                     bool myRotationClockwise) const;
  RealNum rotateAgainstAfterRotating(const Shape &inputBeforeMove,
                                     const RealNum &inputRotationAngle,
                                     const Point2D &inputRotationFulcrum,
                                     bool inputRotationClockwise,
                                     const Point2D &myRotationFulcrum,
                                     bool myRotationClockwise) const;
  std::string toString() const;

  template <std::size_t EDGES>
  static Shape approximateCircle(const Point2D &centre, const RealNum &radius) {
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
