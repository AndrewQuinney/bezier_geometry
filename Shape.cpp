#include "Shape.hpp"

#include <unordered_map>

#include "GeometryUtil.hpp"

namespace bezier_geometry {
namespace {
bool compareSecond(const std::pair<RealNum, RealNum> &input1,
                   const std::pair<RealNum, RealNum> &input2) {
  return input1.second < input2.second;
}

class ValueCalculation {
public:
  class ResultHolder {
  protected:
    ResultHolder() {}
  };

  virtual ~ValueCalculation() = default;

  /*
   * Returns the parameters and magnitude values for the input curve where the
   * curve magnitude's rate of change is 0 relative to the curve parameter.
   * */
  virtual CritsAndValues
  getMagnitudeCritsAndValues(const BezierCurveQ &input) const = 0;

  /*
   * The input edge's slope is very close to the target direction, return true
   * if the curve extends in the target direction as its parameter increases.
   *
   * Assumes that this is being called for an edge intersection at the highest
   * magnitude of the shape the edges belong to.
   * */
  virtual bool isEdgeDirectionChangeRelevant(const BezierCurveQ &edge,
                                             const RealNum &param) const = 0;

  /*
   * Both input edges have an increasing magnitude towards their intersection
   * (for 'edgeBefore' its end, for 'edgeAfter' its start), return true if
   * 'edgeAfter' is relevant, which would imply that 'edgeBefore' is not.  The
   * input curves are assumed to be increasing at a rate such that their next
   * crits' magnitude relative to the target direction is sufficiently different
   * than that of their intersection.
   *
   * Assumes that this is being called for an edge intersection at the highest
   * magnitude of the shape the edges belong to.
   * */
  virtual bool isEdgeIntersectionDirectionChangeRelevant(
      const BezierCurveQ &edgeBefore,
      const CritsAndValues &edgeBeforeCritsAndValues,
      const BezierCurveQ &edgeAfter,
      const CritsAndValues &edgeAfterCritsAndValues) const = 0;

  /*
   * Calculate the direction required to shift the edge to a higher magnitude
   * while not moving it along the target direction.  This is used to test odd
   * cases in edge relevance.
   * */
  virtual void getIncreasingMagnitudePerpendicularShiftDirection(
      const BezierCurveQ &edge, const RealNum &param, RealNum &outputSlope,
      bool &outputRight, bool &outputUp) const = 0;

  /*
   * Returns the difference in 'range' between two points in the movement
   * described by this object.  If the moving edge point were to be moved as
   * described by this object in a number of units matching the return value of
   * this function, then the points would have no range difference (though they
   * would not necessarily be the same point).
   *
   * This assumes that the points passed are not special in any way for this
   * movement (i.e. not the fulcrum for a rotation).
   * */
  virtual RealNum rangeDifference(const Point2D &movingEdgePoint,
                                  const Point2D &stationaryEdgePoint) const = 0;

  /*
   * Returns the parameters for 'edge' that result the same magnitude as target
   * in the movement described by this object.
   *
   * As above, this assumes that the points passed are not special in any way
   * for this movement (i.e. not the fulcrum for a rotation).
   * */
  virtual std::vector<RealNum> sameMagnitude(const BezierCurveQ &edge,
                                             const Point2D &target) const = 0;

  /*
   * There is a block between two edges, but this block would not be detected by
   * comparing the edges individually, only by considering the edges as part of
   * an entire crit interval.  Update the calculation result, generally this
   * will not contain some information that would come from the edges detecting
   * the block.
   *
   * If the range between is equal to or greater than that of the result, then
   * the result is not updated.
   * */
  virtual bool updateResultForMagnitudeEdgeCase(
      const BezierCurveQ &movingEdge, const RealNum &movingEdgeParam,
      const BezierCurveQ &blockingEdge, const RealNum &blockingEdgeParam,
      const RealNum &rangeBetween, const ResultHolder &result) const = 0;

  /*
   * The input shapes have a significant magnitude overlap, calculate the
   * distance for a normal move and update the result if the distance is now the
   * smallest.
   *
   * Both of the inputs are assumed to be on an interval of relevant edges for
   * each curve in the movement described by this object, the stationary is
   * assumed to be in the path of this object's movement, and both shapes that
   * these edges belong to are assumed to have no significant preexisting
   * overlap.  Therefore, if there is no block due to an infinite initial
   * intersection between these 2 curves then it is considered a block as both
   * shapes extend in opposite directions from this intersection.  Similarly if
   * the internal block calculation of this method does not report a block,
   * assume it is a block on intersection since a block MUST exist here.
   *
   * If the computed range between is found to be equal (fuzzy compare) to the
   * current output range, it is not updated since it is assumed that higher
   * priority and therefore more relevant edge combinations are tested before
   * others.
   * */
  virtual bool updateResultForNormalMove(
      const BezierCurveQ &moving, const RealNum &movingLowerParam,
      const RealNum &movingHigherParam,
      const std::pair<RealNum, RealNum> &movingRange,
      const CritsAndValues &movingCritsAndValues,
      const BezierCurveQ &stationary, const RealNum &stationaryLowerParam,
      const RealNum &stationaryHigherParam,
      const std::pair<RealNum, RealNum> &stationaryRange,
      const CritsAndValues &stationaryCritsAndValues,
      const ResultHolder &result) const = 0;

  /*
   * Returns the 'range' along the movement described by this object that the
   * edge encompasses within the input parameters. A range represents a distance
   * along the movement described by this object, the returned pair consist of a
   * 'min range', meaning the point in the edge within the parameters whose
   * range is the farthest in the opposite direction of this object's movement
   * and a 'max range' which represents a point with the same requirements but
   * is farthest in the direction of this object's movement.
   *
   * While the concept of 'min range' and 'max range' may seem obvious, it can
   * also mean that the max range is a smaller number than the min range (for
   * example, consider ranges as rotation angles in [0, 360]).
   * */
  virtual std::pair<RealNum, RealNum>
  getRange(const BezierCurveQ &edge, const RealNum &startParam,
           const RealNum &endParam) const = 0;
};

class ShiftValueCalculation : public ValueCalculation {
public:
  class ShiftResultHolder : public ValueCalculation::ResultHolder {
  public:
    ShiftResultHolder(BezierCurveQ::ShiftAgainstResult &result)
        : result(result) {
      result.distance = -1;
      result.param = -1;
      result.inputParam = -1;
      result.blockedCWVerticalAngleStart = -1;
      result.blockedCWVerticalAngleEnd = -1;
      result.infiniteInitialIntersection = false;
      result.atMyCrit = false;
      result.atInputCrit = false;
    }

    BezierCurveQ::ShiftAgainstResult &result;
  };

  ShiftValueCalculation(const RealNum &slope, bool right, bool up)
      : slope(slope), right(right), up(up) {}

  CritsAndValues
  getMagnitudeCritsAndValues(const BezierCurveQ &input) const override {
    return input.getPerpendicularMagnitudeCritsAndValues(slope);
  }

  bool isEdgeDirectionChangeRelevant(const BezierCurveQ &edge,
                                     const RealNum &param)
      const override { // Note the assumptions this method makes in the doc for
    // the implemented interface.
    const RealNum TARGET_DISTANCE = 100;
    const Point2D targetDirectionPoint(
        edge.valueAt(param).shift(TARGET_DISTANCE, slope, right, up));
    return edge.getDirectionIndicator(param).distanceFrom(
               targetDirectionPoint) < TARGET_DISTANCE;
  }

  bool isEdgeIntersectionDirectionChangeRelevant(
      const BezierCurveQ &edgeBefore,
      const CritsAndValues &edgeBeforeCritsAndValues,
      const BezierCurveQ &edgeAfter,
      const CritsAndValues &edgeAfterCritsAndValues)
      const override { // Note the assumptions this method makes in the doc for
    // the implemented interface.
    (void)edgeBeforeCritsAndValues;
    (void)edgeAfterCritsAndValues;
    RealNum placeholder1, placeholder2;
    bool placeholder3, placeholder4;
    return edgeBefore.curveIntersectionBlocksShift(
        1, slope, right, up, 0, edgeAfter, false, placeholder1, placeholder2,
        placeholder3, placeholder4);
  }

  void getIncreasingMagnitudePerpendicularShiftDirection(
      const BezierCurveQ &edge, const RealNum &param, RealNum &outputSlope,
      bool &outputRight, bool &outputUp) const override {
    outputSlope = (-1.0) / slope;
    const Point2D point(edge.valueAt(param));
    Point2D directionPoint(point.shift(100, outputSlope, true, true));
    if (Point2D::getPerpendicularMagnitude(directionPoint, slope) <
        Point2D::getPerpendicularMagnitude(point, slope)) {
      directionPoint = directionPoint.rotate(point, 180);
    }
    outputRight = directionPoint.getX() >= point.getX();
    outputUp = directionPoint.getY() >= point.getY();
  }

  RealNum rangeDifference(const Point2D &movingEdgePoint,
                          const Point2D &stationaryEdgePoint) const override {
    const Point2D directionPoint(movingEdgePoint.shift(100, slope, right, up));
    const RealNum perpSlope = (-1.0) / slope;
    const RealNum movingEdgePointTestMag =
        Point2D::getPerpendicularMagnitude(movingEdgePoint, perpSlope);
    return (Point2D::getPerpendicularMagnitude(stationaryEdgePoint, perpSlope) -
            movingEdgePointTestMag) *
           (movingEdgePointTestMag < Point2D::getPerpendicularMagnitude(
                                         directionPoint, perpSlope)
                ? 1.0
                : (-1.0));
  }

  std::vector<RealNum> sameMagnitude(const BezierCurveQ &edge,
                                     const Point2D &target) const override {
    return edge.pointShiftAgainstParams(target, slope, false);
  }

  bool updateResultForMagnitudeEdgeCase(
      const BezierCurveQ &movingEdge, const RealNum &movingEdgeParam,
      const BezierCurveQ &blockingEdge, const RealNum &blockingEdgeParam,
      const RealNum &rangeBetween, const ResultHolder &result) const override {
    (void)movingEdge;
    (void)blockingEdge;
    const ShiftResultHolder *const shiftResultHolder =
        static_cast<const ShiftResultHolder *>(&result);
    if (shiftResultHolder->result.distance >= 0 &&
        (shiftResultHolder->result.distance < rangeBetween ||
         fuzzyEquals(shiftResultHolder->result.distance,
                     rangeBetween))) { // The fuzzy compare is here because
                                       // very close preexisting results are
                                       // considered more relevant.
      return false;
    }
    debug() << "ShiftValueCalculation::updateResultForMagnitudeEdgeCase -"
            << " rangeBetween: " << toString(rangeBetween) << std::endl;
    shiftResultHolder->result.distance = rangeBetween;
    shiftResultHolder->result.param = movingEdgeParam;
    shiftResultHolder->result.inputParam = blockingEdgeParam;
    shiftResultHolder->result.blockedCWVerticalAngleStart = -1;
    shiftResultHolder->result.blockedCWVerticalAngleEnd = -1;
    shiftResultHolder->result.infiniteInitialIntersection = false;
    shiftResultHolder->result.atMyCrit = false;
    shiftResultHolder->result.atInputCrit = false;
    return true;
  }

  bool updateResultForNormalMove(
      const BezierCurveQ &moving, const RealNum &movingLowerParam,
      const RealNum &movingHigherParam,
      const std::pair<RealNum, RealNum> &movingRange,
      const CritsAndValues &movingCritsAndValues,
      const BezierCurveQ &stationary, const RealNum &stationaryLowerParam,
      const RealNum &stationaryHigherParam,
      const std::pair<RealNum, RealNum> &stationaryRange,
      const CritsAndValues &stationaryCritsAndValues,
      const ResultHolder &result) const override {
    struct LocalFunctions {
      static bool
      shiftIncreasesRange(const RealNum &slope, const bool right,
                          const bool up) { // Assumes that the origin always has
        // a perpendicular magnitude of 0.
        return Point2D::getPerpendicularMagnitude(
                   Point2D(0, 0).shift(100, slope, right, up), (-1.0) / slope) >
               0;
      }

      static RealNum
      getMinShiftDistance(const std::pair<RealNum, RealNum> &movingRange,
                          const std::pair<RealNum, RealNum> &stationaryRange,
                          const RealNum &slope, const bool right,
                          const bool up) {
        /*
         * The passed range parameters are the result of calling 'getRange' on
         * this object, meaning that they are min/max pairs of perpendicular
         * magnitude values for the slope perpendicular to mine.  The distance
         * between intervals will dictate the MINIMUM shift distance between
         * these shapes.
         * */
        if (movingRange.first > stationaryRange.second) {
          if (!shiftIncreasesRange(slope, right, up)) {
            return movingRange.first - stationaryRange.second;
          } else if (!sufficientlyClose(
                         movingRange.first,
                         stationaryRange
                             .second)) { // The stationary edge is not
                                         // in the shift direction.
            return std::numeric_limits<RealNum>::infinity();
          }
        } else if (movingRange.second < stationaryRange.first) {
          if (shiftIncreasesRange(slope, right, up)) {
            return stationaryRange.first - movingRange.second;
          } else if (!sufficientlyClose(
                         movingRange.second,
                         stationaryRange.first)) { // The stationary edge is not
                                                   // in the shift direction.
            return std::numeric_limits<RealNum>::infinity();
          }
        } // Any other case means there is a touch or overlap in the edges'
          // range, meaning a minimum distance of 0.
        return 0;
      }
    };
    const ShiftResultHolder *const shiftResultHolder =
        static_cast<const ShiftResultHolder *>(&result);
    {
      const RealNum minShiftDistance = LocalFunctions::getMinShiftDistance(
          movingRange, stationaryRange, slope, right, up);
      debug() << "ShiftValueCalculation::updateResultForNormalMove -"
              << " minShiftDistance: " << toString(minShiftDistance)
              << std::endl;
      if (std::isinf(minShiftDistance) ||
          (shiftResultHolder->result.distance >= 0 &&
           (shiftResultHolder->result.distance < minShiftDistance ||
            fuzzyEquals(
                shiftResultHolder->result.distance,
                minShiftDistance)))) { // The fuzzy compare is here because
                                       // very close preexisting results are
                                       // considered more relevant.
        return false;
      }
    }
    BezierCurveQ::ShiftAgainstResult shiftOutput;
    moving.shiftAgainst(stationary, slope, right, up, shiftOutput);
    debug()
        << "ShiftValueCalculation::updateResultForNormalMove - shift result,"
        << " distance: " << toString(shiftOutput.distance)
        << " blockedCWVerticalAngleStart: "
        << toString(shiftOutput.blockedCWVerticalAngleStart)
        << " blockedCWVerticalAngleEnd: "
        << toString(shiftOutput.blockedCWVerticalAngleEnd)
        << " param: " << toString(shiftOutput.param)
        << " movingLowerParam: " << toString(movingLowerParam)
        << " movingHigherParam: " << toString(movingHigherParam)
        << " inputParam: " << toString(shiftOutput.inputParam)
        << " stationaryLowerParam: " << toString(stationaryLowerParam)
        << " stationaryHigherParam: " << toString(stationaryHigherParam)
        << std::endl;
    if (shiftOutput.distance >= 0 &&
        (shiftOutput.param >= movingLowerParam ||
         moving.sufficientlyCloseAlongCurve(shiftOutput.param,
                                            movingLowerParam)) &&
        (shiftOutput.param <= movingHigherParam ||
         moving.sufficientlyCloseAlongCurve(shiftOutput.param,
                                            movingHigherParam)) &&
        (shiftOutput.inputParam >= stationaryLowerParam ||
         stationary.sufficientlyCloseAlongCurve(shiftOutput.inputParam,
                                                stationaryLowerParam)) &&
        (shiftOutput.inputParam <= stationaryHigherParam ||
         stationary.sufficientlyCloseAlongCurve(shiftOutput.inputParam,
                                                stationaryHigherParam))) {
      /*
       * If the above statement is not satisfied, the block must exist on a
       * non-relevant portion of the moving and/or the stationary edge, there is
       * an infinite intersection (see below), or there is a block due to
       * intersection on the relevant portions of these curves that is simply
       * not detected at the edge scope.
       * */
      if (shiftResultHolder->result.distance < 0 ||
          (shiftResultHolder->result.distance > shiftOutput.distance &&
           !fuzzyEquals(
               shiftResultHolder->result.distance,
               shiftOutput.distance))) { // The fuzzy compare is here because
                                         // very close preexisting results are
                                         // considered more relevant.
        shiftResultHolder->result = shiftOutput;
        return true;
      }
    } else if (shiftOutput.infiniteInitialIntersection) {
      if (shiftResultHolder->result.distance < 0 ||
          (shiftResultHolder->result.distance > 0 &&
           !fuzzyEquals(
               shiftResultHolder->result.distance,
               0))) { // The fuzzy compare is here because very close
                      // preexisting results are considered more relevant.
        /*
         * From the documentation for this method - there MUST be a block here
         * since these are relevant edges in the movement described by this
         * object, if this was not a block due to an infinite initial
         * intersection (also no magnitude crits in either curve, otherwise an
         * infinite initial intersection blocks), backtrack slightly and
         * recalculate.
         * */
        RealNum backtrackDistance = 1.0;
        const RealNum movingSlope = moving.rateOfChangeAtParam(
            (movingLowerParam + movingHigherParam) / 2.0);
        RealNum acuteSlopeAngle =
            std::fabs(std::atan(movingSlope) - std::atan(slope));
        if (acuteSlopeAngle > M_PI_2) {
          acuteSlopeAngle = M_PI - acuteSlopeAngle;
        }
        if (acuteSlopeAngle < M_PI_4) {
          /*
           * The acute angle between slopes can never be considered 0 since this
           * method is not called unless both curves have a significant
           * magnitude overlap in this object's direction slope.
           * */
          backtrackDistance = backtrackDistance / std::sin(acuteSlopeAngle);
        }
        updateResultForNormalMove(
            moving.shift(backtrackDistance, slope, !right, !up),
            movingLowerParam, movingHigherParam, {0, 0}, movingCritsAndValues,
            stationary, stationaryLowerParam, stationaryHigherParam, {0, 0},
            stationaryCritsAndValues, ShiftResultHolder(shiftOutput));
        if (shiftOutput.distance < 0) {
          throw std::string("Infinite initial intersection on shift, but "
                            "backing up did not produce a result.") +
              " moving: " + moving.toString() +
              " movingLowerParam: " + toString(movingLowerParam) +
              " movingHigherParam: " + toString(movingHigherParam) +
              " stationary: " + stationary.toString() +
              " stationaryLowerParam: " + toString(stationaryLowerParam) +
              " stationaryHigherParam: " + toString(stationaryHigherParam) +
              " slope: " + toString(slope) +
              " right: " + (right ? "TRUE" : "FALSE") +
              " up: " + (up ? "TRUE" : "FALSE");
        }
        shiftOutput.distance = 0;
        shiftOutput.infiniteInitialIntersection = true;
        shiftResultHolder->result = shiftOutput;
        return true;
      }
    } else {
      /*
       * It was verified above (min shift distance caluclation) that the
       * stationary lies in the path of the shift - there MUST be a block here.
       * */
      for (const std::pair<RealNum, RealNum> &currentIntersection :
           moving.pointsOfIntersection(stationary)) {
        if ((currentIntersection.first >= movingLowerParam ||
             moving.sufficientlyCloseAlongCurve(movingLowerParam,
                                                currentIntersection.first)) &&
            (currentIntersection.first <= movingHigherParam ||
             moving.sufficientlyCloseAlongCurve(movingHigherParam,
                                                currentIntersection.first)) &&
            (currentIntersection.second >= stationaryLowerParam ||
             stationary.sufficientlyCloseAlongCurve(
                 stationaryLowerParam, currentIntersection.second)) &&
            (currentIntersection.second <= stationaryHigherParam ||
             stationary.sufficientlyCloseAlongCurve(
                 stationaryHigherParam, currentIntersection.second))) {
          return updateResultForMagnitudeEdgeCase(
              moving, currentIntersection.first, stationary,
              currentIntersection.second, 0, result);
        }
      }
      /*
       * It is possible to reach here because there could be another point on
       * each edge that blocks the shift but is outside the input parameter
       * ranges, causing the above search to be executed.  This is ok because
       * that implies that some other relevant section will result in a smaller
       * distance.
       * */
      // throw std::string("Could not find intersection for blocked shift.");
    }
    return false;
  }

  std::pair<RealNum, RealNum> getRange(const BezierCurveQ &edge,
                                       const RealNum &startParam,
                                       const RealNum &endParam) const override {
    const RealNum perpSlope = (-1.0) / slope;
    const RealNum startRange =
        Point2D::getPerpendicularMagnitude(edge.valueAt(startParam), perpSlope);
    const RealNum endRange =
        Point2D::getPerpendicularMagnitude(edge.valueAt(endParam), perpSlope);
    const auto inputParamRange = std::minmax(startRange, endRange);
    std::pair<RealNum, RealNum> result(inputParamRange.first,
                                       inputParamRange.second);
    for (const std::pair<RealNum, RealNum> &currentCrit :
         edge.getPerpendicularMagnitudeCritsAndValues(perpSlope)
             .critsAndValues) {
      if ((currentCrit.first > startParam) && (currentCrit.first < endParam)) {
        if (result.first > currentCrit.second) {
          result.first = currentCrit.second;
        } else if (result.second < currentCrit.second) {
          result.second = currentCrit.second;
        }
      }
    }
    return result;
  }

private:
  const RealNum slope;
  const bool right;
  const bool up;
};

class RotateValueCalculation : public ValueCalculation {
public:
  class RotateResultHolder : public ValueCalculation::ResultHolder {
  public:
    RotateResultHolder(BezierCurveQ::RotateAgainstResult &result)
        : result(result) {
      result.angle = -1;
      result.param = -1;
      result.inputParam = -1;
      result.blockedCWVerticalAngleStart = -1;
      result.blockedCWVerticalAngleEnd = -1;
      result.infiniteInitialIntersection = false;
      result.atMyCrit = false;
      result.atInputCrit = false;
    }

    BezierCurveQ::RotateAgainstResult &result;
  };

  RotateValueCalculation(const Point2D &fulcrum, bool clockwise)
      : fulcrum(fulcrum), clockwise(clockwise) {}

  CritsAndValues
  getMagnitudeCritsAndValues(const BezierCurveQ &input) const override {
    return input.getDistanceCritsAndValues(fulcrum);
  }

  /*
   * Because of the assumptions made by this method as outlined in the
   * implemented interface, the input parameter cannot be the rotation fulcrum.
   * */
  bool isEdgeDirectionChangeRelevant(const BezierCurveQ &edge,
                                     const RealNum &param) const override {
    const RealNum TARGET_DISTANCE = 100;
    const Point2D point(edge.valueAt(param));
    const Point2D targetDirectionPoint(point.shift(
        TARGET_DISTANCE, (-1.0) / Point2D::getSlopeBetween(fulcrum, point),
        clockwise == (point.getY() > fulcrum.getY()),
        clockwise == (point.getX() < fulcrum.getX())));
    return edge.getDirectionIndicator(param).distanceFrom(
               targetDirectionPoint) < TARGET_DISTANCE;
  }

  /*
   * Because of the assumptions made by this method as outlined in the
   * implemented interface, the input parameter cannot be the rotation fulcrum.
   * */
  bool isEdgeIntersectionDirectionChangeRelevant(
      const BezierCurveQ &edgeBefore,
      const CritsAndValues &edgeBeforeCritsAndValues,
      const BezierCurveQ &edgeAfter,
      const CritsAndValues &edgeAfterCritsAndValues) const override {
    RealNum placeholder1, placeholder2;
    bool placeholder3, placeholder4;
    return edgeBefore.curveIntersectionBlocksRotate(
        1, edgeBeforeCritsAndValues, edgeAfter, 0, edgeAfterCritsAndValues,
        fulcrum, clockwise, placeholder1, placeholder2, placeholder3,
        placeholder4);
  }

  void getIncreasingMagnitudePerpendicularShiftDirection(
      const BezierCurveQ &edge, const RealNum &param, RealNum &outputSlope,
      bool &outputRight, bool &outputUp) const override {
    const Point2D point(edge.valueAt(param));
    outputSlope = Point2D::getSlopeBetween(fulcrum, point);
    outputRight = point.getX() >= fulcrum.getX();
    outputUp = point.getY() >= fulcrum.getY();
  }

  RealNum rangeDifference(const Point2D &movingEdgePoint,
                          const Point2D &stationaryEdgePoint) const override {
    const RealNum result = Point2D::getAngleBetween(
        movingEdgePoint, stationaryEdgePoint, fulcrum, clockwise);
    if (sufficientlyClose(result, 360)) {
      return 0;
    }
    return result;
  }

  std::vector<RealNum> sameMagnitude(const BezierCurveQ &edge,
                                     const Point2D &target) const override {
    return edge.getCurveDistanceParams(fulcrum, target.distanceFrom(fulcrum), 0,
                                       1);
  }

  bool updateResultForMagnitudeEdgeCase(
      const BezierCurveQ &movingEdge, const RealNum &movingEdgeParam,
      const BezierCurveQ &blockingEdge, const RealNum &blockingEdgeParam,
      const RealNum &rangeBetween, const ResultHolder &result) const override {
    const RotateResultHolder *const rotateResultHolder =
        static_cast<const RotateResultHolder *>(&result);
    if ((rotateResultHolder->result.angle >= 0 &&
         (rotateResultHolder->result.angle < rangeBetween ||
          fuzzyEquals(rotateResultHolder->result.angle, rangeBetween))) ||
        movingEdge.valueAt(movingEdgeParam) == fulcrum ||
        blockingEdge.valueAt(blockingEdgeParam) ==
            fulcrum) { // The fuzzy compare is here because very close
                       // preexisting results are considered more relevant.
      debug() << "RotateValueCalculation::updateResultForMagnitudeEdgeCase - "
                 "skipping update,"
              << " current output angle: "
              << toString(rotateResultHolder->result.angle)
              << " rangeBetween: " << toString(rangeBetween) << std::endl;
      return false;
    }
    debug()
        << "RotateValueCalculation::updateResultForMagnitudeEdgeCase -"
        << " movingEdgeParam: " << toString(movingEdgeParam)
        << " blockingEdgeParam: " << toString(blockingEdgeParam)
        << " rangeBetween: " << toString(rangeBetween) << " moving distance: "
        << toString(movingEdge.valueAt(movingEdgeParam).distanceFrom(fulcrum))
        << " blocking distance: "
        << toString(
               blockingEdge.valueAt(blockingEdgeParam).distanceFrom(fulcrum))
        << std::endl;
    rotateResultHolder->result.angle = rangeBetween;
    rotateResultHolder->result.param = movingEdgeParam;
    rotateResultHolder->result.inputParam = blockingEdgeParam;
    rotateResultHolder->result.blockedCWVerticalAngleStart = -1;
    rotateResultHolder->result.blockedCWVerticalAngleEnd = -1;
    rotateResultHolder->result.infiniteInitialIntersection = false;
    rotateResultHolder->result.atMyCrit = false;
    rotateResultHolder->result.atInputCrit = false;
    return true;
  }

  bool updateResultForNormalMove(
      const BezierCurveQ &moving, const RealNum &movingLowerParam,
      const RealNum &movingHigherParam,
      const std::pair<RealNum, RealNum> &movingRange,
      const CritsAndValues &movingCritsAndValues,
      const BezierCurveQ &stationary, const RealNum &stationaryLowerParam,
      const RealNum &stationaryHigherParam,
      const std::pair<RealNum, RealNum> &stationaryRange,
      const CritsAndValues &stationaryCritsAndValues,
      const ResultHolder &result) const override {
    (void)movingCritsAndValues;
    (void)
        stationaryCritsAndValues; // Future use for passing to 'rotateAgainst'.
    struct LocalFunctions {
      static RealNum
      getMinRotateAngle(const std::pair<RealNum, RealNum> &movingRange,
                        const std::pair<RealNum, RealNum> &stationaryRange,
                        const bool clockwise) {
        /*
         * The input range parameters are the result of calling 'getRange' on
         * this object.  Therefore they are clockwise angles with the vertical
         * point from the fulcrum.  The first value in each pair is always
         * counterclockwise (or of equal vertical angle) to the second, and the
         * curve moves clockwise from the first value's parameter to that of the
         * second.
         * */
        if (movingRange.first <= movingRange.second &&
            stationaryRange.first <= stationaryRange.second) {
          if (movingRange.first > stationaryRange.second ||
              (sufficientlyClose(movingRange.first, stationaryRange.second) &&
               !sufficientlyClose(movingRange.second,
                                  stationaryRange.second))) {
            return std::max(
                0.0, !clockwise ? (movingRange.first - stationaryRange.second)
                                : (stationaryRange.first - movingRange.second +
                                   360.0));
          } else if (movingRange.second < stationaryRange.first ||
                     (sufficientlyClose(movingRange.second,
                                        stationaryRange.first) &&
                      !sufficientlyClose(movingRange.first,
                                         stationaryRange.first))) {
            return std::max(
                0.0, clockwise ? (stationaryRange.first - movingRange.second)
                               : (movingRange.first - stationaryRange.second +
                                  360.0));
          }
        } else if (movingRange.first <= movingRange.second) {
          if ((movingRange.first > stationaryRange.second ||
               (sufficientlyClose(movingRange.first, stationaryRange.second) &&
                !sufficientlyClose(movingRange.second,
                                   stationaryRange.second))) &&
              (movingRange.second < stationaryRange.first ||
               (sufficientlyClose(movingRange.second, stationaryRange.first) &&
                !sufficientlyClose(movingRange.first,
                                   stationaryRange.first)))) {
            return std::max(static_cast<RealNum>(0.0),
                            clockwise
                                ? (stationaryRange.first - movingRange.second)
                                : (movingRange.first - stationaryRange.second));
          }
        } else if (stationaryRange.first <= stationaryRange.second) {
          if ((movingRange.first > stationaryRange.second ||
               (sufficientlyClose(movingRange.first, stationaryRange.second) &&
                !sufficientlyClose(movingRange.first,
                                   stationaryRange.first))) &&
              (movingRange.second < stationaryRange.first ||
               (sufficientlyClose(movingRange.second, stationaryRange.first) &&
                !sufficientlyClose(movingRange.second,
                                   stationaryRange.second)))) {
            return std::max(static_cast<RealNum>(0.0),
                            clockwise
                                ? (stationaryRange.first - movingRange.second)
                                : (movingRange.first - stationaryRange.second));
          }
        } else if (!clockwise && (sufficientlyClose(movingRange.second + 360.0,
                                                    stationaryRange.first) &&
                                  !sufficientlyClose(movingRange.first,
                                                     stationaryRange.first))) {
          return std::max(static_cast<RealNum>(0.0),
                          movingRange.first - stationaryRange.second);
        } else if (clockwise &&
                   sufficientlyClose(movingRange.first,
                                     stationaryRange.second + 360.0) &&
                   !sufficientlyClose(movingRange.second,
                                      stationaryRange.second + 360.0)) {
          return std::max(static_cast<RealNum>(0.0),
                          stationaryRange.first - movingRange.second);
        }
        return 0;
      }
    };
    const RealNum minRotationAngle = LocalFunctions::getMinRotateAngle(
        movingRange, stationaryRange, clockwise);
    const RotateResultHolder *const rotateResultHolder =
        static_cast<const RotateResultHolder *>(&result);
    if (rotateResultHolder->result.angle >= 0 &&
        (minRotationAngle > rotateResultHolder->result.angle ||
         fuzzyEquals(minRotationAngle,
                     rotateResultHolder->result
                         .angle))) { // The fuzzy compare is here because very
                                     // close preexisting results are
                                     // considered more relevant.
      debug() << "RotateValueCalculation::updateResultForNormalMove - skipping "
                 "calculation,"
              << " current output angle: "
              << toString(rotateResultHolder->result.angle)
              << " minimum angle: " << toString(minRotationAngle) << std::endl;
      return false;
    }
    BezierCurveQ::RotateAgainstResult rotateOutput;
    moving.rotateAgainst(stationary, fulcrum, clockwise, rotateOutput);
    debug() << "RotateValueCalculation::updateResultForNormalMove -"
            << " moving: " << moving
            << " moving param: " << toString(rotateOutput.param)
            << " moving lower param: " << toString(movingLowerParam)
            << " moving higher param: " << toString(movingHigherParam)
            << " movingCritsAndValues: " << movingCritsAndValues.critsAndValues
            << " stationary: " << stationary
            << " stationary param: " << toString(rotateOutput.inputParam)
            << " stationary lower param: " << toString(stationaryLowerParam)
            << " stationary higher param: " << toString(stationaryHigherParam)
            << " stationaryCritsAndValues: "
            << stationaryCritsAndValues.critsAndValues
            << " angle: " << toString(rotateOutput.angle)
            << " previous output angle: "
            << toString(rotateResultHolder->result.angle)
            << " blockedCWVerticalAngleStart: "
            << toString(rotateOutput.blockedCWVerticalAngleStart)
            << " blockedCWVerticalAngleEnd: "
            << toString(rotateOutput.blockedCWVerticalAngleEnd) << std::endl;
    if (rotateOutput.angle < 0) {
      debug() << "moving crit intervals: "
              << moving.getDistanceCritsAndValues(fulcrum).critsAndValues
              << " stationary crit intervals: "
              << stationary.getDistanceCritsAndValues(fulcrum).critsAndValues
              << std::endl;
      throw std::string("No rotation block - this method assumes there is a "
                        "significant magnitude overlap.");
    }
    // The fuzzy compare is here because very close preexisting results are
    // considered more relevant.
    if ((rotateResultHolder->result.angle < 0 ||
         (rotateResultHolder->result.angle > rotateOutput.angle &&
          !fuzzyEquals(rotateResultHolder->result.angle,
                       rotateOutput.angle))) &&
        (!rotateOutput.infiniteInitialIntersection ||
         rotateOutput.angle < 180) &&
        (rotateOutput.param >= movingLowerParam ||
         moving.sufficientlyCloseAlongCurve(rotateOutput.param,
                                            movingLowerParam)) &&
        (rotateOutput.param <= movingHigherParam ||
         moving.sufficientlyCloseAlongCurve(rotateOutput.param,
                                            movingHigherParam)) &&
        (rotateOutput.inputParam >= stationaryLowerParam ||
         stationary.sufficientlyCloseAlongCurve(rotateOutput.inputParam,
                                                stationaryLowerParam)) &&
        (rotateOutput.inputParam <= stationaryHigherParam ||
         stationary.sufficientlyCloseAlongCurve(
             rotateOutput.inputParam,
             stationaryHigherParam))) { // This may have been an infinite
                                        // initial intersection, but that's ok
                                        // if the parameters are within range.
      rotateResultHolder->result = rotateOutput;
      debug() << "RotateValueCalculation::updateResultForNormalMove -"
              << " updated result: " << rotateResultHolder->result.angle
              << " moving edge point: "
              << moving.valueAt(rotateResultHolder->result.param)
              << " stationary edge point: "
              << stationary.valueAt(rotateResultHolder->result.inputParam)
              << " blockedCWVerticalAngleStart: "
              << toString(
                     rotateResultHolder->result.blockedCWVerticalAngleStart)
              << " blockedCWVerticalAngleEnd: "
              << toString(rotateResultHolder->result.blockedCWVerticalAngleEnd)
              << " initial intersection: "
              << moving.pointsOfIntersection(stationary)
              << " movingLowerParam: " << toString(movingLowerParam)
              << " movingHigherParam: " << toString(movingHigherParam)
              << " stationaryLowerParam: " << toString(stationaryLowerParam)
              << " stationaryHigherParam: " << toString(stationaryHigherParam)
              << std::endl;
      return true;
    } else if (rotateOutput
                   .infiniteInitialIntersection) { // An infinite initial
                                                   // intersection blocks if
                                                   // any of the overlap is
                                                   // within the input
                                                   // parameters.
      const auto intersection(moving.pointsOfIntersection(stationary));
      if (!BezierCurveQ::isIntersectionInfinite(intersection)) {
        throw std::string(
            "Rotate calculation reported an infinite initial intersection but "
            "that is inconsistent with at direct intersection calculation.");
      }
      debug() << "RotateValueCalculation::updateResultForNormalMove - infinite "
                 "initial intersection case,"
              << " intersection: " << intersection
              << " movingLowerParam: " << toString(movingLowerParam)
              << " movingHigherParam: " << toString(movingHigherParam)
              << " stationaryLowerParam: " << toString(stationaryLowerParam)
              << " stationaryHigherParam: " << toString(stationaryHigherParam)
              << std::endl;
      const RealNum stationaryLowerOverlapParam =
          std::min(intersection[1].second, intersection[2].second);
      const RealNum stationaryHigherOverlapParam =
          std::max(intersection[1].second, intersection[2].second);
      if (movingLowerParam < intersection[2].first &&
          !moving.sufficientlyCloseAlongCurve(movingLowerParam,
                                              intersection[2].first) &&
          movingHigherParam > intersection[1].first &&
          !moving.sufficientlyCloseAlongCurve(movingHigherParam,
                                              intersection[1].first) &&
          stationaryLowerParam < stationaryHigherOverlapParam &&
          !stationary.sufficientlyCloseAlongCurve(
              stationaryLowerParam, stationaryHigherOverlapParam) &&
          stationaryHigherParam > stationaryLowerOverlapParam &&
          !stationary.sufficientlyCloseAlongCurve(
              stationaryHigherParam,
              stationaryLowerOverlapParam)) { // The infinitely intersecting
                                              // portion at least partially
                                              // overlaps the intervals on each
                                              // input curve.
        rotateResultHolder->result = rotateOutput;
        if ((intersection[1].first > movingLowerParam ||
             moving.sufficientlyCloseAlongCurve(intersection[1].first,
                                                movingLowerParam)) &&
            (intersection[1].first < movingHigherParam ||
             moving.sufficientlyCloseAlongCurve(intersection[1].first,
                                                movingHigherParam)) &&
            (intersection[1].second > stationaryLowerParam ||
             stationary.sufficientlyCloseAlongCurve(intersection[1].second,
                                                    stationaryLowerParam)) &&
            (intersection[1].second < stationaryHigherParam ||
             stationary.sufficientlyCloseAlongCurve(intersection[1].second,
                                                    stationaryHigherParam))) {
          /*
           * The moving curve's lower parameter for the infinitely intersecting
           * portion is in the input interval and the corresponding parameter
           * for the stationary curve is also in the input interval.
           * */
          if ((intersection[2].first > movingLowerParam ||
               moving.sufficientlyCloseAlongCurve(intersection[2].first,
                                                  movingLowerParam)) &&
              (intersection[2].first < movingHigherParam ||
               moving.sufficientlyCloseAlongCurve(intersection[2].first,
                                                  movingHigherParam)) &&
              (intersection[2].second > stationaryLowerParam ||
               stationary.sufficientlyCloseAlongCurve(intersection[2].second,
                                                      stationaryLowerParam)) &&
              (intersection[2].second < stationaryHigherParam ||
               stationary.sufficientlyCloseAlongCurve(intersection[2].second,
                                                      stationaryHigherParam))) {
            /*
             * The moving curve's higher parameter for the infinitely
             * intersecting portion is in the input interval and the
             * corresponding parameter for the stationary curve is also in the
             * input interval.
             *
             * i.e. The entire infinitely intersecting portion is inside the
             * input interval.
             * */
            if (moving.valueAt(intersection[1].first).distanceFrom(fulcrum) >
                moving.valueAt(intersection[2].first).distanceFrom(fulcrum)) {
              rotateResultHolder->result.param =
                  std::max(std::min(intersection[1].first, movingHigherParam),
                           movingLowerParam);
              rotateResultHolder->result.inputParam = std::max(
                  std::min(intersection[1].second, stationaryHigherParam),
                  stationaryLowerParam);
            } else {
              rotateResultHolder->result.param =
                  std::max(std::min(intersection[2].first, movingHigherParam),
                           movingLowerParam);
              rotateResultHolder->result.inputParam = std::max(
                  std::min(intersection[2].second, stationaryHigherParam),
                  stationaryLowerParam);
            }
          } else {
            rotateResultHolder->result.param =
                std::max(std::min(intersection[1].first, movingHigherParam),
                         movingLowerParam);
            rotateResultHolder->result.inputParam = std::max(
                std::min(intersection[1].second, stationaryHigherParam),
                stationaryLowerParam);
          }
        } else {
          rotateResultHolder->result.param =
              std::max(std::min(intersection[2].first, movingHigherParam),
                       movingLowerParam);
          rotateResultHolder->result.inputParam =
              std::max(std::min(intersection[2].second, stationaryHigherParam),
                       stationaryLowerParam);
        }
        rotateResultHolder->result.angle = 0;
        rotateResultHolder->result.blockedCWVerticalAngleStart = -1;
        rotateResultHolder->result.blockedCWVerticalAngleEnd = -1;
        return true;
      }
    }
    return false;
  }

  std::pair<RealNum, RealNum> getRange(const BezierCurveQ &edge,
                                       const RealNum &startParam,
                                       const RealNum &endParam) const override {
    struct LocalFunctions {
      static RealNum getCWVerticalAngle(const Point2D &fulcrum,
                                        const Point2D &point) {
        return Point2D::getAngleBetween(
            Point2D(fulcrum.getX(), fulcrum.getY() + 100.0), point, fulcrum,
            true);
      }
    };
    /*
     * This method assumes that the interval defined by the input parameters
     * does not cross the rotation fulcrum.  Since the logic making use of this
     * divides the curve by distance crits, this requirement is satisfied.
     * */
    std::vector<BezierCurveQ::CWAngleInterval> intervalFirstSet,
        intervalSecondSet;
    edge.getCWAngleIntervals(fulcrum, intervalFirstSet, intervalSecondSet);
    RealNum startAngle = std::numeric_limits<RealNum>::quiet_NaN();
    RealNum endAngle = std::numeric_limits<RealNum>::quiet_NaN();
    for (const BezierCurveQ::CWAngleInterval &currentInterval :
         intervalFirstSet.empty()    ? intervalSecondSet
         : intervalSecondSet.empty() ? intervalFirstSet
         : std::max(intervalFirstSet.back().startParam,
                    intervalFirstSet.back().endParam) >= endParam ||
                 edge.valueAt(endParam) == fulcrum
             ? intervalFirstSet
             : intervalSecondSet) {
      const bool increasingParamCW =
          currentInterval.startParam < currentInterval.endParam;
      const RealNum currentIntervalLowerParam = increasingParamCW
                                                    ? currentInterval.startParam
                                                    : currentInterval.endParam;
      const RealNum currentIntervalHigherParam =
          increasingParamCW ? currentInterval.endParam
                            : currentInterval.startParam;
      if (currentIntervalLowerParam < endParam &&
          currentIntervalHigherParam > startParam) {
        const RealNum currentLowerParamAngle =
            currentIntervalLowerParam > startParam ||
                    edge.sufficientlyCloseAlongCurve(currentIntervalLowerParam,
                                                     startParam)
                ? increasingParamCW ? currentInterval.startCWVerticalAngle
                                    : currentInterval.endCWVerticalAngle
                : LocalFunctions::getCWVerticalAngle(fulcrum,
                                                     edge.valueAt(startParam));
        const RealNum currentHigherParamAngle =
            currentIntervalHigherParam < endParam ||
                    edge.sufficientlyCloseAlongCurve(currentIntervalHigherParam,
                                                     endParam)
                ? increasingParamCW ? currentInterval.endCWVerticalAngle
                                    : currentInterval.startCWVerticalAngle
                : LocalFunctions::getCWVerticalAngle(fulcrum,
                                                     edge.valueAt(endParam));
        const RealNum currentCWStartAngle = increasingParamCW
                                                ? currentLowerParamAngle
                                                : currentHigherParamAngle;
        const RealNum currentCWEndAngle = increasingParamCW
                                              ? currentHigherParamAngle
                                              : currentLowerParamAngle;
        if (std::isnan(startAngle)) {
          startAngle = currentCWStartAngle;
          endAngle = currentCWEndAngle;
        } else if (sufficientlyClose(startAngle, currentCWEndAngle)) {
          startAngle = currentCWStartAngle;
        } else if (sufficientlyClose(endAngle, currentCWStartAngle)) {
          endAngle = currentCWEndAngle;
        } else if ((currentCWStartAngle <= currentCWEndAngle) ==
                   (startAngle <= endAngle)) {
          startAngle = std::min(startAngle, currentCWStartAngle);
          endAngle = std::max(endAngle, currentCWEndAngle);
        } else if (startAngle <= endAngle) {
          if (endAngle >= currentCWStartAngle) {
            endAngle = currentCWEndAngle;
          } else if (startAngle <= currentCWEndAngle) {
            startAngle = currentCWStartAngle;
          }
        } else {
          if (currentCWEndAngle >= startAngle) {
            startAngle = currentCWStartAngle;
          } else if (currentCWStartAngle <= endAngle) {
            endAngle = currentCWEndAngle;
          }
        }
      }
    }
    {
      const Point2D origin(0, 0);
      const Point2D vertical(0, 100);
      if (vertical.rotate(origin, startAngle * (-1.0)) ==
          vertical.rotate(origin, endAngle * (-1.0))) {
        return {startAngle, startAngle};
      }
    }
    return {startAngle, endAngle};
  }

private:
  const Point2D &fulcrum;
  const bool clockwise;
};

struct CritInterval {
  struct CritIntervalEdgeEntry {
    const BezierCurveQ *edge;
    RealNum startParam;
    RealNum endParam;
    RealNum startParamMagnitude;
    RealNum endParamMagnitude;
    std::pair<RealNum, RealNum> range;
  };
  std::vector<CritIntervalEdgeEntry> edges;
  bool relevant;
};

std::vector<CritInterval>
getCritIntervals(const std::vector<BezierCurveQ> &edges,
                 const ValueCalculation &vc, const bool reverseRelevance,
                 std::unordered_map<const BezierCurveQ *, CritsAndValues>
                     &outputEdgeCritsAndValues) {
  struct LocalFunctions {
    static bool sameMagnitudes(
        const RealNum &input1,
        const RealNum
            &input2) { // Unless some kind of fuzzy compare is warranted in the
      // future, just use precise equality.
      return input1 == input2;
    }

    static bool compareStartMagnitude(const CritInterval &input1,
                                      const CritInterval &input2) {
      return input1.edges.front().startParamMagnitude <
             input2.edges.front().startParamMagnitude;
    }
  };
  /*********************************************************************************************************************/
  outputEdgeCritsAndValues.clear();
  std::vector<CritInterval> critIntervals;
  for (const BezierCurveQ &currentEdge :
       edges) { // Group consecutive edges by increasing/decreasing magnitude
                // intervals.
    outputEdgeCritsAndValues.insert(
        {&currentEdge, vc.getMagnitudeCritsAndValues(currentEdge)});
    const CritsAndValues *const currentCritsAndValues =
        &(outputEdgeCritsAndValues[&currentEdge]);
    for (std::vector<std::pair<RealNum, RealNum>>::const_iterator currentCrit =
             currentCritsAndValues->critsAndValues.cbegin() + 1;
         currentCrit != currentCritsAndValues->critsAndValues.cend();
         currentCrit++) {
      const CritInterval::CritIntervalEdgeEntry newEntry(
          {&currentEdge, (currentCrit - 1)->first, currentCrit->first,
           (currentCrit - 1)->second, currentCrit->second,
           vc.getRange(currentEdge, (currentCrit - 1)->first,
                       currentCrit->first)});
      if (critIntervals.empty() ||
          (!LocalFunctions::sameMagnitudes(
               critIntervals.back().edges.front().startParamMagnitude,
               critIntervals.back().edges.back().endParamMagnitude) &&
           !LocalFunctions::sameMagnitudes(newEntry.startParamMagnitude,
                                           newEntry.endParamMagnitude) &&
           (critIntervals.back().edges.front().startParamMagnitude <
            critIntervals.back().edges.back().endParamMagnitude) !=
               (newEntry.startParamMagnitude <
                newEntry.endParamMagnitude))) { // New interval.
        critIntervals.push_back({{newEntry}, false});
      } else if (critIntervals.back().edges.back().edge == &currentEdge) {
        /*
         * This case can happen when an edge has a magnitude crit but the rate
         * of magnitude change is so small and the crit is close enough to an
         * endpoint that one side of the crit has an equal perpendicular
         * magnitude.  Without this case, the edge would be added twice.
         *
         * Generally, this is a rare occurrence.
         * */
        critIntervals.back().edges.back().endParam = newEntry.endParam;
        critIntervals.back().edges.back().endParamMagnitude =
            newEntry.endParamMagnitude;
        critIntervals.back().edges.back().range = vc.getRange(
            currentEdge, critIntervals.back().edges.back().startParam,
            critIntervals.back().edges.back().endParam);
      } else { // Add to newest interval.
        critIntervals.back().edges.push_back(newEntry);
      }
    }
  }
  if (critIntervals.size() > 1 &&
      (critIntervals.front().edges.front().startParamMagnitude <
       critIntervals.front().edges.back().endParamMagnitude) ==
          (critIntervals.back().edges.front().startParamMagnitude <
           critIntervals.back()
               .edges.back()
               .endParamMagnitude)) { // The first interval's first edge will
                                      // never match the last interval's last
                                      // edge.
    // critIntervals.back().edges.push_back(critIntervals.front().edges);
    critIntervals.back().edges.insert(critIntervals.back().edges.end(),
                                      critIntervals.front().edges.begin(),
                                      critIntervals.front().edges.end());
    critIntervals.erase(critIntervals.begin());
  }
  /*
   * Determine which groups are relevant.
   * */
  {
    std::vector<CritInterval>::iterator maxMagnitude =
        std::minmax_element(critIntervals.begin(), critIntervals.end(),
                            &LocalFunctions::compareStartMagnitude)
            .second;
    {
      const std::vector<CritInterval>::iterator previousInterval =
          maxMagnitude == critIntervals.begin() ? critIntervals.end() - 1
                                                : maxMagnitude - 1;
      if (previousInterval->edges.back().edge ==
          maxMagnitude->edges.front()
              .edge) { // Magnitude crit is inside a curve - use value calc to
                       // determine relevance of in-curve direction change.
        maxMagnitude->relevant = vc.isEdgeDirectionChangeRelevant(
            *(previousInterval->edges.back().edge),
            previousInterval->edges.back().endParam);
      } else {
        const CritsAndValues *const previousEdgeCrits =
            &(outputEdgeCritsAndValues[previousInterval->edges.back().edge]);
        const CritsAndValues *const currentEdgeCrits =
            &(outputEdgeCritsAndValues[maxMagnitude->edges.front().edge]);
        const RealNum previousClosestCritMagnitude =
            (previousEdgeCrits->critsAndValues.cend() - 2)->second;
        const bool previousParallel =
            sufficientlyClose(previousEdgeCrits->critsAndValues.back().second,
                              previousClosestCritMagnitude);
        const RealNum currentClosestCritMagnitude =
            (currentEdgeCrits->critsAndValues.cbegin() + 1)->second;
        const bool currentParallel =
            sufficientlyClose(currentEdgeCrits->critsAndValues.front().second,
                              currentClosestCritMagnitude);
        if (!previousParallel && !currentParallel) {
          maxMagnitude->relevant = vc.isEdgeIntersectionDirectionChangeRelevant(
              *(previousInterval->edges.back().edge), *previousEdgeCrits,
              *(maxMagnitude->edges.front().edge), *currentEdgeCrits);
        } else if (currentParallel && previousParallel) {
          const bool currentGoingInDirection = vc.isEdgeDirectionChangeRelevant(
              *(maxMagnitude->edges.front().edge), 0);
          /*
           * The call to isEdgeDirectionChangeRelevant will return true if the
           * curve moves in the target direction as its parameter increases.  If
           * the test parameter is for the end of the curve then interpretation
           * of this value must be reversed since the only direction is for a
           * lower parameter.
           * */
          if (currentGoingInDirection ==
              vc.isEdgeDirectionChangeRelevant(
                  *(previousInterval->edges.back().edge),
                  1)) { // The curves move in opposite directions.
            maxMagnitude->relevant = currentGoingInDirection;
          } else if ((previousEdgeCrits->critsAndValues.back().second >
                      previousClosestCritMagnitude) !=
                     (currentEdgeCrits->critsAndValues.front().second >
                      currentClosestCritMagnitude)) {
            maxMagnitude->relevant =
                (currentEdgeCrits->critsAndValues.front().second >
                 currentClosestCritMagnitude) == currentGoingInDirection;
          } else {
            RealNum testSlope;
            bool testRight, testUp;
            vc.getIncreasingMagnitudePerpendicularShiftDirection(
                *(previousInterval->edges.back().edge), 1, testSlope, testRight,
                testUp);
            RealNum placeholder1, placeholder2;
            bool placeholder3, placeholder4;
            maxMagnitude->relevant =
                previousInterval->edges.back()
                    .edge->curveIntersectionBlocksShift(
                        1, testSlope, testRight, testUp, 0,
                        *(maxMagnitude->edges.front().edge), false,
                        placeholder1, placeholder2, placeholder3,
                        placeholder4) == currentGoingInDirection;
          }
        } else if (currentParallel) {
          maxMagnitude->relevant = vc.isEdgeDirectionChangeRelevant(
              *(maxMagnitude->edges.front().edge), 0);
        } else {
          /*
           * See above comment about interepreting this value for the end of the
           * curve.  This means the previous edge is moving away from the target
           * direction.
           * */
          maxMagnitude->relevant = vc.isEdgeDirectionChangeRelevant(
              *(previousInterval->edges.back().edge), 1);
        }
      }
    }
    if (reverseRelevance) {
      maxMagnitude->relevant = !maxMagnitude->relevant;
    }
    for (std::vector<CritInterval>::iterator updateInterval =
             (maxMagnitude + 1) == critIntervals.end() ? critIntervals.begin()
                                                       : (maxMagnitude + 1);
         updateInterval != maxMagnitude;
         updateInterval = (updateInterval + 1) == critIntervals.end()
                              ? critIntervals.begin()
                              : (updateInterval + 1)) {
      std::vector<CritInterval>::iterator previousUpdateInterval =
          updateInterval == critIntervals.begin() ? (critIntervals.end() - 1)
                                                  : (updateInterval - 1);
      updateInterval->relevant = !(previousUpdateInterval->relevant);
    }
  }
  /*
   * Transfer any edges that have magnitudes that are all precisely along the
   * edge magnitudes (closer than 'sufficiently close') of the relevant
   * intervals to the next irrelevant interval.
   * */
  if (critIntervals.size() > 1) {
    for (std::vector<CritInterval>::iterator currentInterval =
             critIntervals.begin();
         currentInterval != critIntervals.end(); currentInterval++) {
      if (!currentInterval->relevant) {
        continue;
      }
      {
        const std::vector<CritInterval>::iterator previousInterval =
            currentInterval == critIntervals.begin() ? (critIntervals.end() - 1)
                                                     : (currentInterval - 1);
        if (previousInterval->edges.back().edge !=
            currentInterval->edges.front().edge) {
          std::vector<CritInterval::CritIntervalEdgeEntry>::iterator
              currentEdge = currentInterval->edges.begin();
          for (; LocalFunctions::sameMagnitudes(
                   currentInterval->edges.front().startParamMagnitude,
                   currentEdge->endParamMagnitude);
               currentEdge++) {
            previousInterval->edges.push_back(*currentEdge);
          }
          currentInterval->edges.erase(currentInterval->edges.begin(),
                                       currentEdge);
        }
      }
      {
        const std::vector<CritInterval>::iterator nextInterval =
            currentInterval == (critIntervals.end() - 1)
                ? (critIntervals.begin())
                : (currentInterval + 1);
        if (nextInterval->edges.front().edge !=
            currentInterval->edges.back().edge) {
          std::vector<CritInterval::CritIntervalEdgeEntry>::iterator erasePtr =
              currentInterval->edges.end();
          for (std::vector<CritInterval::CritIntervalEdgeEntry>::
                   reverse_iterator currentEdge =
                       currentInterval->edges.rbegin();
               LocalFunctions::sameMagnitudes(
                   currentInterval->edges.back().endParamMagnitude,
                   currentEdge->startParamMagnitude);
               currentEdge++, erasePtr--) {
            nextInterval->edges.insert(nextInterval->edges.begin(),
                                       *currentEdge);
          }
          currentInterval->edges.erase(erasePtr, currentInterval->edges.end());
        }
      }
    }
  }
  return critIntervals;
}

/*
 * Assumes that there is no preexisting significant overlap between the shapes.
 * */
void moveAgainst(const ValueCalculation &vc, const Shape &moving,
                 const Shape &stationary,
                 const ValueCalculation::ResultHolder &resultHolder,
                 const BezierCurveQ *&outputMovingEdge,
                 const BezierCurveQ *&outputStationaryEdge) {
  struct MagnitudeRange {
    RealNum lowerMagnitude;
    RealNum lowerMagnitudeParam;
    RealNum higherMagnitude;
    RealNum higherMagnitudeParam;
  };
  struct IntervalCombo {
    const CritInterval *movingInterval;
    const CritInterval *stationaryInterval;
    RealNum distanceBetween;

    bool operator<(const IntervalCombo &input) const {
      return distanceBetween < input.distanceBetween;
    }
  };
  struct EdgeCombo {
    const CritInterval::CritIntervalEdgeEntry *movingEdge;
    const CritInterval::CritIntervalEdgeEntry *stationaryEdge;
    MagnitudeRange movingEdgeMagnitudeRange;
    MagnitudeRange stationaryEdgeMagnitudeRange;
    RealNum overlapSize;
    bool edgeCaseMovingLowerStationaryHigher;
    bool edgeCaseMovingHigherStationaryLower;

    bool operator<(
        const EdgeCombo &input) const { // Descending order of overlap size.
      return overlapSize > input.overlapSize;
    }
  };
  struct LocalFunctions {
    static MagnitudeRange
    getMagnitudeRange(const CritsAndValues &critsAndvalues,
                      const RealNum &minParam, const RealNum &maxParam) {
      MagnitudeRange result({std::numeric_limits<RealNum>::quiet_NaN(),
                             std::numeric_limits<RealNum>::quiet_NaN(),
                             std::numeric_limits<RealNum>::quiet_NaN(),
                             std::numeric_limits<RealNum>::quiet_NaN()});
      for (const std::pair<RealNum, RealNum> &currentCrit :
           critsAndvalues.critsAndValues) {
        if ((fuzzyEquals(currentCrit.first, minParam) ||
             currentCrit.first > minParam) &&
            (fuzzyEquals(currentCrit.first, maxParam) ||
             currentCrit.first < maxParam)) {
          if (std::isnan(result.lowerMagnitude) ||
              result.lowerMagnitude > currentCrit.second) {
            result.lowerMagnitude = currentCrit.second;
            result.lowerMagnitudeParam = currentCrit.first;
          }
          if (std::isnan(result.higherMagnitude) ||
              result.higherMagnitude < currentCrit.second) {
            result.higherMagnitude = currentCrit.second;
            result.higherMagnitudeParam = currentCrit.first;
          }
        }
      }
      return result;
    }

    static std::pair<RealNum, RealNum> getSmallestRangeDifference(
        const bool matchingEdgeIsMoving, const BezierCurveQ &matchingEdge,
        const RealNum &matchingEdgeMinParam,
        const RealNum &matchingEdgeMaxParam,
        const BezierCurveQ &targetMagnitudeEdge,
        const RealNum &targetMagnitudeEdgeParam, const ValueCalculation &vc) {
      std::pair<RealNum, RealNum> result(
          std::numeric_limits<RealNum>::quiet_NaN(),
          std::numeric_limits<RealNum>::quiet_NaN());
      const Point2D targetMagnitudePoint(
          targetMagnitudeEdge.valueAt(targetMagnitudeEdgeParam));
      for (const RealNum &currentParam :
           vc.sameMagnitude(matchingEdge, targetMagnitudePoint)) {
        debug() << "getSmallestRangeDifference -"
                << " same magnitude param: " << toString(currentParam)
                << " target magnitude edge point: "
                << targetMagnitudeEdge.valueAt(targetMagnitudeEdgeParam)
                << " matching edge point: "
                << matchingEdge.valueAt(currentParam) << std::endl;
        if ((currentParam < matchingEdgeMinParam &&
             !matchingEdge.sufficientlyCloseAlongCurve(currentParam,
                                                       matchingEdgeMinParam)) ||
            (currentParam > matchingEdgeMaxParam &&
             !matchingEdge.sufficientlyCloseAlongCurve(currentParam,
                                                       matchingEdgeMaxParam))) {
          continue;
        }
        const RealNum currentRangeDifference =
            matchingEdgeIsMoving
                ? vc.rangeDifference(matchingEdge.valueAt(currentParam),
                                     targetMagnitudePoint)
                : vc.rangeDifference(targetMagnitudePoint,
                                     matchingEdge.valueAt(currentParam));
        if (currentRangeDifference < 0) {
          if (sufficientlySmall(currentRangeDifference)) {
            result.second = 0;
            result.first = currentParam;
            break;
          }
        } else if (std::isnan(result.second) ||
                   result.second > currentRangeDifference) {
          result.second = currentRangeDifference;
          result.first = currentParam;
        }
      }
      return result;
    }

    static bool updateResultForMagnitudeEdgeCase(
        const bool matchingEdgeIsMoving, const BezierCurveQ &matchingEdge,
        const RealNum &matchingEdgeMinParam,
        const RealNum &matchingEdgeMaxParam,
        const BezierCurveQ &targetMagnitudeEdge,
        const RealNum &targetMagnitudeEdgeParam, const ValueCalculation &vc,
        const ValueCalculation::ResultHolder &result,
        const BezierCurveQ *&outputMovingEdge,
        const BezierCurveQ *&outputStationaryEdge,
        RealNum &outputSmallestRangeDifferenceParam,
        RealNum &outputSmallestRangeDifference) {
      {
        const std::pair<RealNum, RealNum> smallestRangeDifference(
            getSmallestRangeDifference(
                matchingEdgeIsMoving, matchingEdge, matchingEdgeMinParam,
                matchingEdgeMaxParam, targetMagnitudeEdge,
                targetMagnitudeEdgeParam, vc));
        outputSmallestRangeDifferenceParam = smallestRangeDifference.first;
        outputSmallestRangeDifference = smallestRangeDifference.second;
      }
      if (!std::isnan(outputSmallestRangeDifference)) {
        if (vc.updateResultForMagnitudeEdgeCase(
                matchingEdgeIsMoving ? matchingEdge : targetMagnitudeEdge,
                matchingEdgeIsMoving ? outputSmallestRangeDifferenceParam
                                     : targetMagnitudeEdgeParam,
                matchingEdgeIsMoving ? targetMagnitudeEdge : matchingEdge,
                matchingEdgeIsMoving ? targetMagnitudeEdgeParam
                                     : outputSmallestRangeDifferenceParam,
                outputSmallestRangeDifference, result)) {
          outputMovingEdge =
              matchingEdgeIsMoving ? &matchingEdge : &targetMagnitudeEdge;
          outputStationaryEdge =
              matchingEdgeIsMoving ? &targetMagnitudeEdge : &matchingEdge;
        }
        return true;
      }
      return false;
    }

    static const CritInterval::CritIntervalEdgeEntry *getEntryForMagnitude(
        const RealNum &targetMagnitude, const bool decreasingTargetMagnitude,
        const std::vector<CritInterval::CritIntervalEdgeEntry> &intervalEdges) {
      /*
       * The fuzzy compares are necessary here due to possible tiny variances
       * between the calculated magnitude of one curve vs that of another that
       * can result in counterintuitive results.
       * */
      const CritInterval::CritIntervalEdgeEntry *bestResult = nullptr;
      RealNum bestResultOverlapSize = -1;
      for (const CritInterval::CritIntervalEdgeEntry &currentEdgeEntry :
           intervalEdges) {
        const auto currentEntryRange =
            std::minmax(currentEdgeEntry.startParamMagnitude,
                        currentEdgeEntry.endParamMagnitude);
        if (currentEntryRange.first <= targetMagnitude &&
            currentEntryRange.second >= targetMagnitude) {
          const RealNum currentOverlapSize =
              decreasingTargetMagnitude
                  ? (targetMagnitude - currentEntryRange.first)
                  : (currentEntryRange.second - targetMagnitude);
          if (currentOverlapSize > bestResultOverlapSize) {
            bestResult = &currentEdgeEntry;
            bestResultOverlapSize = currentOverlapSize;
          }
        } else if ((decreasingTargetMagnitude &&
                    sufficientlyClose(targetMagnitude,
                                      currentEntryRange.second)) ||
                   (!decreasingTargetMagnitude &&
                    sufficientlyClose(targetMagnitude,
                                      currentEntryRange.first))) {
          const RealNum currentOverlapSize =
              currentEntryRange.second - currentEntryRange.first;
          if (currentOverlapSize > bestResultOverlapSize) {
            bestResult = &currentEdgeEntry;
            bestResultOverlapSize = currentOverlapSize;
          }
        }
      }
      if (bestResult == nullptr) {
        throw std::string("Crit interval edge entry not found.");
      }
      return bestResult;
    }

    static const CritInterval::CritIntervalEdgeEntry *
    getAdjacentIntervalEdge(const CritInterval &interval,
                            const std::vector<CritInterval> &intervals,
                            const bool next) {
      const std::vector<CritInterval>::const_iterator intervalItr(
          std::find_if(intervals.cbegin(), intervals.cend(),
                       [&](const CritInterval &input) -> bool {
                         return &input == &interval;
                       }));
      return &(next
                   ? (intervalItr == (intervals.cend() - 1) ? intervals.cbegin()
                                                            : (intervalItr + 1))
                         ->edges.front()
                   : (intervalItr == intervals.cbegin() ? (intervals.cend() - 1)
                                                        : (intervalItr - 1))
                         ->edges.back());
    }

    static bool edgeMostlyInsideMagnitudeRange(
        const CritInterval::CritIntervalEdgeEntry &shape1Edge,
        const std::unordered_map<const BezierCurveQ *, CritsAndValues>
            &shape1CritsAndValues,
        const RealNum &targetMagnitude, const bool lowerMagnitudeIsInside) {
      const MagnitudeRange magnitudeRange(
          getMagnitudeRange(shape1CritsAndValues.at(shape1Edge.edge),
                            shape1Edge.startParam, shape1Edge.endParam));
      return lowerMagnitudeIsInside
                 ? ((magnitudeRange.higherMagnitude - targetMagnitude) <=
                    (targetMagnitude - magnitudeRange.lowerMagnitude))
                 : ((targetMagnitude - magnitudeRange.lowerMagnitude) <=
                    (magnitudeRange.higherMagnitude - targetMagnitude));
    }

    static bool validMagnitudeEdgeCase(
        const MagnitudeRange &highMagnitudeRange,
        const CritInterval &highMagnitudeInterval,
        const CritInterval::CritIntervalEdgeEntry &currentHighMagnitudeEdge,
        const std::unordered_map<const BezierCurveQ *, CritsAndValues>
            &highMagnitudeEdgeCritsAndValues,
        const std::vector<CritInterval> &highMagnitudeIntervals,
        const MagnitudeRange &lowMagnitudeRange,
        const CritInterval &lowMagnitudeInterval,
        const CritInterval::CritIntervalEdgeEntry &currentLowMagnitudeEdge,
        const std::unordered_map<const BezierCurveQ *, CritsAndValues>
            &lowMagnitudeEdgeCritsAndValues,
        const std::vector<CritInterval> &lowMagnitudeIntervals) {
      return
          // Ensure that this magnitude is not the magnitude endpoint of both
          // crit intervals.
          !sufficientlyClose(
              highMagnitudeRange.lowerMagnitude,
              std::min(highMagnitudeInterval.edges.front().startParamMagnitude,
                       highMagnitudeInterval.edges.back().endParamMagnitude)) ||
          !sufficientlyClose(
              lowMagnitudeRange.higherMagnitude,
              std::max(lowMagnitudeInterval.edges.front().startParamMagnitude,
                       lowMagnitudeInterval.edges.back().endParamMagnitude))
          // If it is a magnitude endpoint, add the combo if edges around the
          // magnitude endpoint can end up mostly inside the other shape.
          || LocalFunctions::edgeMostlyInsideMagnitudeRange(
                 currentHighMagnitudeEdge, highMagnitudeEdgeCritsAndValues,
                 lowMagnitudeRange.higherMagnitude, true) ||
          LocalFunctions::edgeMostlyInsideMagnitudeRange(
              currentLowMagnitudeEdge, lowMagnitudeEdgeCritsAndValues,
              highMagnitudeRange.lowerMagnitude, false) ||
          (&currentHighMagnitudeEdge == &(highMagnitudeInterval.edges.back()) &&
           LocalFunctions::edgeMostlyInsideMagnitudeRange(
               *LocalFunctions::getAdjacentIntervalEdge(
                   highMagnitudeInterval, highMagnitudeIntervals, true),
               highMagnitudeEdgeCritsAndValues,
               lowMagnitudeRange.higherMagnitude, true)) ||
          (&currentHighMagnitudeEdge ==
               &(highMagnitudeInterval.edges.front()) &&
           LocalFunctions::edgeMostlyInsideMagnitudeRange(
               *LocalFunctions::getAdjacentIntervalEdge(
                   highMagnitudeInterval, highMagnitudeIntervals, false),
               highMagnitudeEdgeCritsAndValues,
               lowMagnitudeRange.higherMagnitude, true)) ||
          (&currentLowMagnitudeEdge == &(lowMagnitudeInterval.edges.back()) &&
           LocalFunctions::edgeMostlyInsideMagnitudeRange(
               *LocalFunctions::getAdjacentIntervalEdge(
                   lowMagnitudeInterval, lowMagnitudeIntervals, true),
               lowMagnitudeEdgeCritsAndValues,
               highMagnitudeRange.lowerMagnitude, false)) ||
          (&currentLowMagnitudeEdge == &(lowMagnitudeInterval.edges.front()) &&
           LocalFunctions::edgeMostlyInsideMagnitudeRange(
               *LocalFunctions::getAdjacentIntervalEdge(
                   lowMagnitudeInterval, lowMagnitudeIntervals, false),
               lowMagnitudeEdgeCritsAndValues,
               highMagnitudeRange.lowerMagnitude, false));
    }
  };
  /********************************************************************************************************************/
  outputMovingEdge = nullptr;
  outputStationaryEdge = nullptr;
  std::unordered_map<const BezierCurveQ *, CritsAndValues>
      movingEdgeCritsAndValues;
  const std::vector<CritInterval> movingIntervals(
      getCritIntervals(moving.getEdges(), vc, false, movingEdgeCritsAndValues));
  std::unordered_map<const BezierCurveQ *, CritsAndValues>
      stationaryEdgeCritsAndValues;
  const std::vector<CritInterval> stationaryIntervals(getCritIntervals(
      stationary.getEdges(), vc, true, stationaryEdgeCritsAndValues));
  {
    const auto printIntervals =
        [](const std::vector<CritInterval> &intervals) -> std::string {
      std::string result;
      for (const CritInterval &current : intervals) {
        result += result.length() > 0 ? ", " : "\n";
        result += "{";
        result += std::string("relevant: ") +
                  (current.relevant ? "TRUE, " : "FALSE, ");
        result += std::string("increasing: ") +
                  (current.edges.front().startParamMagnitude <
                           current.edges.back().endParamMagnitude
                       ? "TRUE, "
                       : "FALSE, ");
        result += "{";
        for (const CritInterval::CritIntervalEdgeEntry &edgeEntry :
             current.edges) {
          result += &edgeEntry == &(current.edges.front()) ? "" : ", ";
          result += "\n{edge: " + edgeEntry.edge->toString();
          result +=
              +", startParam: " + toString(edgeEntry.startParam) +
              ", endParam: " + toString(edgeEntry.endParam) +
              ", startParamMagnitude: " +
              toString(edgeEntry.startParamMagnitude) +
              ", endParamMagnitude: " + toString(edgeEntry.endParamMagnitude) +
              ", rangeStart: " + toString(edgeEntry.range.first) +
              ", rangeEnd: " + toString(edgeEntry.range.second);
          result += "}";
        }
        result += "}";
        result += "}\n";
      }
      return result;
    };
    debug() << "moveAgainst - MOVING CRIT INTERVALS: "
            << printIntervals(movingIntervals) << std::endl;
    debug() << "moveAgainst - STATIONARY CRIT INTERVALS: "
            << printIntervals(stationaryIntervals) << std::endl;
  }
  std::vector<IntervalCombo> intervalCombos;
  for (const CritInterval &movingInterval : movingIntervals) {
    if (!movingInterval.relevant) {
      continue;
    }
    const auto movingIntervalRange =
        std::minmax(movingInterval.edges.front().startParamMagnitude,
                    movingInterval.edges.back().endParamMagnitude);
    for (const CritInterval &stationaryInterval : stationaryIntervals) {
      if (!stationaryInterval.relevant) {
        continue;
      }
      const auto stationaryIntervalRange =
          std::minmax(stationaryInterval.edges.front().startParamMagnitude,
                      stationaryInterval.edges.back().endParamMagnitude);
      debug() << "moveAgainst - current moving interval range: "
              << movingIntervalRange.first << movingIntervalRange.second
              << " current stationary interval range: "
              << stationaryIntervalRange.first << stationaryIntervalRange.second
              << std::endl;
      if (movingIntervalRange.second < stationaryIntervalRange.first ||
          movingIntervalRange.first > stationaryIntervalRange.second) {
        continue;
      }
      // Always use the higher magnitude interval endpoint - ensures a valid
      // result (avoids rotation fulcrum cases).
      if (movingIntervalRange.second > stationaryIntervalRange.second) {
        const CritInterval::CritIntervalEdgeEntry *const movingEdgeInterval =
            LocalFunctions::getEntryForMagnitude(stationaryIntervalRange.second,
                                                 true, movingInterval.edges);
        const CritInterval::CritIntervalEdgeEntry *const
            stationaryEdgeInterval = LocalFunctions::getEntryForMagnitude(
                stationaryIntervalRange.second, true, stationaryInterval.edges);
        const std::pair<RealNum, RealNum> smallestRangeDifference(
            LocalFunctions::getSmallestRangeDifference(
                true, *(movingEdgeInterval->edge),
                movingEdgeInterval->startParam, movingEdgeInterval->endParam,
                *(stationaryEdgeInterval->edge),
                stationaryEdgeInterval->endParamMagnitude >
                        stationaryEdgeInterval->startParamMagnitude
                    ? stationaryEdgeInterval->endParam
                    : stationaryEdgeInterval->startParam,
                vc));
        if (!std::isnan(smallestRangeDifference.first)) {
          intervalCombos.push_back({&movingInterval, &stationaryInterval,
                                    smallestRangeDifference.second});
        }
      } else {
        const CritInterval::CritIntervalEdgeEntry *const movingEdgeInterval =
            LocalFunctions::getEntryForMagnitude(movingIntervalRange.second,
                                                 true, movingInterval.edges);
        const CritInterval::CritIntervalEdgeEntry *const
            stationaryEdgeInterval = LocalFunctions::getEntryForMagnitude(
                movingIntervalRange.second, true, stationaryInterval.edges);
        const std::pair<RealNum, RealNum> smallestRangeDifference(
            LocalFunctions::getSmallestRangeDifference(
                false, *(stationaryEdgeInterval->edge),
                stationaryEdgeInterval->startParam,
                stationaryEdgeInterval->endParam, *(movingEdgeInterval->edge),
                movingEdgeInterval->endParamMagnitude >
                        movingEdgeInterval->startParamMagnitude
                    ? movingEdgeInterval->endParam
                    : movingEdgeInterval->startParam,
                vc));
        if (!std::isnan(smallestRangeDifference.first)) {
          intervalCombos.push_back({&movingInterval, &stationaryInterval,
                                    smallestRangeDifference.second});
        }
      }
    }
  }
  debug() << "moveAgainst - interval combo count: " << intervalCombos.size()
          << std::endl;
  std::sort(intervalCombos.begin(), intervalCombos.end());
  for (const IntervalCombo &currentCombo : intervalCombos) {
    std::vector<EdgeCombo> edgeCombos;
    for (const CritInterval::CritIntervalEdgeEntry &currentMovingEdge :
         currentCombo.movingInterval->edges) {
      const MagnitudeRange movingMagnitudeRange(
          LocalFunctions::getMagnitudeRange(
              movingEdgeCritsAndValues[currentMovingEdge.edge],
              currentMovingEdge.startParam, currentMovingEdge.endParam));
      for (const CritInterval::CritIntervalEdgeEntry &currentStationaryEdge :
           currentCombo.stationaryInterval->edges) {
        const MagnitudeRange stationaryMagnitudeRange(
            LocalFunctions::getMagnitudeRange(
                stationaryEdgeCritsAndValues[currentStationaryEdge.edge],
                currentStationaryEdge.startParam,
                currentStationaryEdge.endParam));
        const RealNum overlapSize =
            std::min(movingMagnitudeRange.higherMagnitude,
                     stationaryMagnitudeRange.higherMagnitude) -
            std::max(movingMagnitudeRange.lowerMagnitude,
                     stationaryMagnitudeRange.lowerMagnitude);
        debug() << "moveAgainst -"
                << " moving edge: " << *currentMovingEdge.edge
                << " stationary edge: " << *currentStationaryEdge.edge
                << " overlapSize: " << toString(overlapSize) << std::endl;
        if (overlapSize >= 0) {
          EdgeCombo currentEdgeCombo(
              {&currentMovingEdge, &currentStationaryEdge, movingMagnitudeRange,
               stationaryMagnitudeRange,
               fuzzyEquals(overlapSize,
                           movingMagnitudeRange.higherMagnitude -
                               movingMagnitudeRange.lowerMagnitude) ||
                       fuzzyEquals(overlapSize,
                                   stationaryMagnitudeRange.higherMagnitude -
                                       stationaryMagnitudeRange.lowerMagnitude)
                   ? std::numeric_limits<RealNum>::max()
                   : overlapSize,
               false, false});
          if (movingMagnitudeRange.lowerMagnitude <
              stationaryMagnitudeRange.higherMagnitude) {
            if (sufficientlyClose(
                    movingMagnitudeRange.lowerMagnitude,
                    stationaryMagnitudeRange
                        .higherMagnitude)) { // Match moving's lesser magnitude
                                             // to stationary's greater
                                             // magnitude.
              if (LocalFunctions::validMagnitudeEdgeCase(
                      movingMagnitudeRange, *currentCombo.movingInterval,
                      currentMovingEdge, movingEdgeCritsAndValues,
                      movingIntervals, stationaryMagnitudeRange,
                      *currentCombo.stationaryInterval, currentStationaryEdge,
                      stationaryEdgeCritsAndValues, stationaryIntervals)) {
                currentEdgeCombo.edgeCaseMovingLowerStationaryHigher = true;
                edgeCombos.push_back(currentEdgeCombo);
              }
            } else if (movingMagnitudeRange.higherMagnitude >
                       stationaryMagnitudeRange.lowerMagnitude) {
              if (sufficientlyClose(
                      movingMagnitudeRange.higherMagnitude,
                      stationaryMagnitudeRange
                          .lowerMagnitude)) { // Match moving's greater
                                              // magnitude to stationary's
                                              // lesser magnitude.
                if (LocalFunctions::validMagnitudeEdgeCase(
                        stationaryMagnitudeRange,
                        *currentCombo.stationaryInterval, currentStationaryEdge,
                        stationaryEdgeCritsAndValues, stationaryIntervals,
                        movingMagnitudeRange, *currentCombo.movingInterval,
                        currentMovingEdge, movingEdgeCritsAndValues,
                        movingIntervals)) {
                  currentEdgeCombo.edgeCaseMovingHigherStationaryLower = true;
                  edgeCombos.push_back(currentEdgeCombo);
                }
                //                    if
                //                    (!sufficientlyClose(movingMagnitudeRange.higherMagnitude,
                //                    std::max(currentCombo.movingInterval->edges.front()
                //                        .startParamMagnitude,
                //                        currentCombo.movingInterval->edges.back().endParamMagnitude))
                //                      ||
                //                      !sufficientlyClose(stationaryMagnitudeRange.lowerMagnitude,
                //                      std::min(currentCombo.stationaryInterval->edges
                //                        .front().startParamMagnitude,
                //                        currentCombo.stationaryInterval->edges.back().endParamMagnitude))
                //                      )
                //                    {
                //                      /*
                //                       * Ensure that this magnitude is not the
                //                       magnitude endpoint of both crit
                //                       intervals, or if it is, add the edge
                //                       * combo only if one shape's edge could
                //                       mostly overlap the other's.
                //                       * */
                //                      currentEdgeCombo.edgeCaseMovingHigherStationaryLower
                //                      = true;
                //                      edgeCombos.push_back(currentEdgeCombo);
                //                    }
              } else {
                edgeCombos.push_back(currentEdgeCombo);
              }
            }
          }
        }
      }
    }
    std::sort(edgeCombos.begin(), edgeCombos.end());
    for (const EdgeCombo &currentEdgeCombo : edgeCombos) {
      RealNum smallestRangeDifferenceParam, smallestRangeDifference;
      debug() << "moveAgainst - getting result for combo,"
              << " moving edge: " << *(currentEdgeCombo.movingEdge->edge)
              << " stationary edge: "
              << *(currentEdgeCombo.stationaryEdge->edge) << std::endl;
      if (currentEdgeCombo.edgeCaseMovingLowerStationaryHigher) {
        LocalFunctions::updateResultForMagnitudeEdgeCase(
            true, *(currentEdgeCombo.movingEdge->edge),
            currentEdgeCombo.movingEdge->startParam,
            currentEdgeCombo.movingEdge->endParam,
            *(currentEdgeCombo.stationaryEdge->edge),
            currentEdgeCombo.stationaryEdgeMagnitudeRange.higherMagnitudeParam,
            vc, resultHolder, outputMovingEdge, outputStationaryEdge,
            smallestRangeDifferenceParam, smallestRangeDifference);
        LocalFunctions::updateResultForMagnitudeEdgeCase(
            false, *(currentEdgeCombo.stationaryEdge->edge),
            currentEdgeCombo.stationaryEdge->startParam,
            currentEdgeCombo.stationaryEdge->endParam,
            *(currentEdgeCombo.movingEdge->edge),
            currentEdgeCombo.movingEdgeMagnitudeRange.lowerMagnitudeParam, vc,
            resultHolder, outputMovingEdge, outputStationaryEdge,
            smallestRangeDifferenceParam, smallestRangeDifference);
      } else if (currentEdgeCombo.edgeCaseMovingHigherStationaryLower) {
        LocalFunctions::updateResultForMagnitudeEdgeCase(
            true, *(currentEdgeCombo.movingEdge->edge),
            currentEdgeCombo.movingEdge->startParam,
            currentEdgeCombo.movingEdge->endParam,
            *(currentEdgeCombo.stationaryEdge->edge),
            currentEdgeCombo.stationaryEdgeMagnitudeRange.lowerMagnitudeParam,
            vc, resultHolder, outputMovingEdge, outputStationaryEdge,
            smallestRangeDifferenceParam, smallestRangeDifference);
        LocalFunctions::updateResultForMagnitudeEdgeCase(
            false, *(currentEdgeCombo.stationaryEdge->edge),
            currentEdgeCombo.stationaryEdge->startParam,
            currentEdgeCombo.stationaryEdge->endParam,
            *(currentEdgeCombo.movingEdge->edge),
            currentEdgeCombo.movingEdgeMagnitudeRange.higherMagnitudeParam, vc,
            resultHolder, outputMovingEdge, outputStationaryEdge,
            smallestRangeDifferenceParam, smallestRangeDifference);
      } else if (vc.updateResultForNormalMove(
                     *(currentEdgeCombo.movingEdge->edge),
                     currentEdgeCombo.movingEdge->startParam,
                     currentEdgeCombo.movingEdge->endParam,
                     currentEdgeCombo.movingEdge->range,
                     movingEdgeCritsAndValues[currentEdgeCombo.movingEdge
                                                  ->edge],
                     *(currentEdgeCombo.stationaryEdge->edge),
                     currentEdgeCombo.stationaryEdge->startParam,
                     currentEdgeCombo.stationaryEdge->endParam,
                     currentEdgeCombo.stationaryEdge->range,
                     stationaryEdgeCritsAndValues[currentEdgeCombo
                                                      .stationaryEdge->edge],
                     resultHolder)) {
        outputMovingEdge = currentEdgeCombo.movingEdge->edge;
        outputStationaryEdge = currentEdgeCombo.stationaryEdge->edge;
      }
    }
  }
}

const CircleArc getArcForPointRotation(const Point2D &fulcrum,
                                       const Point2D &rotatingPoint,
                                       const RealNum &rotationAngle,
                                       const bool clockwise) {
  struct LocalFunctions {
    static RealNum correctAngle(const RealNum &input) {
      if (input < 0) {
        return input + (360 * (std::floor(std::fabs(input) / 360.0) + 1));
      } else if (input > 360) {
        return input - (360 * std::floor(input / 360.0));
      } else {
        return input;
      }
    }
  };
  const RealNum endpointPVCW =
      Point2D::getAngleBetween(Point2D(fulcrum.getX(), fulcrum.getY() + 100.0),
                               rotatingPoint, fulcrum, true);
  const RealNum arcOtherEndPVCW =
      rotationAngle > 360 || sufficientlyClose(rotationAngle, 360)
          ? endpointPVCW
      : clockwise ? LocalFunctions::correctAngle(endpointPVCW + rotationAngle)
                  : LocalFunctions::correctAngle(endpointPVCW - rotationAngle);
  return CircleArc(fulcrum, fulcrum.distanceFrom(rotatingPoint),
                   clockwise ? endpointPVCW : arcOtherEndPVCW,
                   clockwise ? arcOtherEndPVCW : endpointPVCW);
}
} // namespace

Shape::Shape(const std::vector<Point2D> &verticies,
             const std::vector<Point2D> &controlPoints)
    : maxX(std::numeric_limits<RealNum>::quiet_NaN()),
      minX(std::numeric_limits<RealNum>::quiet_NaN()),
      maxY(std::numeric_limits<RealNum>::quiet_NaN()),
      minY(std::numeric_limits<RealNum>::quiet_NaN()) {
  if (verticies.size() < 2) {
    throw std::string("Attempted to create a shape with less than 2 edges.");
  }
  if (verticies.size() != controlPoints.size()) {
    throw std::string(
        "The number of verticies does not match the number of control points.");
  }
  std::vector<Point2D>::const_iterator i = verticies.begin();
  std::vector<Point2D>::const_iterator j = controlPoints.begin();
  const Point2D *lastVertex = &(*i);
  i++;
  edges.reserve(verticies.size());
  while (i != verticies.end()) {
    addEdge(*lastVertex, *i, *j);
    lastVertex = &(*i);
    i++;
    j++;
  }
  addEdge(*lastVertex, verticies[0], *j);
}

const RealNum &Shape::getMaxX() const { return maxX; }

const RealNum &Shape::getMinX() const { return minX; }

const RealNum &Shape::getMaxY() const { return maxY; }

const RealNum &Shape::getMinY() const { return minY; }

Shape Shape::rotate(const Point2D &fulcrum, const RealNum &angle) const {
  std::vector<Point2D> rotatedVerticies;
  rotatedVerticies.reserve(edges.size());
  std::vector<Point2D> rotatedControls;
  rotatedControls.reserve(edges.size());
  for (std::vector<BezierCurveQ>::const_iterator currentEdge = edges.cbegin();
       currentEdge != edges.cend(); currentEdge++) {
    rotatedControls.push_back(currentEdge->getControl().rotate(fulcrum, angle));
    if (currentEdge == edges.cbegin()) {
      rotatedVerticies.push_back(
          currentEdge->valueAt(0).rotate(fulcrum, angle));
    }
    if ((currentEdge + 1) != edges.cend()) {
      rotatedVerticies.push_back(
          currentEdge->valueAt(1).rotate(fulcrum, angle));
    }
  }
  return Shape(rotatedVerticies, rotatedControls);
}

Shape Shape::shift(const RealNum &distance, const RealNum &slope, bool right,
                   bool up) const {
  std::vector<Point2D> shiftedVerticies;
  shiftedVerticies.reserve(edges.size());
  std::vector<Point2D> shiftedControls;
  shiftedControls.reserve(edges.size());
  for (std::vector<BezierCurveQ>::const_iterator currentEdge = edges.cbegin();
       currentEdge != edges.cend(); currentEdge++) {
    shiftedControls.push_back(
        currentEdge->getControl().shift(distance, slope, right, up));
    if (currentEdge == edges.cbegin()) {
      shiftedVerticies.push_back(
          currentEdge->valueAt(0).shift(distance, slope, right, up));
    }
    if ((currentEdge + 1) != edges.cend()) {
      shiftedVerticies.push_back(
          currentEdge->valueAt(1).shift(distance, slope, right, up));
    }
  }
  return Shape(shiftedVerticies, shiftedControls);
}

std::vector<Shape::ShapeOverlap> Shape::getOverlap(const Shape &input)
    const { // Returns an object representing the shape of the overlap of this
  // shape and the input shape.
  /*
  ------------------------------------------------------------------------------
  Start step 1: Calculate the intersection of each of this shape's edges with
  those of the input shape; if any set of edges 'cross' then store the curves
  and crossing points, same for touches; if any set of edges are parallel, store
  the start and end pairs of edges of each consecutive sequence of parallel
  edges.
  */
  std::vector<EdgeTouch> touches;
  {
    std::vector<BezierCurveQ>::const_iterator myEdges = edges.begin();
    const BezierCurveQ *myFirstEdge = &(*myEdges);
    myEdges++;
    const BezierCurveQ *myNextEdge = myFirstEdge;
    const BezierCurveQ *myPreviousEdge = &(edges.back());
    while (myNextEdge != myFirstEdge || myEdges != edges.end()) {
      const BezierCurveQ *myCurrentEdge = myNextEdge;
      if (myEdges != edges.end()) {
        myNextEdge = &(*myEdges);
        myEdges++;
      } else {
        myNextEdge = myFirstEdge;
      }
      std::vector<BezierCurveQ>::const_iterator inputEdges =
          input.edges.begin();
      const BezierCurveQ *inputFirstEdge = &(*inputEdges);
      inputEdges++;
      const BezierCurveQ *inputNextEdge = inputFirstEdge;
      const BezierCurveQ *inputPreviousEdge = &(input.edges.back());
      while (inputNextEdge != inputFirstEdge ||
             inputEdges != input.edges.end()) {
        const BezierCurveQ *inputCurrentEdge = inputNextEdge;
        if (inputEdges != input.edges.end()) {
          inputNextEdge = &(*inputEdges);
          inputEdges++;
        } else {
          inputNextEdge = inputFirstEdge;
        }
        std::vector<std::pair<RealNum, RealNum>> myIntersectionParams(
            myCurrentEdge->pointsOfIntersection(*inputCurrentEdge));
        if (BezierCurveQ::isIntersectionInfinite(
                myIntersectionParams)) { // Special case: a parallel pair of
                                         // edges, one from each shape, start
                                         // and/or end at the same point.
          if (myCurrentEdge->valueAt(0) == inputCurrentEdge->valueAt(0) &&
              !BezierCurveQ::isIntersectionInfinite(
                  myPreviousEdge->pointsOfIntersection(*inputPreviousEdge))) {
            touches.push_back(EdgeTouch(EdgeTouch::PARALLEL_START,
                                        *myPreviousEdge, 1, *inputPreviousEdge,
                                        1));
          }
          if (myCurrentEdge->valueAt(0) == inputCurrentEdge->valueAt(1) &&
              !BezierCurveQ::isIntersectionInfinite(
                  myPreviousEdge->pointsOfIntersection(*inputNextEdge))) {
            touches.push_back(EdgeTouch(EdgeTouch::PARALLEL_START,
                                        *myPreviousEdge, 1, *inputNextEdge, 0));
          }
          if (myCurrentEdge->valueAt(1) == inputCurrentEdge->valueAt(0) &&
              !BezierCurveQ::isIntersectionInfinite(
                  myNextEdge->pointsOfIntersection(*inputPreviousEdge))) {
            touches.push_back(EdgeTouch(EdgeTouch::PARALLEL_START, *myNextEdge,
                                        0, *inputPreviousEdge, 1));
          }
          if (myCurrentEdge->valueAt(1) == inputCurrentEdge->valueAt(1) &&
              !BezierCurveQ::isIntersectionInfinite(
                  myNextEdge->pointsOfIntersection(*inputNextEdge))) {
            touches.push_back(EdgeTouch(EdgeTouch::PARALLEL_START, *myNextEdge,
                                        0, *inputNextEdge, 0));
          }
        } else if (myIntersectionParams.size() > 0) {
          for (std::vector<std::pair<RealNum, RealNum>>::iterator
                   myCurrentParam = myIntersectionParams.begin();
               myCurrentParam != myIntersectionParams.end(); myCurrentParam++) {
            Point2D intersectionPoint(
                myCurrentEdge->valueAt(myCurrentParam->first));
            const BezierCurveQ *myEdge1 = myCurrentEdge;
            RealNum myEdge1Param = myCurrentParam->first;
            const BezierCurveQ *myEdge2 = myCurrentEdge;
            RealNum myEdge2Param = myCurrentParam->first;
            const BezierCurveQ *inputEdge1 = inputCurrentEdge;
            RealNum inputEdge1Param = myCurrentParam->second;
            const BezierCurveQ *inputEdge2 = inputCurrentEdge;
            RealNum inputEdge2Param = myCurrentParam->second;
            bool intersectionTest =
                false; // This is set to true if this shape's and/or the input
                       // shape's edges intersect each other at the
                       // intersection of a shape's edges.
            if (nearEnd(*myCurrentEdge, intersectionPoint)) {
              std::vector<std::pair<RealNum, RealNum>> myNextIntersection(
                  myNextEdge->pointsOfIntersection(*inputCurrentEdge));
              if (!BezierCurveQ::isIntersectionInfinite(myNextIntersection) &&
                  myNextIntersection.size() > 0 &&
                  nearStart(
                      *myNextEdge,
                      myNextEdge->valueAt(
                          myNextIntersection[0]
                              .first))) { // The current input edge touches at
                                          // the intersection of this shape's
                                          // current and next edges.
                intersectionTest = true;
                myEdge1 = myCurrentEdge;
                myEdge1Param = 1;
                myEdge2 = myNextEdge;
                myEdge2Param = 0;
              } else if (BezierCurveQ::isIntersectionInfinite(
                             myNextIntersection)) {
                if (!nearStart(*inputCurrentEdge, intersectionPoint) &&
                    !nearEnd(*inputCurrentEdge, intersectionPoint)) {
                  touches.push_back(
                      EdgeTouch(EdgeTouch::PARALLEL_START, *myCurrentEdge, 1,
                                *inputCurrentEdge, myCurrentParam->second));
                }
                break;
              }
            } else if (nearStart(*myCurrentEdge, intersectionPoint)) {
              std::vector<std::pair<RealNum, RealNum>> myPreviousIntersection(
                  myPreviousEdge->pointsOfIntersection(*inputCurrentEdge));
              if (!BezierCurveQ::isIntersectionInfinite(
                      myPreviousIntersection) &&
                  myPreviousIntersection.size() > 0 &&
                  nearEnd(*myPreviousEdge,
                          myPreviousEdge->valueAt(
                              myPreviousIntersection.back()
                                  .first))) { // Handled in a previous
                                              // iteration of this algorithm.
                break;
              } else if (BezierCurveQ::isIntersectionInfinite(
                             myPreviousIntersection)) {
                if (!nearStart(*inputCurrentEdge, intersectionPoint) &&
                    !nearEnd(*inputCurrentEdge, intersectionPoint)) {
                  touches.push_back(
                      EdgeTouch(EdgeTouch::PARALLEL_START, *myCurrentEdge, 0,
                                *inputCurrentEdge, myCurrentParam->second));
                }
                break;
              }
            }
            if (nearEnd(*inputCurrentEdge, intersectionPoint)) {
              std::vector<std::pair<RealNum, RealNum>> inputNextIntersection(
                  inputNextEdge->pointsOfIntersection(*myCurrentEdge));
              if (!BezierCurveQ::isIntersectionInfinite(
                      inputNextIntersection) &&
                  inputNextIntersection.size() > 0 &&
                  nearStart(
                      *inputNextEdge,
                      inputNextEdge->valueAt(
                          inputNextIntersection[0]
                              .first))) { // This shape's, current edge touches
                                          // at the intersection of the input
                                          // shape's current and next edges.
                intersectionTest = true;
                inputEdge1 = inputCurrentEdge;
                inputEdge1Param = 1;
                inputEdge2 = inputNextEdge;
                inputEdge2Param = 0;
              } else if (BezierCurveQ::isIntersectionInfinite(
                             inputNextIntersection)) {
                if (!nearStart(*myCurrentEdge, intersectionPoint) &&
                    !nearEnd(*myCurrentEdge, intersectionPoint)) {
                  touches.push_back(
                      EdgeTouch(EdgeTouch::PARALLEL_START, *myCurrentEdge,
                                myCurrentParam->first, *inputCurrentEdge, 1));
                }
                break;
              }
            } else if (nearStart(*inputCurrentEdge, intersectionPoint)) {
              std::vector<std::pair<RealNum, RealNum>>
                  inputPreviousIntersection(
                      inputPreviousEdge->pointsOfIntersection(*myCurrentEdge));
              if (!BezierCurveQ::isIntersectionInfinite(
                      inputPreviousIntersection) &&
                  inputPreviousIntersection.size() > 0 &&
                  nearEnd(*inputPreviousEdge,
                          inputPreviousEdge->valueAt(
                              inputPreviousIntersection.back()
                                  .first))) { // Handled in a previous
                                              // iteration of this algorithm.
                break;
              } else if (BezierCurveQ::isIntersectionInfinite(
                             inputPreviousIntersection)) {
                if (!nearStart(*myCurrentEdge, intersectionPoint) &&
                    !nearEnd(*myCurrentEdge, intersectionPoint)) {
                  touches.push_back(
                      EdgeTouch(EdgeTouch::PARALLEL_START, *myCurrentEdge,
                                myCurrentParam->first, *inputCurrentEdge, 0));
                }
                break;
              }
            }
            if (intersectionTest) {
              if (BezierCurveQ::isIntersectionInfinite(
                      myEdge2->pointsOfIntersection(
                          *inputEdge2))) { // Parallel start handled as part of
                                           // a special case.
                break;
              }
              EdgeTouch::TouchType tt;
              if (crossesAtIntersection(*myEdge1, myEdge1Param, *myEdge2,
                                        myEdge2Param, *inputEdge1,
                                        inputEdge1Param, *inputEdge2,
                                        inputEdge2Param)) {
                tt = EdgeTouch::CROSS;
              } else {
                tt = EdgeTouch::TOUCH;
              }
              touches.push_back(EdgeTouch(tt, *myEdge1, myEdge1Param,
                                          *inputEdge1, inputEdge1Param));
              touches.back().setSecondSet(*myEdge2, myEdge2Param, *inputEdge2,
                                          inputEdge2Param);
            } else {
              if (sufficientlyClose(
                      myCurrentEdge->rateOfChangeAtParam(myCurrentParam->first),
                      inputCurrentEdge->rateOfChangeAtParam(
                          myCurrentParam->second))) {
                touches.push_back(EdgeTouch(
                    EdgeTouch::CROSS, *myCurrentEdge, myCurrentParam->first,
                    *inputCurrentEdge, myCurrentParam->second));
              } else {
                touches.push_back(EdgeTouch(
                    EdgeTouch::TOUCH, *myCurrentEdge, myCurrentParam->first,
                    *inputCurrentEdge, myCurrentParam->second));
              }
            }
          }
        }
        inputPreviousEdge = inputCurrentEdge;
      }
      myPreviousEdge = myCurrentEdge;
    }
  }
  /*
  End step 1
  ------------------------------------------------------------------------------
  Start step 2: Using the set of 'touches' computed in the previous step,
  determine which sets actually lie within the input shape, and vice versa;
  match consecutive sequences of edge sets into ShapeOverlap objects, which are
  themselves new shapes representing the outline of the overlap(s) between this
  and the input shapes.
  */
  {
    std::vector<ShapeOverlap> result;
    std::vector<EdgeSection> myInsideEdges = getInsideEdges(
        *this, input, touches,
        true); // My edges that lie at least partially inside the input shape.
    std::vector<EdgeSection> inputInsideEdges = getInsideEdges(
        input, *this, touches, false); // The input shape's edges that lie at
                                       // least partially inside this shape.
    while (myInsideEdges.size() > 0 || inputInsideEdges.size() > 0) {
      // result.push_back(ShapeOverlap());
      std::vector<EdgeSection> currentOverlap;
      std::vector<EdgeSection> *currentSource = &inputInsideEdges;
      currentOverlap.push_back(myInsideEdges.back());
      myInsideEdges.erase(--myInsideEdges.end());
      bool matchEnd = true;
      Point2D startPoint(currentOverlap.back().edges.front()->valueAt(
          currentOverlap.back().startParam));
      for (;;) {
        Point2D testPoint(matchEnd
                              ? currentOverlap.back().edges.back()->valueAt(
                                    currentOverlap.back().endParam)
                              : currentOverlap.back().edges.front()->valueAt(
                                    currentOverlap.back().startParam));
        for (std::vector<EdgeSection>::iterator i = currentSource->begin();
             i != currentSource->end(); i++) {
          if (i->edges.front()->valueAt(i->startParam) == testPoint) {
            currentOverlap.push_back(*i);
            currentSource->erase(i);
            matchEnd = true;
            break;
          } else if (i->edges.back()->valueAt(i->endParam) == testPoint) {
            currentOverlap.push_back(*i);
            currentSource->erase(i);
            matchEnd = false;
            break;
          }
        }
        currentSource = currentSource == &myInsideEdges ? &inputInsideEdges
                                                        : &myInsideEdges;
        if (matchEnd && currentOverlap.back().edges.back()->valueAt(
                            currentOverlap.back().endParam) == startPoint) {
          break;
        } else if (!matchEnd &&
                   currentOverlap.back().edges.front()->valueAt(
                       currentOverlap.back().startParam) == startPoint) {
          break;
        }
      }
      result.push_back(ShapeOverlap(currentOverlap));
    }
    return result;
  }
  /*
  End step 2
  ------------------------------------------------------------------------------
  */
}

Shape::ShapeOverlap::ShapeOverlap(const std::vector<EdgeSection> &sections)
    : sections(sections) {}

const std::vector<Shape::EdgeSection> &
Shape::ShapeOverlap::getSections() const {
  return sections;
}

int Shape::pointInPolygon(const Point2D &testPoint)
    const { // Returns 1 if the passed point lies inside this shape, -1 if the
  // passed point is outside, 0 if the passed point lies on the edge
  // of this shape.
  const int TEST_LINE_DISTANCE = 10000;
  const int JUMP_DISTANCE = 100;
  for (int i = 0; i <= TEST_LINE_DISTANCE / JUMP_DISTANCE;
       i++) { // Keep getting long lines from the test point until one does not
              // cross near the end of an edge and is not parallel to an edge.
    BezierCurveQ currentTestLine = BezierCurveQ::straightLine(
        testPoint,
        Point2D(testPoint.getX() + (i * JUMP_DISTANCE),
                testPoint.getY() + TEST_LINE_DISTANCE - (i * JUMP_DISTANCE)));
    bool testValid = true;
    int intersectionCount = 0;
    for (std::vector<BezierCurveQ>::const_iterator currentEdge = edges.begin();
         currentEdge != edges.end(); currentEdge++) {
      std::vector<std::pair<RealNum, RealNum>> result(
          currentEdge->pointsOfIntersection(currentTestLine));
      if (BezierCurveQ::isIntersectionInfinite(result)) {
        testValid = false;
        break;
      }
      for (std::vector<std::pair<RealNum, RealNum>>::iterator currentParam =
               result.begin();
           currentParam != result.end(); currentParam++) {
        Point2D currentPoint(currentEdge->valueAt(currentParam->first));
        if (currentPoint == testPoint) {
          return 0;
        }
        if (nearStart(*currentEdge, currentPoint) ||
            nearEnd(*currentEdge, currentPoint) ||
            sufficientlyClose(
                currentTestLine.rateOfChangeAtParam(0),
                currentEdge->rateOfChangeAtParam(currentParam->first))) {
          testValid = false;
          break;
        }
      }
      if (!testValid) {
        break;
      }
      intersectionCount += result.size();
    }
    if (testValid) { // The passed point is inside if there is an odd number of
                     // intersections with this shape's edges.
      return (2 * (intersectionCount / 2) != intersectionCount) ? 1 : -1;
    }
  }
  throw std::string("Could not find a valid test line for a PiP test.");
}

const std::vector<BezierCurveQ> &Shape::getEdges() const { return edges; }

void Shape::addEdge(const Point2D &start, const Point2D &end,
                    const Point2D &control) {
  edges.push_back(BezierCurveQ(start, end, control));
  RealNum currentMaxX = edges.back().getMaxXExtent();
  RealNum currentMinX = edges.back().getMinXExtent();
  RealNum currentMaxY = edges.back().getMaxYExtent();
  RealNum currentMinY = edges.back().getMinYExtent();
  if (std::isnan(maxX) || currentMaxX > maxX) {
    maxX = currentMaxX;
  }
  if (std::isnan(minX) || currentMinX < minX) {
    minX = currentMinX;
  }
  if (std::isnan(maxY) || currentMaxY > maxY) {
    maxY = currentMaxY;
  }
  if (std::isnan(minY) || currentMinY < minY) {
    minY = currentMinY;
  }
}

Shape::EdgeTouch::EdgeTouch(const TouchType &tt, const BezierCurveQ &myEdge,
                            const RealNum &myEdgeParam,
                            const BezierCurveQ &inputEdge,
                            const RealNum &inputEdgeParam)
    : type(tt), myEdge(&myEdge), myEdgeParam(myEdgeParam),
      inputEdge(&inputEdge), inputEdgeParam(inputEdgeParam),
      mySecondEdge(nullptr), mySecondEdgeParam(-1), inputSecondEdge(nullptr),
      inputSecondEdgeParam(-1) {}

const BezierCurveQ *Shape::EdgeTouch::getEdge(bool getMine,
                                              bool getFirst) const {
  if (getMine) {
    if (getFirst) {
      return myEdge;
    } else {
      return mySecondEdge;
    }
  } else {
    if (getFirst) {
      return inputEdge;
    } else {
      return inputSecondEdge;
    }
  }
}

const RealNum &Shape::EdgeTouch::getEdgeParam(bool getMine,
                                              bool getFirst) const {
  if (getMine) {
    if (getFirst) {
      return myEdgeParam;
    } else {
      return mySecondEdgeParam;
    }
  } else {
    if (getFirst) {
      return inputEdgeParam;
    } else {
      return inputSecondEdgeParam;
    }
  }
}

const Shape::EdgeTouch::TouchType &Shape::EdgeTouch::getType() const {
  return type;
}

void Shape::EdgeTouch::setSecondSet(const BezierCurveQ &mySecondEdge,
                                    const RealNum &mySecondEdgeParam,
                                    const BezierCurveQ &inputSecondEdge,
                                    const RealNum &inputSecondEdgeParam) {
  this->mySecondEdge = &mySecondEdge;
  this->mySecondEdgeParam = mySecondEdgeParam;
  this->inputSecondEdge = &inputSecondEdge;
  this->inputSecondEdgeParam = inputSecondEdgeParam;
}

int Shape::addEdgeSet(const std::vector<BezierCurveQ> &shapeEdges,
                      int currentEdgeIdx, const EdgeTouch &startTouch,
                      const EdgeTouch &endTouch,
                      std::vector<EdgeSection> &result, bool getMine) {
  int idx(currentEdgeIdx);
  result.push_back(EdgeSection());
  for (;;) {
    result.back().edges.push_back(
        &(shapeEdges[std::vector<BezierCurveQ>::size_type(idx)]));
    if (&(shapeEdges[std::vector<BezierCurveQ>::size_type(idx)]) ==
        endTouch.getEdge(getMine, true)) {
      break;
    }
    idx++;
  }
  result.back().startParam = startTouch.getEdge(getMine, false) != nullptr
                                 ? startTouch.getEdgeParam(getMine, false)
                                 : startTouch.getEdgeParam(getMine, true);
  result.back().endParam = endTouch.getEdgeParam(getMine, true);
  return idx;
}

int Shape::PiPTestBetweenTouches(const EdgeTouch &touch1,
                                 const EdgeTouch &touch2,
                                 const Shape &inputShape, bool getMine) {
  Point2D testPoint(0, 0);
  if (touch2.getEdge(getMine, true) == touch1.getEdge(getMine, true)) {
    testPoint = touch2.getEdge(getMine, true)
                    ->valueAt((touch2.getEdgeParam(getMine, true) +
                               touch1.getEdgeParam(getMine, true)) /
                              2.0);
  } else {
    if (touch1.getEdge(getMine, false) != nullptr) {
      if (touch1.getEdge(getMine, false) == touch2.getEdge(getMine, true)) {
        testPoint = touch1.getEdge(getMine, false)
                        ->valueAt((touch1.getEdgeParam(getMine, false) +
                                   touch2.getEdgeParam(getMine, true)) /
                                  2.0);
      } else {
        testPoint = touch1.getEdge(getMine, false)->valueAt(1);
      }
    } else {
      testPoint = touch1.getEdge(getMine, true)->valueAt(1);
    }
  }
  int result = inputShape.pointInPolygon(testPoint);
  if (result == 0 && (touch1.getType() != EdgeTouch::PARALLEL_START ||
                      touch2.getType() != EdgeTouch::PARALLEL_START)) {
    throw std::string("Bad test point!");
  }
  return result;
}

/*
The edges of shape1 and shape2 intersect at the intersection of two of shape1's
edges and/or at the intersection of two of shape2's edges. Determine if this
intersection is actually a cross (not just a touch). -shape1Edge1 and
shape1Edge2 could be the same curve if the intersection is between two of
shape2's edges -shape2Edge1 and shape2Edge2 could be the same curve if the
intersection is between two of shape1's edges -NONE of shape1's input edges are
parallel to shape2's input edges
*/
bool Shape::crossesAtIntersection(
    const BezierCurveQ &shape1Edge1, const RealNum &shape1Edge1Param,
    const BezierCurveQ &shape1Edge2, const RealNum &shape1Edge2Param,
    const BezierCurveQ &shape2Edge1, const RealNum &shape2Edge1Param,
    const BezierCurveQ &shape2Edge2, const RealNum &shape2Edge2Param) {
  /*
  ------------------------------------------------------------------------------
  Start step 1: extrapolate each curve's rate of change at the intersection
  point into a long straight line.  This will help to calculate appropriate test
  points.
  */
  Point2D intPoint(shape1Edge1.valueAt(shape1Edge1Param));
  BezierCurveQ intLineS1E1(BezierCurveQ::longStraightLine(
      shape1Edge1.rateOfChangeAtParam(shape1Edge1Param), intPoint));
  BezierCurveQ intLineS1E2(BezierCurveQ::longStraightLine(
      shape1Edge2.rateOfChangeAtParam(shape1Edge2Param), intPoint));
  BezierCurveQ intLineS2E1(BezierCurveQ::longStraightLine(
      shape2Edge1.rateOfChangeAtParam(shape2Edge1Param), intPoint));
  BezierCurveQ intLineS2E2(BezierCurveQ::longStraightLine(
      shape2Edge2.rateOfChangeAtParam(shape2Edge2Param), intPoint));
  /*
  End step 1
  ------------------------------------------------------------------------------
  Start step 2: using the lines for each curve's intersection, get a 'test
  point' for each curve.  A test point is a point on each curve that does not
  lie on either of the other shape's intersection lines and whose curve does not
  intersect either of the other shape's intersection lines between it and the
  intersection point. -a test point's position relative to the intersection
  point represents the direction that the curve approaches the intersection
  relative to that of the other curves
  -it is cheaper to simply take another point along each curve that is extremely
  close to the intersection point, but this method could be subject to variable
  degrees of error based on the total length of the curves and their behaviour
  at the intersection point
  */
  RealNum testParam;
  testParam =
      std::max(getTestParam(shape1Edge1, shape1Edge1Param, true, shape2Edge1),
               getTestParam(shape1Edge1, shape1Edge1Param, true, shape2Edge2));
  if (!BezierCurveQ::isIntersectionInfinite(
          shape1Edge1.pointsOfIntersection(intLineS2E1))) {
    testParam = std::max(testParam, getTestParam(shape1Edge1, shape1Edge1Param,
                                                 true, intLineS2E1));
  }
  if (!BezierCurveQ::isIntersectionInfinite(
          shape1Edge1.pointsOfIntersection(intLineS2E2))) {
    testParam = std::max(testParam, getTestParam(shape1Edge1, shape1Edge1Param,
                                                 true, intLineS2E2));
  }
  Point2D testPointS1E1 = shape1Edge1.valueAt(testParam);
  testParam =
      std::max(getTestParam(shape1Edge2, shape1Edge2Param, false, shape2Edge1),
               getTestParam(shape1Edge2, shape1Edge2Param, false, shape2Edge2));
  if (!BezierCurveQ::isIntersectionInfinite(
          shape1Edge2.pointsOfIntersection(intLineS2E1))) {
    testParam = std::max(testParam, getTestParam(shape1Edge2, shape1Edge2Param,
                                                 false, intLineS2E1));
  }
  if (!BezierCurveQ::isIntersectionInfinite(
          shape1Edge2.pointsOfIntersection(intLineS2E2))) {
    testParam = std::max(testParam, getTestParam(shape1Edge2, shape1Edge2Param,
                                                 false, intLineS2E2));
  }
  Point2D testPointS1E2 = shape1Edge2.valueAt(testParam);
  testParam =
      std::max(getTestParam(shape2Edge1, shape2Edge1Param, true, shape1Edge1),
               getTestParam(shape2Edge1, shape2Edge1Param, true, shape1Edge2));
  if (!BezierCurveQ::isIntersectionInfinite(
          shape2Edge1.pointsOfIntersection(intLineS1E1))) {
    testParam = std::max(testParam, getTestParam(shape2Edge1, shape2Edge1Param,
                                                 true, intLineS1E1));
  }
  if (!BezierCurveQ::isIntersectionInfinite(
          shape2Edge1.pointsOfIntersection(intLineS1E2))) {
    testParam = std::max(testParam, getTestParam(shape2Edge1, shape2Edge1Param,
                                                 true, intLineS1E2));
  }
  Point2D testPointS2E1 = shape2Edge1.valueAt(testParam);
  testParam =
      std::max(getTestParam(shape2Edge2, shape2Edge2Param, false, shape1Edge1),
               getTestParam(shape2Edge2, shape2Edge2Param, false, shape1Edge2));
  if (!BezierCurveQ::isIntersectionInfinite(
          shape2Edge2.pointsOfIntersection(intLineS1E1))) {
    testParam = std::max(testParam, getTestParam(shape2Edge2, shape2Edge2Param,
                                                 false, intLineS1E1));
  }
  if (!BezierCurveQ::isIntersectionInfinite(
          shape2Edge2.pointsOfIntersection(intLineS1E2))) {
    testParam = std::max(testParam, getTestParam(shape2Edge2, shape2Edge2Param,
                                                 false, intLineS1E2));
  }
  Point2D testPointS2E2(shape2Edge2.valueAt(testParam));
  /*
  End step 2
  ------------------------------------------------------------------------------
  Start step 3: the test points and the intersection points can be considered 4
  line segments that originate at the intersection point and pass through one of
  the 4 intersection points respectively.  There is a cross at this intersection
  if one (and only one) of shape2's line segments lies inside the small angle
  created by shape1's line segments. -this step covers the simple cases: when
  the two lines from shape1 form a straight line
  */
  RealNum slopeS1E1 = (intPoint.getY() - testPointS1E1.getY()) /
                      (intPoint.getX() - testPointS1E1.getX());
  RealNum slopeS1E2 = (intPoint.getY() - testPointS1E2.getY()) /
                      (intPoint.getX() - testPointS1E2.getX());
  if (std::isinf(slopeS1E1) &&
      std::isinf(
          slopeS1E2)) { // The two test points from shape1 and the intersection
                        // point all lie on a vertical straight line.
    return (testPointS2E1.getX() < intPoint.getX()) !=
           (testPointS2E2.getX() < intPoint.getX());
  }
  if (sufficientlySmall(slopeS1E1) &&
      sufficientlySmall(
          slopeS1E2)) { // The two test points from shape1 and the intersection
                        // point all lie on a horizontal straight line.
    return (testPointS2E1.getY() < intPoint.getY()) !=
           (testPointS2E2.getY() < intPoint.getY());
  }
  if (!std::isinf(slopeS1E1) && !std::isinf(slopeS1E2) &&
      sufficientlyClose(
          slopeS1E1,
          slopeS1E2)) { // The two test points from shape1 and the intersection
                        // point all lie on a straight line.
    if (std::fabs(slopeS1E1) > 1) {
      RealNum testX = intPoint.getX() +
                      ((testPointS2E1.getY() - intPoint.getY()) / slopeS1E1);
      bool leftS2E1 = testPointS2E1.getX() < testX;
      testX = intPoint.getX() +
              ((testPointS2E2.getY() - intPoint.getY()) / slopeS1E1);
      bool leftS2E2 = testPointS2E2.getX() < testX;
      return leftS2E1 != leftS2E2;
    }
    RealNum testY = intPoint.getY() +
                    ((testPointS2E1.getX() - intPoint.getX()) * slopeS1E1);
    bool aboveS2E1 = testPointS2E1.getY() > testY;
    testY = intPoint.getY() +
            ((testPointS2E2.getX() - intPoint.getX()) * slopeS1E1);
    bool aboveS2E2 = testPointS2E2.getY() > testY;
    return aboveS2E1 != aboveS2E2;
  }
  /*
  End step 3
  ------------------------------------------------------------------------------
  Start step 4: the two lines formed by the two test points from shape 1 and the
  intersection point have an angle between them and therefore a line between
  shape1's test points will not cross the intersection point.  This is useful
  because their intersection with lines formed by the two test points from
  shape2 will dictate whether or not this is a cross.
  */
  BezierCurveQ testLine(
      BezierCurveQ::straightLine(testPointS1E1, testPointS1E2));
  BezierCurveQ testLineS2E1(
      BezierCurveQ::longStraightLine(intPoint, testPointS2E1));
  BezierCurveQ testLineS2E2(
      BezierCurveQ::longStraightLine(intPoint, testPointS2E2));
  std::vector<std::pair<RealNum, RealNum>> intS2E1(
      testLine.pointsOfIntersection(testLineS2E1));
  std::vector<std::pair<RealNum, RealNum>> intS2E2(
      testLine.pointsOfIntersection(testLineS2E2));
  return (intS2E1.size() > 0) != (intS2E2.size() > 0);
  /*
  End step 4
  ------------------------------------------------------------------------------
  */
}

RealNum Shape::getTestParam(const BezierCurveQ &testCurve,
                            const RealNum &testCurveParam, bool goingTowards,
                            const BezierCurveQ &input) {
  std::vector<std::pair<RealNum, RealNum>> intersections(
      testCurve.pointsOfIntersection(input));
  if (intersections.size() > 0) {
    Point2D testCurvePoint(testCurve.valueAt(testCurveParam));
    std::vector<std::pair<RealNum, RealNum>>::iterator currentParam =
        intersections.begin();
    if (goingTowards) {
      while (currentParam != intersections.end() &&
             currentParam->first < testCurveParam &&
             testCurve.valueAt(currentParam->first) != testCurvePoint) {
        currentParam++;
      }
      intersections.erase(currentParam, intersections.end());
    } else {
      while (currentParam != intersections.end() &&
             (currentParam->first < testCurveParam ||
              testCurve.valueAt(currentParam->first) == testCurvePoint)) {
        currentParam++;
      }
      if (currentParam != intersections.begin()) {
        intersections.erase(intersections.begin(), currentParam);
      }
    }
  }
  RealNum testParam;
  if (intersections.size() == 0) {
    testParam = goingTowards ? 0 : 1;
  } else {
    testParam =
        (testCurveParam +
         intersections[goingTowards ? intersections.size() - 1 : 0].first) /
        2.0;
  }
  return testParam;
}

bool Shape::nearStart(const BezierCurveQ &inputCurve,
                      const Point2D &inputPoint) {
  return inputCurve.valueAt(0) == inputPoint;
}

bool Shape::nearEnd(const BezierCurveQ &inputCurve, const Point2D &inputPoint) {
  return inputCurve.valueAt(1) == inputPoint;
}

bool Shape::TouchSort::operator()(const EdgeTouch &touch1,
                                  const EdgeTouch &touch2) const {
  for (std::vector<BezierCurveQ>::const_iterator currentEdge =
           sourceEdgeList->begin();
       currentEdge != sourceEdgeList->end(); currentEdge++) {
    if (&(*currentEdge) == touch1.getEdge(getMine, true) &&
        &(*currentEdge) != touch2.getEdge(getMine, true)) {
      return true;
    }
    if (&(*currentEdge) == touch2.getEdge(getMine, true) &&
        &(*currentEdge) != touch1.getEdge(getMine, true)) {
      return false;
    }
  }
  return touch1.getEdgeParam(getMine, true) <
         touch2.getEdgeParam(getMine, true);
}

/*
Returns the list of edge sections from testShape that lie inside
destinationShape, given the list of edge touches. When getMine is true,
testShape is this shape.
*/
std::vector<Shape::EdgeSection>
Shape::getInsideEdges(const Shape &testShape, const Shape &destinationShape,
                      std::vector<EdgeTouch> &touches, bool getMine) {
  {
    TouchSort sorter;
    sorter.sourceEdgeList = &(testShape.edges);
    sorter.getMine = getMine;
    std::sort(touches.begin(), touches.end(), sorter);
  }
  std::vector<EdgeSection> result;
  std::vector<EdgeTouch>::iterator touchIter = touches.begin();
  EdgeTouch *nextTouch = &(*touchIter);
  touchIter++;
  int lastStatus = PiPTestBetweenTouches(touches.back(), *nextTouch,
                                         destinationShape, getMine);
  int edgeIdx = 0;
  while (nextTouch != &(touches.front()) || touchIter != touches.end()) {
    EdgeTouch *currentTouch = nextTouch;
    if (touchIter != touches.end()) {
      nextTouch = &(*touchIter);
      touchIter++;
    } else {
      nextTouch = &(touches.front());
    }
    while (&(testShape.edges[std::vector<BezierCurveQ>::size_type(edgeIdx)]) !=
           currentTouch->getEdge(getMine, true)) {
      edgeIdx++;
    }
    if (currentTouch->getEdge(getMine, false) != nullptr) {
      while (
          &(testShape.edges[std::vector<BezierCurveQ>::size_type(edgeIdx)]) !=
          currentTouch->getEdge(getMine, false)) {
        edgeIdx++;
      }
    }
    if (currentTouch->getType() == EdgeTouch::TOUCH) {
      if (lastStatus == 1) {
        // Overlap here.
        edgeIdx = addEdgeSet(testShape.edges, edgeIdx, *currentTouch,
                             *nextTouch, result, getMine);
      }
    } else if (currentTouch->getType() == EdgeTouch::CROSS) {
      if (lastStatus == -1) {
        // Overlap here.
        edgeIdx = addEdgeSet(testShape.edges, edgeIdx, *currentTouch,
                             *nextTouch, result, getMine);
        lastStatus = 1;
      } else {
        lastStatus = -1;
      }
    } else if (currentTouch->getType() == EdgeTouch::PARALLEL_START) {
      if (lastStatus == 0) {
        int PiPResult = PiPTestBetweenTouches(*currentTouch, *nextTouch,
                                              destinationShape, getMine);
        if (PiPResult == 1) {
          // Overlap here.
          edgeIdx = addEdgeSet(testShape.edges, edgeIdx, *currentTouch,
                               *nextTouch, result, getMine);
        }
        lastStatus = PiPResult;
      } else {
        lastStatus = 0;
      }
    } else {
      throw std::string("Unhandled case!");
    }
  }
  return result;
}

void Shape::shiftAgainst(const Shape &input, const RealNum &slope, bool right,
                         bool up, Shape::ShiftAgainstResult &output) const {
  debug() << "Shape::shiftAgainst -"
          << " this: " << *this << " input: " << input
          << " slope: " << ::bezier_geometry::toString(slope)
          << " right: " << right << " up: " << up << std::endl;
  moveAgainst(ShiftValueCalculation(slope, right, up), *this, input,
              ShiftValueCalculation::ShiftResultHolder(output.movingEdgeResult),
              output.movingEdge, output.stationaryEdge);
}

void Shape::rotateAgainst(const Shape &input, const Point2D &fulcrum,
                          bool clockwise, RotateAgainstResult &output) const {
  debug() << "Shape::rotateAgainst -"
          << " this: " << *this << " input: " << input
          << " fulcrum: " << fulcrum << " clockwise: " << clockwise
          << std::endl;
  moveAgainst(
      RotateValueCalculation(fulcrum, clockwise), *this, input,
      RotateValueCalculation::RotateResultHolder(output.movingEdgeResult),
      output.movingEdge, output.stationaryEdge);
  debug() << "Shape::rotateAgainst - returning result: "
          << ::bezier_geometry::toString(output.movingEdgeResult.angle)
          << std::endl;
}

RealNum Shape::shiftAgainstAfterShifting(
    const Shape &inputBeforeMove, const RealNum &inputShiftDistance,
    const RealNum &inputShiftSlope, bool inputShiftRight, bool inputShiftUp,
    const RealNum &myShiftSlope, bool myShiftRight, bool myShiftUp) const {
  debug() << "^ Shape::shiftAgainstAfterShifting -"
          << " this: " << *this << " inputBeforeMove: " << inputBeforeMove
          << " inputShiftDistance: "
          << ::bezier_geometry::toString(inputShiftDistance)
          << " inputShiftSlope: "
          << ::bezier_geometry::toString(inputShiftSlope)
          << " inputShiftRight: " << inputShiftRight
          << " inputShiftUp: " << inputShiftUp
          << " myShiftSlope: " << ::bezier_geometry::toString(myShiftSlope)
          << " myShiftRight: " << myShiftRight << " myShiftUp: " << myShiftUp
          << std::endl;
  RealNum result = -1;
  /*
  1 - my shift direction decreases the perpendicular magnitude relative to the
  input's shift direction - therefore only test against the shifted shape and
  the streak endpoints that have a higher perpendicular magnitude relative to
  the input's shift direction. 0 - my shift direction either matches the input's
  shift direction or is in the exact opposite direction - only test against the
  shifted shape. -1 - my shift direction increases the perpendicular magnitude
  relative to the inputs' shift direction - therefore only test against the
  shifted shape and the streak endpoints that have a lower perpendicular
  magnitude relative to the input's shift direction.
  */
  const int testScenario = [&inputShiftSlope, &myShiftSlope, myShiftRight,
                            myShiftUp]() -> int {
    const Point2D origin(0, 0);
    const Point2D myDirectionPoint(
        origin.shift(100, myShiftSlope, myShiftRight, myShiftUp));
    const RealNum originPerpMag =
        Point2D::getPerpendicularMagnitude(origin, inputShiftSlope);
    const RealNum myDirectionPointPerpMag =
        Point2D::getPerpendicularMagnitude(myDirectionPoint, inputShiftSlope);
    return sufficientlyClose(originPerpMag, myDirectionPointPerpMag) ? 0
           : originPerpMag > myDirectionPointPerpMag                 ? 1
                                                                     : -1;
  }();
  if (testScenario != 0 && !sufficientlySmall(inputShiftDistance)) {
    std::unordered_map<const BezierCurveQ *, CritsAndValues> placeholder;
    for (const CritInterval &currentInterval :
         getCritIntervals(inputBeforeMove.getEdges(),
                          ShiftValueCalculation(inputShiftSlope,
                                                inputShiftRight, inputShiftUp),
                          false, placeholder)) {
      if (!currentInterval.relevant) {
        continue;
      }
      const Point2D streakStart(currentInterval.edges.front().edge->valueAt(
          currentInterval.edges.front().startParam));
      const RealNum streakStartPerpMag =
          Point2D::getPerpendicularMagnitude(streakStart, inputShiftSlope);
      const Point2D streakEnd(currentInterval.edges.back().edge->valueAt(
          currentInterval.edges.back().endParam));
      const RealNum streakEndPerpMag =
          Point2D::getPerpendicularMagnitude(streakEnd, inputShiftSlope);
      const RealNum inputShiftPerpSlope = (-1.0) / inputShiftSlope;
      const bool rightAndUpFromStart =
          (Point2D::getPerpendicularMagnitude(
               streakStart.shift(100, inputShiftPerpSlope, true, true),
               inputShiftSlope) > streakStartPerpMag) ==
          (streakEndPerpMag > streakStartPerpMag);
      const bool useStart =
          (testScenario == 1 && streakStartPerpMag > streakEndPerpMag) ||
          (testScenario == -1 && streakStartPerpMag < streakEndPerpMag);
      const Point2D shapeStart(useStart ? streakStart : streakEnd);
      const bool insideIsRightAndUp =
          useStart ? rightAndUpFromStart : !rightAndUpFromStart;
      const Point2D edgeHalfWay(
          shapeStart.shift(inputShiftDistance / 2.0, inputShiftSlope,
                           inputShiftRight, inputShiftUp));
      const Shape streakEndShape(
          {shapeStart, shapeStart.shift(inputShiftDistance, inputShiftSlope,
                                        inputShiftRight, inputShiftUp)},
          {edgeHalfWay,
           edgeHalfWay.shift(
               std::max(static_cast<RealNum>(100.0), inputShiftDistance),
               inputShiftPerpSlope, insideIsRightAndUp, insideIsRightAndUp)});
      ShiftAgainstResult currentResult;
      shiftAgainst(streakEndShape, myShiftSlope, myShiftRight, myShiftUp,
                   currentResult);
      if (currentResult.stationaryEdge == &(streakEndShape.getEdges()[0]) &&
          (result < 0 || currentResult.movingEdgeResult.distance < result)) {
        result = currentResult.movingEdgeResult.distance;
      }
    }
  }
  {
    ShiftAgainstResult currentResult;
    shiftAgainst(inputBeforeMove.shift(inputShiftDistance, inputShiftSlope,
                                       inputShiftRight, inputShiftUp),
                 myShiftSlope, myShiftRight, myShiftUp, currentResult);
    if (currentResult.stationaryEdge != nullptr &&
        (result < 0 || currentResult.movingEdgeResult.distance < result)) {
      result = currentResult.movingEdgeResult.distance;
    }
  }
  return result;
}

RealNum Shape::shiftAgainstAfterRotating(
    const Shape &inputBeforeMove, const RealNum &inputRotationAngle,
    const Point2D &inputRotationFulcrum, bool inputRotationClockwise,
    const RealNum &myShiftSlope, bool myShiftRight, bool myShiftUp) const {
  RealNum result = -1;
  if (!sufficientlySmall(inputRotationAngle)) {
    const Point2D inputRotationFulcrumPV(inputRotationFulcrum.getX(),
                                         inputRotationFulcrum.getY() + 100.0);
    std::unordered_map<const BezierCurveQ *, CritsAndValues>
        myEdgeCritsAndValues;
    const std::vector<CritInterval> myCritIntervals(getCritIntervals(
        getEdges(),
        ShiftValueCalculation(myShiftSlope, myShiftRight, myShiftUp), false,
        myEdgeCritsAndValues));
    std::unordered_map<const BezierCurveQ *, CritsAndValues> placeholder;
    for (const CritInterval &inputCurrentInterval :
         getCritIntervals(inputBeforeMove.getEdges(),
                          RotateValueCalculation(inputRotationFulcrum,
                                                 inputRotationClockwise),
                          false, placeholder)) {
      if (!inputCurrentInterval.relevant) {
        continue;
      }
      for (const Point2D &currentIntervalEndpoint :
           {inputCurrentInterval.edges.front().edge->valueAt(
                inputCurrentInterval.edges.front().startParam),
            inputCurrentInterval.edges.back().edge->valueAt(
                inputCurrentInterval.edges.back().endParam)}) {
        if (currentIntervalEndpoint == inputRotationFulcrum) {
          continue;
        }
        const CircleArc currentArc(getArcForPointRotation(
            inputRotationFulcrum, currentIntervalEndpoint, inputRotationAngle,
            inputRotationClockwise));
        const CritsAndValues currentArcCrits(
            currentArc.getPerpendicularMagnitudeCritsAndValues(myShiftSlope));
        const auto currentArcPerpMagRange = std::minmax_element(
            currentArcCrits.critsAndValues.cbegin(),
            currentArcCrits.critsAndValues.cend(), compareSecond);
        for (const CritInterval &myCritInterval : myCritIntervals) {
          if (!myCritInterval.relevant) {
            continue;
          }
          for (const CritInterval::CritIntervalEdgeEntry &myCurrentEdge :
               myCritInterval.edges) {
            const CritsAndValues *const currentEdgeCrits =
                &(myEdgeCritsAndValues[myCurrentEdge.edge]);
            const auto currentEdgePerpMagRange = std::minmax_element(
                currentEdgeCrits->critsAndValues.cbegin(),
                currentEdgeCrits->critsAndValues.cend(), compareSecond);
            if (currentArcPerpMagRange.first->second >
                    currentEdgePerpMagRange.second->second ||
                sufficientlyClose(currentArcPerpMagRange.first->second,
                                  currentEdgePerpMagRange.second->second) ||
                currentArcPerpMagRange.second->second <
                    currentEdgePerpMagRange.first->second ||
                sufficientlyClose(currentArcPerpMagRange.second->second,
                                  currentEdgePerpMagRange.first->second)) {
              continue;
            }
            RealNum currentOutputDistance, currentOutputParam;
            Point2D currentOutputCirclePoint;
            myCurrentEdge.edge->shiftAgainstCircleArc(
                currentArc, myShiftSlope, myShiftRight, myShiftUp,
                currentOutputDistance, currentOutputParam,
                currentOutputCirclePoint);
            if (currentOutputDistance >= 0 &&
                (result < 0 || result > currentOutputDistance)) {
              result = currentOutputDistance;
            }
          }
        }
      }
    }
  }
  {
    ShiftAgainstResult currentResult;
    shiftAgainst(
        inputBeforeMove.rotate(inputRotationFulcrum,
                               inputRotationAngle *
                                   (inputRotationClockwise ? (-1.0) : 1.0)),
        myShiftSlope, myShiftRight, myShiftUp, currentResult);
    if (currentResult.stationaryEdge != nullptr &&
        (result < 0 || currentResult.movingEdgeResult.distance < result)) {
      result = currentResult.movingEdgeResult.distance;
    }
  }
  return result;
}

RealNum Shape::rotateAgainstAfterShifting(
    const Shape &inputBeforeMove, const RealNum &inputShiftDistance,
    const RealNum &inputShiftSlope, bool inputShiftRight, bool inputShiftUp,
    const Point2D &myRotationFulcrum, bool myRotationClockwise) const {
  RealNum result = -1;
  if (!sufficientlySmall(inputShiftDistance)) {
    const RealNum perpSlope = (-1.0) / inputShiftSlope;
    const bool upRightIncPerpMag = [&inputShiftSlope, &perpSlope]() -> bool {
      const Point2D origin(0, 0);
      return Point2D::getPerpendicularMagnitude(
                 origin.shift(100, perpSlope, true, true), inputShiftSlope) >
             Point2D::getPerpendicularMagnitude(origin, inputShiftSlope);
    }();
    std::unordered_map<const BezierCurveQ *, CritsAndValues> placeholder;
    for (const CritInterval &inputCurrentInterval :
         getCritIntervals(inputBeforeMove.getEdges(),
                          ShiftValueCalculation(inputShiftSlope,
                                                inputShiftRight, inputShiftUp),
                          false, placeholder)) {
      if (!inputCurrentInterval.relevant) {
        continue;
      }
      const Point2D intervalEndpoint1(
          inputCurrentInterval.edges.front().edge->valueAt(
              inputCurrentInterval.edges.front().startParam));
      const Point2D intervalEndpoint2(
          inputCurrentInterval.edges.back().edge->valueAt(
              inputCurrentInterval.edges.back().endParam));
      const bool endpoint2HigherPerpMag =
          Point2D::getPerpendicularMagnitude(intervalEndpoint2,
                                             inputShiftSlope) >
          Point2D::getPerpendicularMagnitude(intervalEndpoint1,
                                             inputShiftSlope);
      for (const Shape &currentTestShape :
           {Shape({intervalEndpoint1,
                   intervalEndpoint1.shift(inputShiftDistance, inputShiftSlope,
                                           inputShiftRight, inputShiftUp)},
                  {intervalEndpoint1.shift(inputShiftDistance / 2.0,
                                           inputShiftSlope, inputShiftRight,
                                           inputShiftUp),
                   intervalEndpoint1
                       .shift(inputShiftDistance / 2.0, inputShiftSlope,
                              inputShiftRight, inputShiftUp)
                       .shift(std::max(static_cast<RealNum>(100.0),
                                       inputShiftDistance),
                              perpSlope,
                              endpoint2HigherPerpMag == upRightIncPerpMag,
                              endpoint2HigherPerpMag == upRightIncPerpMag)}),
            Shape({intervalEndpoint2,
                   intervalEndpoint2.shift(inputShiftDistance, inputShiftSlope,
                                           inputShiftRight, inputShiftUp)},
                  {intervalEndpoint2.shift(inputShiftDistance / 2.0,
                                           inputShiftSlope, inputShiftRight,
                                           inputShiftUp),
                   intervalEndpoint2
                       .shift(inputShiftDistance / 2.0, inputShiftSlope,
                              inputShiftRight, inputShiftUp)
                       .shift(std::max(static_cast<RealNum>(100.0),
                                       inputShiftDistance),
                              perpSlope,
                              endpoint2HigherPerpMag != upRightIncPerpMag,
                              endpoint2HigherPerpMag != upRightIncPerpMag)})}) {
        RotateAgainstResult currentOutput;
        rotateAgainst(currentTestShape, myRotationFulcrum, myRotationClockwise,
                      currentOutput);
        if (currentOutput.stationaryEdge == &(currentTestShape.getEdges()[0]) &&
            (result < 0 || currentOutput.movingEdgeResult.angle < result)) {
          result = currentOutput.movingEdgeResult.angle;
        }
      }
    }
  }
  {
    RotateAgainstResult currentOutput;
    rotateAgainst(inputBeforeMove.shift(inputShiftDistance, inputShiftSlope,
                                        inputShiftRight, inputShiftUp),
                  myRotationFulcrum, myRotationClockwise, currentOutput);
    if (currentOutput.stationaryEdge != nullptr &&
        (result < 0 || result > currentOutput.movingEdgeResult.angle)) {
      result = currentOutput.movingEdgeResult.angle;
    }
  }
  return result;
}

RealNum Shape::rotateAgainstAfterRotating(const Shape &inputBeforeMove,
                                          const RealNum &inputRotationAngle,
                                          const Point2D &inputRotationFulcrum,
                                          bool inputRotationClockwise,
                                          const Point2D &myRotationFulcrum,
                                          bool myRotationClockwise) const {
  RealNum result = -1;
  if (!sufficientlySmall(inputRotationAngle)) {
    const Point2D inputRotationFulcrumPV(inputRotationFulcrum.getX(),
                                         inputRotationFulcrum.getY() + 100.0);
    std::unordered_map<const BezierCurveQ *, CritsAndValues>
        myEdgeCritsAndValues;
    const std::vector<CritInterval> myCritIntervals(getCritIntervals(
        getEdges(),
        RotateValueCalculation(myRotationFulcrum, myRotationClockwise), false,
        myEdgeCritsAndValues));
    std::unordered_map<const BezierCurveQ *, CritsAndValues> placeholder;
    for (const CritInterval &inputCurrentInterval :
         getCritIntervals(inputBeforeMove.getEdges(),
                          RotateValueCalculation(inputRotationFulcrum,
                                                 inputRotationClockwise),
                          false, placeholder)) {
      if (!inputCurrentInterval.relevant) {
        continue;
      }
      for (const Point2D &currentStreakEndpoint :
           {inputCurrentInterval.edges.front().edge->valueAt(
                inputCurrentInterval.edges.front().startParam),
            inputCurrentInterval.edges.back().edge->valueAt(
                inputCurrentInterval.edges.back().endParam)}) {
        if (currentStreakEndpoint == inputRotationFulcrum) {
          continue;
        }
        const CircleArc currentArc(
            getArcForPointRotation(inputRotationFulcrum, currentStreakEndpoint,
                                   inputRotationAngle, inputRotationClockwise));
        const CritsAndValues currentArcCrits(
            currentArc.getDistanceCritsAndValues(myRotationFulcrum));
        const auto currentArcDistanceRange = std::minmax_element(
            currentArcCrits.critsAndValues.cbegin(),
            currentArcCrits.critsAndValues.cend(), compareSecond);
        for (const CritInterval &myCritInterval : myCritIntervals) {
          if (!myCritInterval.relevant) {
            continue;
          }
          for (const CritInterval::CritIntervalEdgeEntry &myCurrentEdge :
               myCritInterval.edges) {
            const CritsAndValues *const currentEdgeCrits =
                &(myEdgeCritsAndValues[myCurrentEdge.edge]);
            const auto currentEdgeDistanceRange = std::minmax_element(
                currentEdgeCrits->critsAndValues.cbegin(),
                currentEdgeCrits->critsAndValues.cend(), compareSecond);
            if (currentArcDistanceRange.first->second >
                    currentEdgeDistanceRange.second->second ||
                sufficientlyClose(currentArcDistanceRange.first->second,
                                  currentEdgeDistanceRange.second->second) ||
                currentArcDistanceRange.second->second <
                    currentEdgeDistanceRange.first->second ||
                sufficientlyClose(currentArcDistanceRange.second->second,
                                  currentEdgeDistanceRange.first->second)) {
              continue;
            }
            RealNum outputAngle, outputParam;
            Point2D outputCirclePoint;
            myCurrentEdge.edge->rotateAgainstCircleArc(
                currentArc, myRotationFulcrum, myRotationClockwise, outputAngle,
                outputParam, outputCirclePoint);
            if (outputAngle >= 0 && (result < 0 || result > outputAngle)) {
              result = outputAngle;
            }
          }
        }
      }
    }
  }
  {
    RotateAgainstResult currentResult;
    rotateAgainst(
        inputBeforeMove.rotate(inputRotationFulcrum,
                               inputRotationAngle *
                                   (inputRotationClockwise ? (-1.0) : 1.0)),
        myRotationFulcrum, myRotationClockwise, currentResult);
    if (currentResult.movingEdge != nullptr &&
        (result < 0 || result > currentResult.movingEdgeResult.angle)) {
      result = currentResult.movingEdgeResult.angle;
    }
  }
  return result;
}

std::string Shape::toString() const {
  std::string output;
  for (const BezierCurveQ &current : getEdges()) {
    output += (output.length() > 0 ? ", " : "") + current.toString();
  }
  return output;
}

Shape Shape::approximateCircle(const Point2D &centre, const RealNum &radius,
                               int edges) {
  std::pair<std::vector<Point2D>, std::vector<Point2D>> circleArcPoints(
      BezierCurveQ::circleArc(centre, radius, 0, 360, edges));
  circleArcPoints.first.pop_back();
  return Shape(circleArcPoints.first, circleArcPoints.second);
}

std::ostream &operator<<(std::ostream &os, const Shape &input) {
  os << input.toString();
  return os;
}
} // namespace bezier_geometry
