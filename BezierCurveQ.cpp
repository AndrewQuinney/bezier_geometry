#include "BezierCurveQ.hpp"

#include <array>

#include "GeometryUtil.hpp"
#include "PrecompiledEquations.hpp"

namespace bezier_geometry {
namespace {
// P = (S - 2C + E)t^2 + 2(C - S)t + S
PolynomialFunction<3> paraFormCoeffsForPoints(const RealNum &startCoord,
                                              const RealNum &endCoord,
                                              const RealNum &controlCoord) {
  return PolynomialFunction<3>(
      {startCoord, RealNum(2.0 * (controlCoord - startCoord)),
       RealNum(startCoord - (2.0 * controlCoord) + endCoord)});
}

RealNum rateOfChangeAtCurveParam(
    const RealNum &parameter, const PolynomialFunction<2> &xDerivative,
    const PolynomialFunction<2>
        &yDerivative) { // The slope of the line created by this curve
  // (rise/run) at the specified parameter.
  return yDerivative.valueAt(parameter) / xDerivative.valueAt(parameter);
}

PolynomialFunction<3>
curveSlopeMatchesFulcrumSlopePF(const PolynomialFunction<3> &xPF,
                                const PolynomialFunction<3> &yPF,
                                const Point2D &fulcrum) {
  return curveSlopeMatchesFulcrumSlope(
      xPF.getCoefficient<2>(), xPF.getCoefficient<1>(), xPF.getCoefficient<0>(),
      yPF.getCoefficient<2>(), yPF.getCoefficient<1>(), yPF.getCoefficient<0>(),
      fulcrum.getX(), fulcrum.getY());
}

PolynomialFunction<5> getDistanceSquaredPF(const PolynomialFunction<3> &xPF,
                                           const PolynomialFunction<3> &yPF,
                                           const Point2D &fulcrum) {
  return distanceFromPointSquared(
      xPF.getCoefficient<2>(), xPF.getCoefficient<1>(), xPF.getCoefficient<0>(),
      yPF.getCoefficient<2>(), yPF.getCoefficient<1>(), yPF.getCoefficient<0>(),
      fulcrum.getX(), fulcrum.getY());
}

Point2D curveValueAt(const RealNum &parameter,
                     const PolynomialFunction<3> &paraFormX,
                     const PolynomialFunction<3> &paraFormY) {
  return Point2D(paraFormX.valueAt(parameter), paraFormY.valueAt(parameter));
}

void populateBlockedAngles(bool inputAngle1IsStart, const RealNum &inputAngle1,
                           const RealNum inputAngle2,
                           RealNum &outputBlockedCWVerticalAngleStart,
                           RealNum &outputBlockedCWVerticalAngleEnd) {
  outputBlockedCWVerticalAngleStart =
      inputAngle1IsStart ? inputAngle1 : inputAngle2;
  outputBlockedCWVerticalAngleEnd =
      inputAngle1IsStart ? inputAngle2 : inputAngle1;
}

bool testForOverlaps(
    const std::initializer_list<std::pair<RealNum, RealNum>> &firstIntervals,
    const std::initializer_list<std::pair<RealNum, RealNum>> &secondIntervals) {
  for (const std::pair<RealNum, RealNum> &currentFirstInterval :
       firstIntervals) {
    const RealNum lower1 =
        std::min(currentFirstInterval.first, currentFirstInterval.second);
    const RealNum upper1 =
        std::max(currentFirstInterval.first, currentFirstInterval.second);
    for (const std::pair<RealNum, RealNum> &currentSecondInterval :
         secondIntervals) {
      const RealNum lower2 =
          std::min(currentSecondInterval.first, currentSecondInterval.second);
      const RealNum upper2 =
          std::max(currentSecondInterval.first, currentSecondInterval.second);
      if (upper1 > lower2 && lower1 < upper2 &&
          !sufficientlyClose(upper1, lower2) &&
          !sufficientlyClose(lower1, upper2)) {
        return true;
      }
    }
  }
  return false;
}

PolynomialFunction<5>
getShiftRelationForSlopePF(const PolynomialFunction<3> &curve1XPF,
                           const PolynomialFunction<3> &curve1YPF,
                           const PolynomialFunction<3> &curve2XPF,
                           const PolynomialFunction<3> &curve2YPF,
                           const RealNum &slope) {
  return shiftRelationForSlope(
      curve1XPF.getCoefficient<2>(), curve1XPF.getCoefficient<1>(),
      curve1XPF.getCoefficient<0>(), curve1YPF.getCoefficient<2>(),
      curve1YPF.getCoefficient<1>(), curve1YPF.getCoefficient<0>(),
      curve2XPF.getCoefficient<2>(), curve2XPF.getCoefficient<1>(),
      curve2XPF.getCoefficient<0>(), curve2YPF.getCoefficient<2>(),
      curve2YPF.getCoefficient<1>(), curve2YPF.getCoefficient<0>(), slope);
}

PolynomialFunction<2>
shiftSubValueNumeratorPF(const PolynomialFunction<3> &curve1XPF,
                         const PolynomialFunction<3> &curve1YPF,
                         const PolynomialFunction<3> &curve2XPF,
                         const PolynomialFunction<3> &curve2YPF) {
  return shiftSubValueNumerator(
      curve1XPF.getCoefficient<1>(), curve1YPF.getCoefficient<1>(),
      curve2XPF.getCoefficient<2>(), curve2XPF.getCoefficient<1>(),
      curve2YPF.getCoefficient<2>(), curve2YPF.getCoefficient<1>());
}

PolynomialFunction<2>
shiftSubValueDenominatorPF(const PolynomialFunction<3> &curve1XPF,
                           const PolynomialFunction<3> &curve1YPF,
                           const PolynomialFunction<3> &curve2XPF,
                           const PolynomialFunction<3> &curve2YPF) {
  return shiftSubValueDenominator(
      curve1XPF.getCoefficient<2>(), curve1YPF.getCoefficient<2>(),
      curve2XPF.getCoefficient<2>(), curve2XPF.getCoefficient<1>(),
      curve2YPF.getCoefficient<2>(), curve2YPF.getCoefficient<1>());
}

bool cwLowersPerpMag(const RealNum &slope, bool right, bool up) {
  const Point2D origin(0, 0);
  const Point2D directionPoint(origin.shift(100, slope, right, up));
  return Point2D::getPerpendicularMagnitude(directionPoint, slope) >
         Point2D::getPerpendicularMagnitude(
             directionPoint.rotate(origin, -90.0), slope);
}

bool upAndRightLowersPerpMag(const RealNum &shiftSlope,
                             const RealNum &magnitudeSlope) {
  const Point2D origin(0, 0);
  const Point2D directionPoint(origin.shift(100, shiftSlope, true, true));
  return Point2D::getPerpendicularMagnitude(directionPoint, magnitudeSlope) <
         Point2D::getPerpendicularMagnitude(origin, magnitudeSlope);
}

void getCWVerticalAngleSuperSet(const RealNum &blockedCWVerticalAngleStart1,
                                const RealNum &blockedCWVerticalAngleEnd1,
                                const RealNum &blockedCWVerticalAngleStart2,
                                const RealNum &blockedCWVerticalAngleEnd2,
                                RealNum &outputBlockedCWVerticalAngleStart,
                                RealNum &outputBlockedCWVerticalAngleEnd) {
  if ((blockedCWVerticalAngleStart1 < blockedCWVerticalAngleStart2) ==
      (blockedCWVerticalAngleEnd1 < blockedCWVerticalAngleEnd2)) {
    if (blockedCWVerticalAngleEnd1 < blockedCWVerticalAngleEnd2) {
      outputBlockedCWVerticalAngleStart = blockedCWVerticalAngleStart1;
      outputBlockedCWVerticalAngleEnd = blockedCWVerticalAngleEnd2;
    } else {
      outputBlockedCWVerticalAngleStart = blockedCWVerticalAngleStart2;
      outputBlockedCWVerticalAngleEnd = blockedCWVerticalAngleEnd1;
    }
  } else {
    if (blockedCWVerticalAngleEnd1 < blockedCWVerticalAngleEnd2) {
      outputBlockedCWVerticalAngleStart = blockedCWVerticalAngleStart2;
      outputBlockedCWVerticalAngleEnd = blockedCWVerticalAngleEnd1;
    } else {
      outputBlockedCWVerticalAngleStart = blockedCWVerticalAngleStart1;
      outputBlockedCWVerticalAngleEnd = blockedCWVerticalAngleEnd2;
    }
  }
}

bool firstIsCloserToSlope(const RealNum &targetSlope, const BezierCurveQ &first,
                          const RealNum &firstParam, const BezierCurveQ &second,
                          const RealNum &secondParam) {
  const RealNum firstSlope = first.rateOfChangeAtParam(firstParam);
  const RealNum secondSlope = second.rateOfChangeAtParam(secondParam);
  if ((std::isinf(firstSlope) && std::isinf(secondSlope)) ||
      firstSlope == secondSlope) {
    return false;
  }
  const RealNum aTanTargetSlope = std::atan(targetSlope);
  RealNum firstDifference = std::abs(std::atan(firstSlope) - aTanTargetSlope);
  firstDifference =
      firstDifference > M_PI_2 ? M_PI - firstDifference : firstDifference;
  RealNum secondDifference = std::abs(std::atan(secondSlope) - aTanTargetSlope);
  secondDifference =
      secondDifference > M_PI_2 ? M_PI - secondDifference : secondDifference;
  return firstDifference < secondDifference;
}

RealNum _getCWVerticalAngle(const RealNum &inputSlope,
                            const RealNum &referenceSlope, bool referenceRight,
                            bool referenceUp, bool getDiffSlopeLowerMagnitude,
                            bool getDiffSlopeHigherMagnitude) {
  if (getDiffSlopeLowerMagnitude && getDiffSlopeHigherMagnitude) {
    throw std::string("Contradicting clockwise/counterclockwise parameters for "
                      "the CW angle.");
  }
  if (!sufficientlyCloseSlopes(inputSlope, referenceSlope)) {
    throw std::string("Reference slope is not close to the input slope: ") +
        std::to_string(inputSlope) + std::string(", ") +
        std::to_string(referenceSlope) + std::string(".");
  }
  const RealNum TEST_DISTANCE = 100.0;
  const Point2D origin(0, 0);
  const Point2D referenceDirection(
      origin.shift(TEST_DISTANCE, referenceSlope, referenceRight, referenceUp));
  Point2D direction(
      origin.shift(TEST_DISTANCE, inputSlope, referenceRight, referenceUp));
  if (direction.distanceFrom(referenceDirection) > TEST_DISTANCE) {
    direction =
        origin.shift(TEST_DISTANCE, inputSlope, !referenceRight, !referenceUp);
  }
  if (getDiffSlopeLowerMagnitude || getDiffSlopeHigherMagnitude) {
    RealNum differentSlope1;
    RealNum differentSlope2;
    sufficientlyDifferentSlopes(inputSlope, differentSlope1, differentSlope2);
    Point2D testPoint1(origin.shift(TEST_DISTANCE, differentSlope1,
                                    referenceRight, referenceUp));
    if (testPoint1.distanceFrom(direction) > TEST_DISTANCE) {
      testPoint1 = origin.shift(TEST_DISTANCE, differentSlope1, !referenceRight,
                                !referenceUp);
    }
    Point2D testPoint2(origin.shift(TEST_DISTANCE, differentSlope2,
                                    referenceRight, referenceUp));
    if (testPoint2.distanceFrom(direction) > TEST_DISTANCE) {
      testPoint2 = origin.shift(TEST_DISTANCE, differentSlope2, !referenceRight,
                                !referenceUp);
    }
    direction =
        ((Point2D::getPerpendicularMagnitude(testPoint1, referenceSlope) <
          Point2D::getPerpendicularMagnitude(testPoint2, referenceSlope)) ==
         getDiffSlopeLowerMagnitude)
            ? testPoint1
            : testPoint2;
  }
  return direction.getCWVerticalAngle(origin);
}

void getLargestBlockedCWVerticalAngle(
    const RealNum &directionSlope, bool directionRight, bool directionUp,
    const RealNum &slope1, const RealNum &slope2,
    RealNum &outputBlockedCWVerticalAngleStart,
    RealNum &outputBlockedCWVerticalAngleEnd) {
  RealNum slope1BlockedCWVerticalAngleStart, slope1BlockedCWVerticalAngleEnd,
      slope2BlockedCWVerticalAngleStart, slope2BlockedCWVerticalAngleEnd;
  {
    const bool lowerPerpMagIsCW =
        cwLowersPerpMag(directionSlope, directionRight, directionUp);
    populateBlockedAngles(
        upAndRightLowersPerpMag(slope1, directionSlope) != lowerPerpMagIsCW,
        _getCWVerticalAngle(slope1, slope1, true, true, false, false),
        _getCWVerticalAngle(slope1, slope1, false, false, false, false),
        slope1BlockedCWVerticalAngleStart, slope1BlockedCWVerticalAngleEnd);
    populateBlockedAngles(
        upAndRightLowersPerpMag(slope2, directionSlope) != lowerPerpMagIsCW,
        _getCWVerticalAngle(slope2, slope2, true, true, false, false),
        _getCWVerticalAngle(slope2, slope2, false, false, false, false),
        slope2BlockedCWVerticalAngleStart, slope2BlockedCWVerticalAngleEnd);
    debug() << "getLargestBlockedCWVerticalAngle - lowerPerpMagIsCW: "
            << lowerPerpMagIsCW
            << " upAndRightLowersPerpMag(slope1, directionSlope): "
            << upAndRightLowersPerpMag(slope1, directionSlope)
            << " upAndRightLowersPerpMag(slope2, directionSlope): "
            << upAndRightLowersPerpMag(slope2, directionSlope) << std::endl;
  }
  // Both intervals will be 180 degrees.
  debug()
      << "getLargestBlockedCWVerticalAngle - getLargestBlockedCWVerticalAngle -"
      << " slope1BlockedCWVerticalAngleStart: "
      << std::to_string(slope1BlockedCWVerticalAngleStart)
      << " slope1BlockedCWVerticalAngleEnd: "
      << std::to_string(slope1BlockedCWVerticalAngleEnd)
      << " slope2BlockedCWVerticalAngleStart: "
      << std::to_string(slope2BlockedCWVerticalAngleStart)
      << " slope2BlockedCWVerticalAngleEnd: "
      << std::to_string(slope2BlockedCWVerticalAngleEnd) << std::endl;
  getCWVerticalAngleSuperSet(
      slope1BlockedCWVerticalAngleStart, slope1BlockedCWVerticalAngleEnd,
      slope2BlockedCWVerticalAngleStart, slope2BlockedCWVerticalAngleEnd,
      outputBlockedCWVerticalAngleStart, outputBlockedCWVerticalAngleEnd);
}

void infiniteIntersectionBlockedInterval(
    const BezierCurveQ &curve, const RealNum &slope, bool right, bool up,
    RealNum &outputBlockedCWVerticalAngleStart,
    RealNum &outputBlockedCWVerticalAngleEnd) {
  const RealNum endpointsSlope =
      Point2D::getSlopeBetween(curve.valueAt(0), curve.valueAt(1));
  if (sufficientlyCloseSlopes(endpointsSlope, slope)) {
    getLargestBlockedCWVerticalAngle(
        slope, right, up, curve.rateOfChangeAtParam(0),
        curve.rateOfChangeAtParam(1), outputBlockedCWVerticalAngleStart,
        outputBlockedCWVerticalAngleEnd);
  } else {
    const RealNum targetSlope = (-1.0) / endpointsSlope;
    const Point2D origin(0, 0);
    const RealNum testAngle = Point2D::getAngleBetween(
        origin.shift(100, targetSlope, true, true),
        origin.shift(100, slope, right, up), origin, true);
    const bool targetUpAndRight = testAngle < 90 || testAngle > 270;
    getLargestBlockedCWVerticalAngle(
        targetSlope, targetUpAndRight, targetUpAndRight,
        curve.rateOfChangeAtParam(0), curve.rateOfChangeAtParam(1),
        outputBlockedCWVerticalAngleStart, outputBlockedCWVerticalAngleEnd);
  }
}

template <typename T, std::size_t MAX_SIZE>
bool getCritPosition(
    const RealNum &param, const T &valueCalculator,
    const CritsAndValues<MAX_SIZE> &critsAndValues,
    typename StaticVector<std::pair<RealNum, RealNum>, MAX_SIZE>::const_iterator
        &outputPosition,
    typename StaticVector<std::pair<RealNum, RealNum>, MAX_SIZE>::const_iterator
        &outputOtherPosition1,
    typename StaticVector<std::pair<RealNum, RealNum>, MAX_SIZE>::const_iterator
        &outputOtherPosition2) {
  outputPosition =
      std::lower_bound(critsAndValues.critsAndValues.begin(),
                       critsAndValues.critsAndValues.end(),
                       std::pair<RealNum, RealNum>(
                           param, std::numeric_limits<RealNum>::lowest()));
  typename StaticVector<std::pair<RealNum, RealNum>, MAX_SIZE>::const_iterator
      testPosition = outputPosition;
  if (outputPosition !=
          critsAndValues.critsAndValues
              .begin() // The only way this does NOT occur is when param is 0.
      && std::abs(param - (outputPosition - 1)->first) <
             std::abs(param - outputPosition->first)) {
    testPosition = (outputPosition - 1);
  }
  if (valueCalculator.paramMatches(testPosition->first, param) &&
      (testPosition == critsAndValues.critsAndValues.begin()
           ? critsAndValues.startIsCrit ? valueCalculator.isRoCZero(param)
                                        : true
       : testPosition == (critsAndValues.critsAndValues.end() - 1)
           ? critsAndValues.endIsCrit ? valueCalculator.isRoCZero(param) : true
           : valueCalculator.isRoCZero(param))) {
    outputPosition = testPosition;
    outputOtherPosition1 = testPosition == critsAndValues.critsAndValues.begin()
                               ? (testPosition + 1)
                               : (testPosition - 1);
    outputOtherPosition2 =
        testPosition == (critsAndValues.critsAndValues.end() - 1)
            ? (testPosition - 1)
            : (testPosition + 1);
    return true;
  } else {
    outputPosition = critsAndValues.critsAndValues.end();
    if (testPosition->first < param) {
      outputOtherPosition1 = testPosition;
      outputOtherPosition2 = testPosition + 1;
    } else if (testPosition->first > param) {
      outputOtherPosition1 = testPosition - 1;
      outputOtherPosition2 = testPosition;
    } else { // This can happen if the input parameter is exactly 0 or 1 and
             // not a crit according to 'isRoCZero'.
      outputOtherPosition1 =
          testPosition == critsAndValues.critsAndValues.begin()
              ? testPosition
              : (testPosition - 1);
      outputOtherPosition2 =
          testPosition == critsAndValues.critsAndValues.begin()
              ? (testPosition + 1)
              : testPosition;
    }
    return false;
  }
}

class CritCalculator {
public:
  bool paramMatches(const RealNum &targetParam,
                    const RealNum &testParam) const {
    return curve.sufficientlyCloseAlongCurve(targetParam, testParam);
  }

protected:
  CritCalculator(const BezierCurveQ &curve) : curve(curve) {}

  const BezierCurveQ &curve;
};

class PerpMagCritCalculator : public CritCalculator {
public:
  PerpMagCritCalculator(const BezierCurveQ &curve, const RealNum &targetSlope)
      : CritCalculator(curve), targetSlope(targetSlope) {}

  bool isRoCZero(const RealNum &testParam) const {
    return sufficientlyCloseSlopes(targetSlope,
                                   curve.rateOfChangeAtParam(testParam));
  }

private:
  const RealNum &targetSlope;
};

class DistanceCritCalculator : public CritCalculator {
public:
  DistanceCritCalculator(const BezierCurveQ &curve,
                         const Point2D &targetFulcrum)
      : CritCalculator(curve), targetFulcrum(targetFulcrum) {}

  bool isRoCZero(const RealNum &testParam) const {
    const Point2D curvePoint(curve.valueAt(testParam));
    if (curvePoint == targetFulcrum) {
      return true;
    }
    return sufficientlyCloseSlopes(
        curve.rateOfChangeAtParam(testParam),
        (-1.0) / Point2D::getSlopeBetween(curvePoint, targetFulcrum));
  }

private:
  const Point2D &targetFulcrum;
};

/*
 * Requires that:
 *
 * -curves intersect at their respective parameters
 * -neither curve has a slope close to the input slope at their respective
 * parameters
 * */
bool movingOverlapMostlyAwayFromShift(const BezierCurveQ &moving,
                                      const RealNum &movingParam,
                                      const BezierCurveQ &stationary,
                                      const RealNum &stationaryParam,
                                      const RealNum &slope, const bool right,
                                      const bool up) {
  const Point2D stationaryPoint(stationary.valueAt(stationaryParam));
  const RealNum stationarySlope =
      stationary.rateOfChangeAtParam(stationaryParam);
  const RealNum stationaryPointPerpMag =
      Point2D::getPerpendicularMagnitude(stationaryPoint, stationarySlope);
  const bool directionGreaterMagnitude =
      Point2D::getPerpendicularMagnitude(
          stationaryPoint.shift(100, slope, right, up), stationarySlope) >
      stationaryPointPerpMag;
  const CritsAndValues<3> movingTestCrits(
      moving.getPerpendicularMagnitudeCritsAndValues(stationarySlope));
  typename StaticVector<std::pair<RealNum, RealNum>, 3>::const_iterator
      movingCritPosition,
      movingOtherCritPosition1, movingOtherCritPosition2;
  debug() << "movingOverlapMostlyAwayFromShift -"
          << " moving: " << moving << " stationary: " << stationary
          << " stationarySlope: " << std::to_string(stationarySlope)
          << " moving slope: "
          << std::to_string(moving.rateOfChangeAtParam(movingParam))
          << " slope: " << std::to_string(slope)
          << " directionGreaterMagnitude: " << directionGreaterMagnitude
          << " stationaryPointPerpMag: "
          << std::to_string(stationaryPointPerpMag)
          << " moving point perp mag: "
          << std::to_string(Point2D::getPerpendicularMagnitude(
                 moving.valueAt(movingParam), stationarySlope))
          << " moving crits: " << [&]() -> std::string {
    std::string result;
    for (const auto &current : movingTestCrits.critsAndValues) {
      result += result.length() > 0 ? ", " : "";
      result += "(";
      result += std::to_string(current.first);
      result += ", ";
      result += std::to_string(current.second);
      result += ")=" + moving.valueAt(current.first).toString();
    }
    return result;
  }() << std::endl;
  if (getCritPosition(movingParam,
                      PerpMagCritCalculator(moving, stationarySlope),
                      movingTestCrits, movingCritPosition,
                      movingOtherCritPosition1, movingOtherCritPosition2)) {
    debug() << "movingOverlapMostlyAwayFromShift -"
            << " movingCritPosition->second: "
            << std::to_string(movingCritPosition->second)
            << " movingOtherCritPosition1->second: "
            << std::to_string(movingOtherCritPosition1->second) << std::endl;
    return !fuzzyEquals(movingOtherCritPosition1->second,
                        movingCritPosition->second) &&
           (movingOtherCritPosition1->second < movingCritPosition->second) ==
               directionGreaterMagnitude;
  } else if (std::abs(movingOtherCritPosition1->second -
                      stationaryPointPerpMag) >
             std::abs(movingOtherCritPosition2->second -
                      stationaryPointPerpMag)) {
    return !fuzzyEquals(movingOtherCritPosition1->second,
                        stationaryPointPerpMag) &&
           (movingOtherCritPosition1->second < stationaryPointPerpMag) ==
               directionGreaterMagnitude;
  } else {
    return !fuzzyEquals(movingOtherCritPosition2->second,
                        stationaryPointPerpMag) &&
           (movingOtherCritPosition2->second < stationaryPointPerpMag) ==
               directionGreaterMagnitude;
  }
}

/*
 * Returns true if the input position is at an actual critical point of the
 * curve, not just an endpoint.
 *
 * -critPosition - a pointer to an entry in a curve's returned list of crit
 * parameters and values, or an invalid reference -postionIsAtCrit - true if
 * critPosition is a valid reference and therefore at an entry in the curve's
 * crits -crits - the curve's list of crit parameters and values, referenced by
 * critPosition and positionIsAtCrit
 */
template <std::size_t MAX_SIZE>
bool atRealCrit(
    const typename StaticVector<std::pair<RealNum, RealNum>,
                                MAX_SIZE>::const_iterator &critPosition,
    const bool positionIsAtCrit, const CritsAndValues<MAX_SIZE> &crits) {
  if (!positionIsAtCrit) {
    return false;
  } else if (critPosition == crits.critsAndValues.begin()) {
    return crits.startIsCrit;
  } else if (critPosition == (crits.critsAndValues.end() - 1)) {
    return crits.endIsCrit;
  } else {
    return true;
  }
}

RealNum getStraightLineDistanceConcavity(const RealNum &perpDistanceFromCurve) {
  return 1.0 / perpDistanceFromCurve;
}

bool compareSecond(const std::pair<RealNum, RealNum> &input1,
                   const std::pair<RealNum, RealNum> &input2) {
  return input1.second < input2.second;
}

bool isFulcrumIntersection(const BezierCurveQ &curve1,
                           const BezierCurveQ &curve2,
                           const RealNum &curve1Param,
                           const RealNum &curve2Param,
                           const CritsAndValues<5> &curve1DistanceCrits,
                           const CritsAndValues<5> &curve2DistanceCrits) {
  const auto curve1MinMaxDistanceCrits = std::minmax_element(
      curve1DistanceCrits.critsAndValues.begin(),
      curve1DistanceCrits.critsAndValues.end(), compareSecond);
  const auto curve2MinMaxDistanceCrits = std::minmax_element(
      curve2DistanceCrits.critsAndValues.begin(),
      curve2DistanceCrits.critsAndValues.end(), compareSecond);
  debug() << "isFulcrumIntersection -"
          << " curve1MinMaxDistanceCrits.first->second: "
          << std::to_string(curve1MinMaxDistanceCrits.first->second)
          << " curve2MinMaxDistanceCrits.first->second: "
          << std::to_string(curve2MinMaxDistanceCrits.first->second)
          << " curve1MinMaxDistanceCrits.first->first: "
          << std::to_string(curve1MinMaxDistanceCrits.first->first)
          << " curve2MinMaxDistanceCrits.first->first: "
          << std::to_string(curve2MinMaxDistanceCrits.first->first)
          << " curve1Param: " << std::to_string(curve1Param)
          << " curve2Param: " << std::to_string(curve2Param) << std::endl;
  return (sufficientlySmall(curve1MinMaxDistanceCrits.first->second) ||
          sufficientlySmall(curve2MinMaxDistanceCrits.first->second)) &&
         curve1.sufficientlyCloseAlongCurve(
             curve1Param, curve1MinMaxDistanceCrits.first->first) &&
         curve2.sufficientlyCloseAlongCurve(
             curve2Param, curve2MinMaxDistanceCrits.first->first);
}

struct OverlappingCurveInterval {
  RealNum curveStartPerpMag;
  RealNum curveEndPerpMag;
  RealNum targetPointPerpMag;
};

/*
'curve' and 'targetCurve' intersect at their respective parameters.  Using the
slope of 'targetCurve', find the perpendicular magnitude crit interval in
'curve' that lies across their intersection.
*/
OverlappingCurveInterval
getOverlappingInterval(const BezierCurveQ &curve, const RealNum &curveParam,
                       const BezierCurveQ &targetCurve,
                       const RealNum &targetCurveParam) {
  const RealNum targetCurveSlope =
      targetCurve.rateOfChangeAtParam(targetCurveParam);
  const RealNum targetCurvePerpMag = Point2D::getPerpendicularMagnitude(
      targetCurve.valueAt(targetCurveParam), targetCurveSlope);
  const CritsAndValues<3> curvePerpMagCrits(
      curve.getPerpendicularMagnitudeCritsAndValues(targetCurveSlope));
  for (StaticVector<std::pair<RealNum, RealNum>, 3>::const_iterator
           currentCrit = curvePerpMagCrits.critsAndValues.begin() + 1;
       currentCrit != curvePerpMagCrits.critsAndValues.end(); currentCrit++) {
    if ((currentCrit - 1)->first <= curveParam &&
        currentCrit->first >= curveParam &&
        ((currentCrit - 1)->second < targetCurvePerpMag) ==
            (currentCrit->second > targetCurvePerpMag)) {
      return {std::min(currentCrit->second, (currentCrit - 1)->second),
              std::max(currentCrit->second, (currentCrit - 1)->second),
              targetCurvePerpMag};
    }
  }
  return {std::numeric_limits<RealNum>::quiet_NaN(),
          std::numeric_limits<RealNum>::quiet_NaN(), targetCurvePerpMag};
}

template <std::size_t ITERATIONS_REMAINING>
inline typename std::enable_if<(ITERATIONS_REMAINING == 0),
                               std::pair<RealNum, RealNum>>::type
twoVarNewton(
    const RealNum &tInitialGuess, const RealNum &uInitialGuess,
    __attribute__((unused))
    const std::array<std::array<RealNum, 4>, 4> &slopeDifferenceRelation,
    __attribute__((unused)) const std::array<std::array<RealNum, 4>, 3>
        &slopeDifferenceRelationTDerivative,
    __attribute__((unused)) const std::array<std::array<RealNum, 3>, 4>
        &slopeDifferenceRelationUDerivative,
    __attribute__((unused))
    const std::array<std::array<RealNum, 5>, 5> &distanceDifferenceRelation,
    __attribute__((unused))
    const PolynomialFunction<4> &distanceDifferenceRelationTDerivative,
    __attribute__((unused))
    const PolynomialFunction<4> &distanceDifferenceRelationUDerivative) {
  return {tInitialGuess, uInitialGuess};
}

template <std::size_t ITERATIONS_REMAINING>
inline typename std::enable_if<(ITERATIONS_REMAINING > 0),
                               std::pair<RealNum, RealNum>>::type
twoVarNewton(
    const RealNum &tInitialGuess, const RealNum &uInitialGuess,
    const std::array<std::array<RealNum, 4>, 4> &slopeDifferenceRelation,
    const std::array<std::array<RealNum, 4>, 3>
        &slopeDifferenceRelationTDerivative,
    const std::array<std::array<RealNum, 3>, 4>
        &slopeDifferenceRelationUDerivative,
    const std::array<std::array<RealNum, 5>, 5> &distanceDifferenceRelation,
    const PolynomialFunction<4> &distanceDifferenceRelationTDerivative,
    const PolynomialFunction<4> &distanceDifferenceRelationUDerivative) {
  const std::pair<RealNum, RealNum> result(solve2VarLinearSystem(
      tInitialGuess, uInitialGuess,
      TwoVarFunctionEvaluator::eval(slopeDifferenceRelationTDerivative,
                                    tInitialGuess, uInitialGuess),
      TwoVarFunctionEvaluator::eval(slopeDifferenceRelationUDerivative,
                                    tInitialGuess, uInitialGuess),
      TwoVarFunctionEvaluator::eval(slopeDifferenceRelation, tInitialGuess,
                                    uInitialGuess),
      distanceDifferenceRelationTDerivative.valueAt(tInitialGuess),
      distanceDifferenceRelationUDerivative.valueAt(uInitialGuess),
      TwoVarFunctionEvaluator::eval(distanceDifferenceRelation, tInitialGuess,
                                    uInitialGuess)));
  return (tInitialGuess == result.first && uInitialGuess == result.second)
             ? result
             : twoVarNewton<ITERATIONS_REMAINING - 1>(
                   result.first, result.second, slopeDifferenceRelation,
                   slopeDifferenceRelationTDerivative,
                   slopeDifferenceRelationUDerivative,
                   distanceDifferenceRelation,
                   distanceDifferenceRelationTDerivative,
                   distanceDifferenceRelationUDerivative);
}
} // namespace

const StaticVector<std::pair<RealNum, RealNum>, 4>
    BezierCurveQ::INFINITE_INTERSECTION(
        {{std::numeric_limits<RealNum>::quiet_NaN(),
          std::numeric_limits<RealNum>::quiet_NaN()}});

BezierCurveQ::BezierCurveQ(const Point2D &s, const Point2D &e, const Point2D &c)
    : control(c),
      paraFormX(paraFormCoeffsForPoints(s.getX(), e.getX(), c.getX())),
      paraFormY(paraFormCoeffsForPoints(s.getY(), e.getY(), c.getY())) {
  if (s == e) {
    throw std::string("Attempted to create a bezier curve with the same start "
                      "and end points.");
  }
  getParaFormX().minMaxValues(0, 1, minX, minXPara, maxX, maxXPara);
  getParaFormY().minMaxValues(0, 1, minY, minYPara, maxY, maxYPara);
}

BezierCurveQ BezierCurveQ::rotate(const Point2D &fulcrum,
                                  const RealNum &angle) const {
  return BezierCurveQ(valueAt(0).rotate(fulcrum, angle),
                      valueAt(1).rotate(fulcrum, angle),
                      getControl().rotate(fulcrum, angle));
}

BezierCurveQ BezierCurveQ::shift(const RealNum &distance, const RealNum &slope,
                                 bool right, bool up) const {
  return BezierCurveQ(valueAt(0).shift(distance, slope, right, up),
                      valueAt(1).shift(distance, slope, right, up),
                      getControl().shift(distance, slope, right, up));
}

bool BezierCurveQ::operator==(const BezierCurveQ &input) const {
  if (getControl() == input.getControl()) {
    return (valueAt(0) == input.valueAt(0) && valueAt(1) == input.valueAt(1)) ||
           (valueAt(1) == input.valueAt(0) && valueAt(0) == input.valueAt(1));
  }
  return false;
}

RealNum BezierCurveQ::maxDistance(const Point2D &input) const {
  return minMaxDistance(input, true, 0, 1);
}

RealNum BezierCurveQ::maxDistance(const Point2D &input, const RealNum &minParam,
                                  const RealNum &maxParam) const {
  return minMaxDistance(input, true, minParam, maxParam);
}

const RealNum &BezierCurveQ::getMaxXExtent() const { return maxX; }

const RealNum &BezierCurveQ::getMaxXPara() const { return maxXPara; }

const RealNum &BezierCurveQ::getMaxYExtent() const { return maxY; }

const RealNum &BezierCurveQ::getMaxYPara() const { return maxYPara; }

RealNum BezierCurveQ::minDistance(const Point2D &input) const {
  return minMaxDistance(input, false, 0, 1);
}

RealNum BezierCurveQ::minDistance(const Point2D &input, const RealNum &minParam,
                                  const RealNum &maxParam) const {
  return minMaxDistance(input, false, minParam, maxParam);
}

const RealNum &BezierCurveQ::getMinXExtent() const { return minX; }

const RealNum &BezierCurveQ::getMinXPara() const { return minXPara; }

const RealNum &BezierCurveQ::getMinYExtent() const { return minY; }

const RealNum &BezierCurveQ::getMinYPara() const { return minYPara; }

RealNum BezierCurveQ::rateOfChangeAtParam(const RealNum &parameter) const {
  checkParam(parameter);
  return rateOfChangeAtCurveParam(parameter, getParaFormX().getDerivative(),
                                  getParaFormY().getDerivative());
}

Point2D BezierCurveQ::getTestPointForEndIntersection(bool start) const {
  const Point2D startPoint(valueAt(0));
  return getTestPointForEndIntersection(
      start ? 0 : 1, start ? startPoint : valueAt(1), startPoint,
      rateOfChangeAtParam(start ? 0 : 1));
}

Point2D BezierCurveQ::getTestPointForEndIntersection(
    const RealNum &myParam, const Point2D &myPoint, const Point2D &myStart,
    const RealNum &myRate) const {
  bool atStart = myPoint == myStart;
  bool shiftRight = getParaFormX().getDerivative().valueAt(myParam) >= 0;
  shiftRight = atStart ? shiftRight : !shiftRight;
  bool shiftUp = getParaFormY().getDerivative().valueAt(myParam) >= 0;
  shiftUp = atStart ? shiftUp : !shiftUp;
  return myPoint.shift(100, myRate, shiftRight, shiftUp);
}

/*
This curve and the input curve intersect at their respective parameters.
Calculate how far across the other curve eache curve extends to see if this
intersection is a 'crisscross' and therefore would block movement in all
directions.
*/
bool BezierCurveQ::overlapBlocksEverything(const RealNum &myParam,
                                           const BezierCurveQ &input,
                                           const RealNum &inputParam,
                                           const bool testMineOnly) const {
  static const struct {
    bool operator()(const BezierCurveQ &curve, const RealNum &curveParam,
                    const BezierCurveQ &testCurve,
                    const RealNum &testCurveParam) const {
      const OverlappingCurveInterval overlappingCurveInterval(
          getOverlappingInterval(curve, curveParam, testCurve, testCurveParam));
      return !std::isnan(overlappingCurveInterval.curveStartPerpMag) &&
             !sufficientlyClose(overlappingCurveInterval.curveStartPerpMag,
                                overlappingCurveInterval.targetPointPerpMag) &&
             !sufficientlyClose(overlappingCurveInterval.curveEndPerpMag,
                                overlappingCurveInterval.targetPointPerpMag);
    }
  } testSlopeOverlap;
  return testSlopeOverlap(*this, myParam, input, inputParam) &&
         (testMineOnly || testSlopeOverlap(input, inputParam, *this, myParam));
}

/*
-returns true if the input curve blocks a shift in the input direction

-this curve and the input curve intersect at their respective parameters
-this curve and the input curve have very close slopes
-neither this nor the input curve have a slope close to the input slope at the
intersection point
*/
bool BezierCurveQ::differentSlopesIntersectionConcavityBlocks(
    const RealNum &slope, bool right, bool up, const RealNum &myParam,
    const BezierCurveQ &input, const RealNum &inputParam) const {
  const Point2D myPoint(valueAt(myParam));
  const Point2D myDirectionPoint(myPoint.shift(100, slope, right, up));
  const Point2D inputPoint(input.valueAt(inputParam));
  const Point2D inputDirectionPoint(inputPoint.shift(100, slope, right, up));
  const Point2D myConcavityIndicator(getConcavityIndicator(myParam));
  const Point2D inputConcavityIndicator(
      input.getConcavityIndicator(inputParam));
  debug() << "BezierCurveQ::differentSlopesIntersectionConcavityBlocks -"
          << " myPoint: " << myPoint
          << " myConcavityIndicator: " << myConcavityIndicator
          << " myDirectionPoint: " << myDirectionPoint
          << " inputPoint: " << inputPoint
          << " inputConcavityIndicator: " << inputConcavityIndicator
          << " inputDirectionPoint: " << inputDirectionPoint
          << " slope: " << std::to_string(slope) << " right: " << right
          << " up: " << up << std::endl;
  const bool myStraightLine = (myConcavityIndicator == myPoint);
  const bool inputStraightLine = (inputConcavityIndicator == inputPoint);
  if (myStraightLine && inputStraightLine) {
    /*
     * This method is being called because the two curves do NOT have an
     * infinite intersection.  If this method returns true for one direction,
     * then it must also return false for the opposite direction, and
     * unconditionally returning a 'false' here would not satisfy that
     * requirement and would be logically false since these are two straight
     * lines with a non-infinite intersection.
     *
     * This curve is considered blocked if it is mostly on the back side of the
     * input curve relative to the shift direction. This works as a tie breaker
     * when the slopes are very close and is consistent with the result of
     * checking slope angles with the shift direction.
     * */
    debug() << "BezierCurveQ::differentSlopesIntersectionConcavityBlocks - "
               "both curves are straight lines."
            << " my slope: " << std::to_string(rateOfChangeAtParam(myParam))
            << " input slope: "
            << std::to_string(input.rateOfChangeAtParam(inputParam))
            << std::endl;
    return movingOverlapMostlyAwayFromShift(*this, myParam, input, inputParam,
                                            slope, right, up);
  } else if (myStraightLine) { // I am a straight line, the input curve is not.
    /*
    The test below determines if the input curve is concave towards the input
    direction, indicating a block since this curve is definitely not.
    */
    const RealNum inputAngleBetween = Point2D::getAngleBetween(
        inputDirectionPoint, inputConcavityIndicator, inputPoint, false);
    debug() << "BezierCurveQ::differentSlopesIntersectionConcavityBlocks - I "
               "am a straight line, input is not,"
            << " inputAngleBetween: " << std::to_string(inputAngleBetween)
            << std::endl;
    return inputAngleBetween < 90.0 || inputAngleBetween > 270.0;
  } else if (inputStraightLine) { // I am not a straight line, the input curve
                                  // is.
    /*
    The test below determines if the this curve is concave away from the input
    direction, indicating a block since the input curve is definitely not.
    */
    const RealNum myAngleBetween = Point2D::getAngleBetween(
        myDirectionPoint, myConcavityIndicator, myPoint, false);
    debug() << "BezierCurveQ::differentSlopesIntersectionConcavityBlocks - I "
               "am not a straight line, the input curve is,"
            << " myAngleBetween: " << myAngleBetween << std::endl;
    return myAngleBetween > 90.0 && myAngleBetween < 270.0;
  } else { // Neither curve is a straight line.
    const RealNum myAngleBetween = Point2D::getAngleBetween(
        myDirectionPoint, myConcavityIndicator, myPoint, false);
    const RealNum inputAngleBetween = Point2D::getAngleBetween(
        inputDirectionPoint, inputConcavityIndicator, inputPoint, false);
    if (myAngleBetween > 90.0 && myAngleBetween < 270.0 &&
        (inputAngleBetween < 90.0 || inputAngleBetween > 270.0)) {
      /*
      I'm concave away from the input direction, the input is concave towards
      it, therefore this is a block.
      */
      return true;
    } else if ((myAngleBetween < 90.0 || myAngleBetween > 270.0) &&
               (inputAngleBetween > 90.0 && inputAngleBetween < 270.0)) {
      /*
      I'm concave towards the input direction, the input is concave away from
      it, therefore this is NOT a block.
      */
      return false;
    } else if ((myAngleBetween < 90.0 || myAngleBetween > 270.0) &&
               (inputAngleBetween < 90.0 || inputAngleBetween > 270.0)) {
      /*
      Both curves are concave towards the input direction, I'm blocked if I'm
      less concave than the input.
      */
      debug() << "BezierCurveQ::differentSlopesIntersectionConcavityBlocks - "
                 "both curves concave towards direction,"
              << " my cs: " << getConcavitySlope(myParam)
              << " input cs: " << input.getConcavitySlope(inputParam)
              << std::endl;
      return getConcavitySlope(myParam) < input.getConcavitySlope(inputParam);
    } else if ((myAngleBetween > 90.0 && myAngleBetween < 270.0) &&
               (inputAngleBetween > 90.0 && inputAngleBetween < 270.0)) {
      /*
      Both curves are concave away from the input direction, I'm blocked if I'm
      more concave than the input.
      */
      debug() << "BezierCurveQ::differentSlopesIntersectionConcavityBlocks - "
                 "both curves concave away from direction,"
              << " my cs: " << getConcavitySlope(myParam)
              << " input cs: " << input.getConcavitySlope(inputParam)
              << std::endl;
      return getConcavitySlope(myParam) > input.getConcavitySlope(inputParam);
    } else {
      throw std::string("UNHANDLED CASE - angles with concavity indicator.");
    }
  }
}

/*
-returns true if the input curve blocks a shift in the input direction

-this curve and the input curve intersect at their respective parameters
-the intersection point is an endpoint of this curve and may or may not be an
endpoint of the input -neither this nor the input curve have a slope close to
the input slope at the intersection point -both curves can extend in the same
perpendicular direction to the input slope at the intersection point
*/
bool BezierCurveQ::differentSlopesEndpointIntersectionBlocks(
    const RealNum &slope, bool right, bool up, const RealNum &myParam,
    bool atMyStart, const BezierCurveQ &input,
    const RealNum &inputParam) const {
  const RealNum myRate = rateOfChangeAtParam(myParam);
  const RealNum inputRate = input.rateOfChangeAtParam(inputParam);
  if (!sufficientlyCloseSlopes(myRate, inputRate)) {
    /*
    Both curves have a slope that does not match the input slope, both slopes
    are sufficiently different.
    */
    bool clockwise = false;
    RealNum myAngle;
    {
      const Point2D myPoint(valueAt(myParam));
      const Point2D directionPoint(myPoint.shift(100, slope, right, up));
      const Point2D testPoint(getTestPointForEndIntersection(atMyStart));
      myAngle = Point2D::getAngleBetween(directionPoint, testPoint, myPoint,
                                         clockwise);
      debug() << "BezierCurveQ::differentSlopesEndpointIntersectionBlocks -"
              << " myParam: " << myParam << " myPoint: " << myPoint
              << " directionPoint: " << directionPoint
              << " testPoint: " << testPoint << " myAngle: " << myAngle
              << " slope " << rateOfChangeAtParam(myParam) << " myRate "
              << std::to_string(myRate) << " inputRate "
              << std::to_string(inputRate) << std::endl;
      if (myAngle > 180.0) {
        myAngle = 360.0 - myAngle;
        clockwise = !clockwise;
      }
    }
    RealNum inputAngle;
    {
      const Point2D inputPoint(input.valueAt(inputParam));
      const Point2D directionPoint(inputPoint.shift(100, slope, right, up));
      const Point2D testPoint(inputPoint.shift(100, inputRate, true, true));
      inputAngle = Point2D::getAngleBetween(directionPoint, testPoint,
                                            inputPoint, clockwise);
      if (inputAngle > 180.0) {
        inputAngle = inputAngle - 180.0;
      }
    }
    debug() << "BezierCurveQ::differentSlopesEndpointIntersectionBlocks -"
            << " myAngle: " << std::to_string(myAngle)
            << " inputAngle: " << std::to_string(inputAngle)
            << " clockwise: " << clockwise << std::endl;
    return myAngle > inputAngle;
  } else { // Both curves have about the same, non-zero, slope.
    debug() << "BezierCurveQ::differentSlopesEndpointIntersectionBlocks - "
               "slopes are close."
            << std::endl;
    return differentSlopesIntersectionConcavityBlocks(slope, right, up, myParam,
                                                      input, inputParam);
  }
}

bool BezierCurveQ::movesTowardsFromEndpoint(bool atStart, const RealNum &slope,
                                            bool right, bool up) const {
  const Point2D curvePoint(valueAt(atStart ? 0 : 1));
  const Point2D directionPoint(curvePoint.shift(100, slope, right, up));
  const Point2D testPoint(getTestPointForEndIntersection(atStart));
  const RealNum angleBetween =
      Point2D::getAngleBetween(directionPoint, testPoint, curvePoint, false);
  debug() << "^ BezierCurveQ::movesTowardsFromEndpoint -"
          << " curvePoint: " << curvePoint
          << " directionPoint: " << directionPoint
          << " testPoint: " << testPoint
          << " angleBetween: " << std::to_string(angleBetween) << std::endl;
  return angleBetween < 90.0 || angleBetween > 270.0;
}

CritsAndValues<3> BezierCurveQ::getPerpendicularMagnitudeCritsAndValues(
    const RealNum &slope) const {
  RealNum myCrit = -1;
  {
    const RealNum slopeEquivalenceParam = paramForSlope(slope);
    if (!std::isnan(slopeEquivalenceParam) &&
        !std::isinf(slopeEquivalenceParam)) {
      if (sufficientlyClose(slopeEquivalenceParam, 0) &&
          sufficientlyCloseAlongCurve(0, slopeEquivalenceParam)) {
        myCrit = 0;
      } else if (sufficientlyClose(slopeEquivalenceParam, 1) &&
                 sufficientlyCloseAlongCurve(1, slopeEquivalenceParam)) {
        myCrit = 1;
      } else if (slopeEquivalenceParam > 0 && slopeEquivalenceParam < 1) {
        myCrit = slopeEquivalenceParam;
      }
    }
  }
  {
    CritsAndValues<3> result;
    result.critsAndValues.push_back(
        {0, Point2D::getPerpendicularMagnitude(valueAt(0), slope)});
    if (myCrit > 0.0 && myCrit < 1.0 && !straightLine()) {
      result.critsAndValues.push_back(
          {myCrit, Point2D::getPerpendicularMagnitude(valueAt(myCrit), slope)});
    }
    result.critsAndValues.push_back(
        {1, Point2D::getPerpendicularMagnitude(valueAt(1), slope)});
    result.startIsCrit = (myCrit == 0.0);
    result.endIsCrit = (myCrit == 1.0);
    return result;
  }
}

bool BezierCurveQ::curveIntersectionBlocksRotate(
    const RealNum &myParam, const CritsAndValues<5> &myDistanceCrits,
    const BezierCurveQ &input, const RealNum &inputParam,
    const CritsAndValues<5> &inputDistanceCrits, const Point2D &fulcrum,
    const bool clockwise, RealNum &outputBlockedCWVerticalAngleStart,
    RealNum &outputBlockedCWVerticalAngleEnd, bool &outputAtMyCrit,
    bool &outputAtInputCrit) const {
  outputBlockedCWVerticalAngleStart = -1;
  outputBlockedCWVerticalAngleEnd = -1;
  outputAtMyCrit = false;
  outputAtInputCrit = false;
  const Point2D myPoint(valueAt(myParam));
  {
    if (isFulcrumIntersection(*this, input, myParam, inputParam,
                              myDistanceCrits, inputDistanceCrits)) {
      debug() << "BezierCurveQ::curveIntersectionBlocksRotate - input "
                 "intersection is at fulcrum."
              << std::endl;
      outputAtMyCrit = true;
      outputAtInputCrit = true;
      if (!sufficientlyCloseSlopes(input.rateOfChangeAtParam(inputParam),
                                   rateOfChangeAtParam(myParam))) {
        debug() << "BezierCurveQ::curveIntersectionBlocksRotate - fulcrum "
                   "intersection slopes are not close"
                << std::endl;
        return false;
      }
      const RealNum mySlope = rateOfChangeAtParam(myParam);
      const RealNum inputSlope = input.rateOfChangeAtParam(inputParam);
      if (sufficientlyCloseAlongCurve(myParam, 0) ||
          sufficientlyCloseAlongCurve(myParam, 1)) {
        const Point2D myTestPoint(
            getTestPointForEndIntersection(myParam < 0.5));
        bool placeholder1, placeholder2;
        return curveIntersectionBlocksShift(
            myParam, (-1.0) / mySlope,
            clockwise == (myTestPoint.getY() >= fulcrum.getY()),
            clockwise == (myTestPoint.getX() <= fulcrum.getX()), inputParam,
            input, false, outputBlockedCWVerticalAngleStart,
            outputBlockedCWVerticalAngleEnd, placeholder1, placeholder2);
      } else {
        if (input.sufficientlyCloseAlongCurve(inputParam, 0) ||
            sufficientlyCloseAlongCurve(inputParam, 1)) {
          const Point2D inputTestPoint(
              input.getTestPointForEndIntersection(inputParam < 0.5));
          bool placeholder1, placeholder2;
          return curveIntersectionBlocksShift(
              myParam, (-1.0) / inputSlope,
              clockwise == (inputTestPoint.getY() >= fulcrum.getY()),
              clockwise == (inputTestPoint.getX() <= fulcrum.getX()),
              inputParam, input, false, outputBlockedCWVerticalAngleStart,
              outputBlockedCWVerticalAngleEnd, placeholder1, placeholder2);
        } else {
          bool placeholder1, placeholder2;
          return curveIntersectionBlocksShift(myParam, (-1.0) / mySlope, true,
                                              true, inputParam, input, false,
                                              outputBlockedCWVerticalAngleStart,
                                              outputBlockedCWVerticalAngleEnd,
                                              placeholder1, placeholder2) ||
                 curveIntersectionBlocksShift(myParam, (-1.0) / mySlope, false,
                                              false, inputParam, input, false,
                                              outputBlockedCWVerticalAngleStart,
                                              outputBlockedCWVerticalAngleEnd,
                                              placeholder1, placeholder2);
        }
      }
    }
  }
  typename StaticVector<std::pair<RealNum, RealNum>, 5>::const_iterator
      myCritPosition,
      myOtherCritPosition1, myOtherCritPosition2, inputCritPosition,
      inputOtherCritPosition1, inputOtherCritPosition2;
  const RealNum tangentShiftSlope =
      (-1.0) / Point2D::getSlopeBetween(myPoint, fulcrum);
  const bool tangentShiftRight =
      (clockwise == (myPoint.getY() > fulcrum.getY()));
  const bool tangentShiftUp = (clockwise == (myPoint.getX() < fulcrum.getX()));
  {
    const bool atMyCrit = getCritPosition(
        myParam, DistanceCritCalculator(*this, fulcrum), myDistanceCrits,
        myCritPosition, myOtherCritPosition1, myOtherCritPosition2);
    const bool atInputCrit = getCritPosition(
        inputParam, DistanceCritCalculator(input, fulcrum), inputDistanceCrits,
        inputCritPosition, inputOtherCritPosition1, inputOtherCritPosition2);
    outputAtMyCrit = atRealCrit(myCritPosition, atMyCrit, myDistanceCrits);
    outputAtInputCrit =
        atRealCrit(inputCritPosition, atInputCrit, inputDistanceCrits);
    if (!atMyCrit || !atInputCrit ||
        myCritPosition->second > myOtherCritPosition1->second ||
        inputCritPosition->second > inputOtherCritPosition1->second) {
      debug() << "BezierCurveQ::curveIntersectionBlocksRotate - returning "
                 "shift result,"
              << " myPoint: " << valueAt(myParam)
              << " inputPoint: " << input.valueAt(inputParam)
              << " atMyCrit: " << atMyCrit << " atInputCrit: " << atInputCrit
              << " myParam: " << std::to_string(myParam)
              << " inputParam: " << std::to_string(inputParam)
              << " myDistanceCrits: " << myDistanceCrits.critsAndValues
              << " inputDistanceCrits: " << inputDistanceCrits.critsAndValues
              << " my slope: " << std::to_string(rateOfChangeAtParam(myParam))
              << " input slope: "
              << std::to_string(input.rateOfChangeAtParam(inputParam))
              << " tangentShiftSlope: " << std::to_string(tangentShiftSlope)
              << " tangentShiftRight: " << tangentShiftRight
              << " tangentShiftUp: " << tangentShiftUp
              << " fulcrum: " << fulcrum << " clockwise: " << clockwise
              << std::endl;
      if ((atMyCrit && atInputCrit &&
           testForOverlaps(
               {{myCritPosition->second, myOtherCritPosition1->second},
                {myCritPosition->second, myOtherCritPosition2->second}},
               {{inputCritPosition->second, inputOtherCritPosition1->second},
                {inputCritPosition->second,
                 inputOtherCritPosition2->second}})) ||
          (atMyCrit && !atInputCrit &&
           testForOverlaps(
               {{myCritPosition->second, myOtherCritPosition1->second},
                {myCritPosition->second, myOtherCritPosition2->second}},
               {{inputOtherCritPosition1->second,
                 inputOtherCritPosition2->second}})) ||
          (!atMyCrit && atInputCrit &&
           testForOverlaps(
               {{myOtherCritPosition1->second, myOtherCritPosition2->second}},
               {{inputCritPosition->second, inputOtherCritPosition1->second},
                {inputCritPosition->second,
                 inputOtherCritPosition2->second}})) ||
          (!atMyCrit && !atInputCrit &&
           testForOverlaps(
               {{myOtherCritPosition1->second, myOtherCritPosition2->second}},
               {{inputOtherCritPosition1->second,
                 inputOtherCritPosition2->second}}))) {
        /*
         * Passing the flag to NOT tolerate magnitude overlaps since the test is
         * done here using fulcrum distances.  The distance ranges are
         * understated at larger angles from the input touch point, so they may
         * otherwise fall into the tolerable range when approximating the
         * rotation with a shift below.
         * */
        bool placeholder1, placeholder2;
        return curveIntersectionBlocksShift(
            myParam, tangentShiftSlope, tangentShiftRight, tangentShiftUp,
            inputParam, input, false, outputBlockedCWVerticalAngleStart,
            outputBlockedCWVerticalAngleEnd, placeholder1, placeholder2);
      } else {
        debug() << "BezierCurveQ::curveIntersectionBlocksRotate - insufficient "
                   "distance interval overlap,"
                << " my distance: "
                << std::to_string(myPoint.distanceFrom(fulcrum))
                << " atMyCrit: " << atMyCrit << " my interval: "
                << (atMyCrit
                        ? (std::to_string(myOtherCritPosition1->second) + "_" +
                           std::to_string(myCritPosition->second) + "_" +
                           std::to_string(myOtherCritPosition2->second))
                        : (std::to_string(myOtherCritPosition1->second) + "_" +
                           std::to_string(myOtherCritPosition2->second)))
                << " my crits: " << [&]() -> std::string {
          std::string result;
          for (const auto &current : myDistanceCrits.critsAndValues) {
            result += result.length() > 0 ? ", " : "";
            result += "(";
            result += std::to_string(current.first);
            result += ", ";
            result += std::to_string(current.second);
            result += ")=" + valueAt(current.first).toString();
          }
          return result;
        }() << " input distance: "
            << std::to_string(input.valueAt(inputParam).distanceFrom(fulcrum))
            << " atInputCrit: " << atInputCrit << " input interval: "
            << (atInputCrit
                    ? (std::to_string(inputOtherCritPosition1->second) + "_" +
                       std::to_string(inputCritPosition->second) + "_" +
                       std::to_string(inputOtherCritPosition2->second))
                    : (std::to_string(inputOtherCritPosition1->second) + "_" +
                       std::to_string(inputOtherCritPosition2->second)))
            << " input crits: " << inputDistanceCrits.critsAndValues
            << std::endl;
        return false;
      }
    }
  }
  // This is a distance crit of both curves.
  debug() << "BezierCurveQ::curveIntersectionBlocksRotate - distance crit of "
             "both curves."
          << std::endl;
  if (!testForOverlaps(
          {{myCritPosition->second, myOtherCritPosition1->second},
           {myCritPosition->second, myOtherCritPosition2->second}},
          {{inputCritPosition->second, inputOtherCritPosition1->second},
           {inputCritPosition->second, inputOtherCritPosition2->second}})) {
    return false;
  }
  const RealNum myRotationTangentSpeed =
      getRotationTangentSpeed(fulcrum, myParam);
  const RealNum inputRotationTangentSpeed =
      input.getRotationTangentSpeed(fulcrum, inputParam);
  if ((myCritPosition == myDistanceCrits.critsAndValues.begin() ||
       myCritPosition == (myDistanceCrits.critsAndValues.end() - 1)) &&
      (inputCritPosition == inputDistanceCrits.critsAndValues.begin() ||
       inputCritPosition == (inputDistanceCrits.critsAndValues.end() - 1))) {
    debug()
        << "BezierCurveQ::curveIntersectionBlocksRotate - endpoint both curves."
        << std::endl;
    const RealNum myDistanceSlope =
        getDistanceSlope(fulcrum, myParam, myRotationTangentSpeed);
    const RealNum inputDistanceSlope =
        input.getDistanceSlope(fulcrum, inputParam, inputRotationTangentSpeed);
    if (sufficientlyCloseSlopes(myDistanceSlope,
                                std::numeric_limits<RealNum>::infinity())) {
      if (sufficientlyCloseSlopes(inputDistanceSlope,
                                  std::numeric_limits<RealNum>::infinity())) {
        bool placeholder1, placeholder2;
        return curveIntersectionBlocksShift(
            myParam, tangentShiftSlope, tangentShiftRight, tangentShiftUp,
            inputParam, input, true, outputBlockedCWVerticalAngleStart,
            outputBlockedCWVerticalAngleEnd, placeholder1, placeholder2);
      } else {
        return clockwise == ((inputRotationTangentSpeed > 0) ==
                             (inputCritPosition ==
                              inputDistanceCrits.critsAndValues.begin()));
      }
    } else if (sufficientlyCloseSlopes(
                   inputDistanceSlope,
                   std::numeric_limits<RealNum>::infinity())) {
      return clockwise !=
             ((myRotationTangentSpeed > 0) ==
              (myCritPosition == myDistanceCrits.critsAndValues.begin()));
    } else {
      const bool myExtendingCW =
          (myRotationTangentSpeed > 0) ==
          (myCritPosition == myDistanceCrits.critsAndValues.begin());
      if (myExtendingCW !=
          ((inputRotationTangentSpeed > 0) ==
           (inputCritPosition == inputDistanceCrits.critsAndValues.begin()))) {
        return clockwise != myExtendingCW;
      } else if (sufficientlyCloseSlopes(
                     myDistanceSlope,
                     inputDistanceSlope)) { // Neither distance slope is near
                                            // infinite.
        Point2D targetShiftPoint(myPoint.shift(
            100, (-1.0) / rateOfChangeAtParam(myParam), true, true));
        {
          const RealNum angleBetween = Point2D::getAngleBetween(
              targetShiftPoint, fulcrum, myPoint, true);
          if ((myExtendingCW == clockwise) ==
              (angleBetween > 90 && angleBetween < 270)) {
            targetShiftPoint = targetShiftPoint.rotate(myPoint, 180);
          }
        }
        bool placeholder1, placeholder2;
        debug() << "BezierCurveQ::curveIntersectionBlocksRotate - returning "
                   "shift result for endpoints,"
                << " myExtendingCW: " << myExtendingCW << " inputExtendingCW: "
                << ((inputRotationTangentSpeed > 0) ==
                    (inputCritPosition ==
                     inputDistanceCrits.critsAndValues.begin()))
                << " myDistanceSlope: " << std::to_string(myDistanceSlope)
                << " inputDistanceSlope: " << std::to_string(inputDistanceSlope)
                << " myRotationTangentSpeed: "
                << std::to_string(myRotationTangentSpeed)
                << " inputRotationTangentSpeed: "
                << std::to_string(inputRotationTangentSpeed)
                << " myParam: " << std::to_string(myParam)
                << " inputParam: " << std::to_string(inputParam)
                << " myPoint: " << myPoint
                << " targetShiftPoint: " << targetShiftPoint << std::endl;
        return curveIntersectionBlocksShift(
            myParam, Point2D::getSlopeBetween(myPoint, targetShiftPoint),
            targetShiftPoint.getX() >= myPoint.getX(),
            targetShiftPoint.getY() >= myPoint.getY(), inputParam, input, true,
            outputBlockedCWVerticalAngleStart, outputBlockedCWVerticalAngleEnd,
            placeholder1, placeholder2);
      } else {
        return (myDistanceSlope > inputDistanceSlope) == clockwise;
      }
    }
  } else if (myCritPosition == myDistanceCrits.critsAndValues.begin() ||
             myCritPosition == (myDistanceCrits.critsAndValues.end() - 1)) {
    debug() << "BezierCurveQ::curveIntersectionBlocksRotate - endpoint of my "
               "curve, not an endpoint of the input."
            << std::endl;
    const bool myExtendingInDirection =
        ((myRotationTangentSpeed > 0) ==
         (myCritPosition == myDistanceCrits.critsAndValues.begin())) ==
        clockwise;
    const RealNum myDistanceSlope =
        getDistanceSlope(fulcrum, myParam, myRotationTangentSpeed);
    if (sufficientlyCloseSlopes(myDistanceSlope, 0)) {
      if (getDistanceConcavity(myParam, fulcrum) >
          input.getDistanceConcavity(inputParam, fulcrum)) {
        return !sufficientlyClose(inputCritPosition->second,
                                  (clockwise == (inputRotationTangentSpeed > 0)
                                       ? inputOtherCritPosition2
                                       : inputOtherCritPosition1)
                                      ->second);
      } else {
        return !myExtendingInDirection;
      }
    } else if (!sufficientlyClose(inputCritPosition->second,
                                  (clockwise == (inputRotationTangentSpeed > 0)
                                       ? inputOtherCritPosition2
                                       : inputOtherCritPosition1)
                                      ->second)) {
      if (sufficientlyCloseSlopes(myDistanceSlope,
                                  std::numeric_limits<RealNum>::infinity()) ||
          !myExtendingInDirection) {
        return true;
      } else {
        return true;
      }
    } else {
      return false;
    }
  } else if (inputCritPosition == inputDistanceCrits.critsAndValues.begin() ||
             inputCritPosition ==
                 (inputDistanceCrits.critsAndValues.end() - 1)) {
    debug() << "BezierCurveQ::curveIntersectionBlocksRotate - at input "
               "endpoint, not at my endpoint."
            << std::endl;
    const bool inputExtendingInDirection =
        ((inputRotationTangentSpeed > 0) ==
         (inputCritPosition == inputDistanceCrits.critsAndValues.begin())) ==
        clockwise;
    const RealNum inputDistanceSlope =
        input.getDistanceSlope(fulcrum, inputParam, inputRotationTangentSpeed);
    if (sufficientlyCloseSlopes(inputDistanceSlope, 0)) {
      if (input.getDistanceConcavity(inputParam, fulcrum) >
          getDistanceConcavity(myParam, fulcrum)) {
        return !sufficientlyClose(myCritPosition->second,
                                  (clockwise == (myRotationTangentSpeed > 0)
                                       ? myOtherCritPosition1
                                       : myOtherCritPosition2)
                                      ->second);
      } else {
        return inputExtendingInDirection;
      }
    } else if (!sufficientlyClose(myCritPosition->second,
                                  (clockwise == (myRotationTangentSpeed > 0)
                                       ? myOtherCritPosition1
                                       : myOtherCritPosition2)
                                      ->second)) {
      debug() << "BezierCurveQ::curveIntersectionBlocksRotate - input inside "
                 "my horseshoe."
              << std::endl;
      if (sufficientlyCloseSlopes(inputDistanceSlope,
                                  std::numeric_limits<RealNum>::infinity()) ||
          inputExtendingInDirection) {
        return true;
      } else {
        return true;
      }
    } else {
      debug()
          << "BezierCurveQ::curveIntersectionBlocksRotate - returning false."
          << std::endl;
      return false;
    }
  } else if (getDistanceConcavity(myParam, fulcrum) >
             input.getDistanceConcavity(inputParam, fulcrum)) {
    debug() << "BezierCurveQ::curveIntersectionBlocksRotate - endpoint of "
               "neither curve, my distance concavity is greater."
            << std::endl;
    return !sufficientlyClose(inputCritPosition->second,
                              (clockwise == (inputRotationTangentSpeed > 0)
                                   ? inputOtherCritPosition2
                                   : inputOtherCritPosition1)
                                  ->second);
  } else {
    debug() << "BezierCurveQ::curveIntersectionBlocksRotate - endpoint of "
               "neither curve, input distance concavity is greater."
            << std::endl;
    return !sufficientlyClose(myCritPosition->second,
                              (clockwise == (myRotationTangentSpeed > 0)
                                   ? myOtherCritPosition1
                                   : myOtherCritPosition2)
                                  ->second);
  }
}

bool BezierCurveQ::curveIntersectionBlocksShift(
    const RealNum &myParam, const RealNum &slope, bool right, bool up,
    const RealNum &inputParam, const BezierCurveQ &input,
    const bool tolerateSmallMagnitudeOverlap,
    RealNum &outputBlockedCWVerticalAngleStart,
    RealNum &outputBlockedCWVerticalAngleEnd, bool &outputAtMyCrit,
    bool &outputAtInputCrit) const {
  struct LocalFunctions {
    static bool concavityIndicatorAngleCWIsCloser(const BezierCurveQ &curve,
                                                  const RealNum &curveParam,
                                                  const RealNum &slope,
                                                  bool right, bool up) {
      const Point2D point(curve.valueAt(curveParam));
      return Point2D::getAngleBetween(curve.getConcavityIndicator(curveParam),
                                      point.shift(100, slope, right, up), point,
                                      true) < 180;
    }
  };
  outputBlockedCWVerticalAngleStart = -1;
  outputBlockedCWVerticalAngleEnd = -1;
  outputAtMyCrit = false;
  outputAtInputCrit = false;
  CritsAndValues<3> myPerpCrits(getPerpendicularMagnitudeCritsAndValues(slope));
  typename StaticVector<std::pair<RealNum, RealNum>, 3>::const_iterator
      myCritPosition,
      myOtherCritPosition1, myOtherCritPosition2;
  const bool atMyCrit = getCritPosition(
      myParam, PerpMagCritCalculator(*this, slope), myPerpCrits, myCritPosition,
      myOtherCritPosition1, myOtherCritPosition2);
  CritsAndValues<3> inputPerpCrits(
      input.getPerpendicularMagnitudeCritsAndValues(slope));
  typename StaticVector<std::pair<RealNum, RealNum>, 3>::const_iterator
      inputCritPosition,
      inputOtherCritPosition1, inputOtherCritPosition2;
  const bool atInputCrit = getCritPosition(
      inputParam, PerpMagCritCalculator(input, slope), inputPerpCrits,
      inputCritPosition, inputOtherCritPosition1, inputOtherCritPosition2);
  /*
  Crit values at the curve endpoints will have a certain amount of lenience in
  regard to whether or not they are considerd crits.  Therefore, if the touch
  point is at on curve's endpoint and not at an endpoint of the other, there
  could be a mismatch on the crit flags because one curve may have a true value
  simply by virtue of it being at an endpoint while it is not necessarily closer
  to the input slope.

  This is corrected for here by testing for these conditions then checking if
  the crit value is actually closer to the input slope than that of the curve
  with no crit value.  If it is not, reset the endpoint flag to false.
  */
  if (atMyCrit &&
      ((myPerpCrits.startIsCrit &&
        myCritPosition == myPerpCrits.critsAndValues.begin()) ||
       (myPerpCrits.endIsCrit &&
        myCritPosition == (myPerpCrits.critsAndValues.end() - 1))) &&
      (!atInputCrit ||
       (!inputPerpCrits.startIsCrit &&
        inputCritPosition == inputPerpCrits.critsAndValues.begin()) ||
       (!inputPerpCrits.endIsCrit &&
        inputCritPosition == (inputPerpCrits.critsAndValues.end() - 1))) &&
      !firstIsCloserToSlope(slope, *this, myParam, input, inputParam)) {
    myPerpCrits.startIsCrit =
        myCritPosition == myPerpCrits.critsAndValues.begin()
            ? false
            : myPerpCrits.startIsCrit;
    myPerpCrits.endIsCrit =
        myCritPosition == (myPerpCrits.critsAndValues.end() - 1)
            ? false
            : myPerpCrits.endIsCrit;
  } else if (atInputCrit &&
             ((inputPerpCrits.startIsCrit &&
               inputCritPosition == inputPerpCrits.critsAndValues.begin()) ||
              (inputPerpCrits.endIsCrit &&
               inputCritPosition ==
                   (inputPerpCrits.critsAndValues.end() - 1))) &&
             (!atMyCrit ||
              (!myPerpCrits.startIsCrit &&
               myCritPosition == myPerpCrits.critsAndValues.begin()) ||
              (!myPerpCrits.endIsCrit &&
               myCritPosition == (myPerpCrits.critsAndValues.end() - 1))) &&
             !firstIsCloserToSlope(slope, input, inputParam, *this, myParam)) {
    inputPerpCrits.startIsCrit =
        inputCritPosition == inputPerpCrits.critsAndValues.begin()
            ? false
            : inputPerpCrits.startIsCrit;
    inputPerpCrits.endIsCrit =
        inputCritPosition == (inputPerpCrits.critsAndValues.end() - 1)
            ? false
            : inputPerpCrits.endIsCrit;
  }
  debug() << "BezierCurveQ::curveIntersectionBlocksShift -"
          << " slope: " << std::to_string(slope) << " right: " << right
          << " up: " << up << " myParam: " << std::to_string(myParam)
          << " inputParam: " << std::to_string(inputParam)
          << " my point: " << valueAt(myParam) << " atMyCrit: " << atMyCrit
          << " myPerpCrits.startIsCrit: " << myPerpCrits.startIsCrit
          << " myPerpCrits.endIsCrit: " << myPerpCrits.endIsCrit
          << " atInputCrit: " << atInputCrit
          << " inputPerpCrits.startIsCrit: " << inputPerpCrits.startIsCrit
          << " inputPerpCrits.endIsCrit: " << inputPerpCrits.endIsCrit
          << " my slope: " << std::to_string(rateOfChangeAtParam(myParam))
          << " input slope: "
          << std::to_string(input.rateOfChangeAtParam(inputParam))
          << " my perp mag interval: "
          << (atMyCrit ? (std::to_string(myOtherCritPosition1->second) + "-" +
                          std::to_string(myCritPosition->second) + "-" +
                          std::to_string(myOtherCritPosition2->second))
                       : (std::to_string(myOtherCritPosition1->second) + "-" +
                          std::to_string(myOtherCritPosition2->second)))
          << " input perp mag interval: "
          << (atInputCrit
                  ? (std::to_string(inputOtherCritPosition1->second) + "-" +
                     std::to_string(inputCritPosition->second) + "-" +
                     std::to_string(inputOtherCritPosition2->second))
                  : (std::to_string(inputOtherCritPosition1->second) + "-" +
                     std::to_string(inputOtherCritPosition2->second)))
          << " myPerpCrits: " << myPerpCrits.critsAndValues
          << " inputPerpCrits: " << [&]() -> std::string {
    std::string result;
    for (const auto &current : inputPerpCrits.critsAndValues) {
      result += result.length() > 0 ? ", " : "";
      result += "(";
      result += std::to_string(current.first);
      result += ", ";
      result += std::to_string(current.second);
      result += ")";
    }
    return result;
  }() << std::endl;
  outputAtMyCrit = atRealCrit(myCritPosition, atMyCrit, myPerpCrits);
  outputAtInputCrit =
      atRealCrit(inputCritPosition, atInputCrit, inputPerpCrits);
  if (atMyCrit) {      // At one of my slope crits.
    if (atInputCrit) { // At one of my slope crits AND one of the input's slope
                       // crits.
      if (tolerateSmallMagnitudeOverlap &&
          !testForOverlaps(
              {{myCritPosition->second, myOtherCritPosition1->second},
               {myCritPosition->second, myOtherCritPosition2->second}},
              {{inputCritPosition->second, inputOtherCritPosition1->second},
               {inputCritPosition->second, inputOtherCritPosition2->second}})) {
        /*
        The perpendicular overlap between these two curves is either extremely
        small or nonexistant, meaning that no consequential overlap will result
        from attempting a shift in the input direction.
        */
        debug() << "BezierCurveQ::curveIntersectionBlocksShift - returning "
                   "false on small overlap."
                << std::endl;
        return false;
      } else {
        /*
        The curves extend in the same perpendicular direction from the slope at
        the intersection point.
        */
        if ((myCritPosition == myPerpCrits.critsAndValues.begin() ||
             myCritPosition == (myPerpCrits.critsAndValues.end() - 1)) &&
            (inputCritPosition == inputPerpCrits.critsAndValues.begin() ||
             inputCritPosition ==
                 (inputPerpCrits.critsAndValues.end() -
                  1))) { // The intersection is at an endpoint of both curves.
          debug() << "BezierCurveQ::curveIntersectionBlocksShift - endpoints "
                     "of both curves,"
                  << " myPerpCrits.startIsCrit: " << myPerpCrits.startIsCrit
                  << " myPerpCrits.endIsCrit: " << myPerpCrits.endIsCrit
                  << std::endl;
          const bool myGoingInDirection = movesTowardsFromEndpoint(
              myCritPosition == myPerpCrits.critsAndValues.begin(), slope,
              right, up);
          if ((myCritPosition == myPerpCrits.critsAndValues.begin() &&
               myPerpCrits.startIsCrit) ||
              (myCritPosition == (myPerpCrits.critsAndValues.end() - 1) &&
               myPerpCrits
                   .endIsCrit)) { // My intersecting endpoint is a slope crit.
            if ((inputCritPosition == inputPerpCrits.critsAndValues.begin() &&
                 inputPerpCrits.startIsCrit) ||
                (inputCritPosition ==
                     (inputPerpCrits.critsAndValues.end() - 1) &&
                 inputPerpCrits.endIsCrit)) { // Both curves' intersecting
                                              // endpoints are slope crits.
              if (myGoingInDirection ==
                  input.movesTowardsFromEndpoint(
                      inputCritPosition ==
                          inputPerpCrits.critsAndValues.begin(),
                      slope, right, up)) {
                /*
                Both curves extend from their respective endpoints in the same
                perpendicular and parallel directions to the slope.  The
                'concavity' is the distinguishing characteristic now.
                */
                if (myGoingInDirection &&
                    getConcavitySlope(myParam) >
                        input.getConcavitySlope(inputParam)) {
                  const RealNum inputRate =
                      input.rateOfChangeAtParam(inputParam);
                  populateBlockedAngles(
                      LocalFunctions::concavityIndicatorAngleCWIsCloser(
                          *this, myParam, slope, right, up),
                      _getCWVerticalAngle(
                          inputRate, slope, right, up,
                          myOtherCritPosition1
                              ->second<myCritPosition->second,
                                       myOtherCritPosition1->second>
                                  myCritPosition->second),
                      _getCWVerticalAngle(inputRate, slope, !right, !up, false,
                                          false),
                      outputBlockedCWVerticalAngleStart,
                      outputBlockedCWVerticalAngleEnd);
                  return true;
                } else if (!myGoingInDirection &&
                           getConcavitySlope(myParam) <
                               input.getConcavitySlope(inputParam)) {
                  const RealNum myRate = rateOfChangeAtParam(myParam);
                  populateBlockedAngles(
                      !LocalFunctions::concavityIndicatorAngleCWIsCloser(
                          *this, myParam, slope, right, up),
                      _getCWVerticalAngle(myRate, slope, right, up,
                                          myOtherCritPosition1->second >
                                              myCritPosition->second,
                                          myOtherCritPosition1->second <
                                              myCritPosition->second),
                      _getCWVerticalAngle(myRate, slope, !right, !up, false,
                                          false),
                      outputBlockedCWVerticalAngleStart,
                      outputBlockedCWVerticalAngleEnd);
                  return true;
                } else {
                  return false;
                }
              } else {
                /*
                The curves extend in opposite directions along the input slope
                and the same perpendicular direction from the slope at the
                intersection point.
                */
                if (!myGoingInDirection) {
                  const RealNum inputRate =
                      input.rateOfChangeAtParam(inputParam);
                  populateBlockedAngles(
                      LocalFunctions::concavityIndicatorAngleCWIsCloser(
                          *this, myParam, slope, right, up),
                      _getCWVerticalAngle(
                          inputRate, slope, right, up,
                          myOtherCritPosition1
                              ->second<myCritPosition->second,
                                       myOtherCritPosition1->second>
                                  myCritPosition->second),
                      _getCWVerticalAngle(inputRate, slope, right, up,
                                          myOtherCritPosition1->second >
                                              myCritPosition->second,
                                          myOtherCritPosition1->second <
                                              myCritPosition->second),
                      outputBlockedCWVerticalAngleStart,
                      outputBlockedCWVerticalAngleEnd);
                  return true;
                } else {
                  return false;
                }
              }
            } else {
              /*
              My intersecting endpoint is a crit, the input's is not.  Since the
              curves extend in the same perpendicular direction from the slope,
              but only my slope is equal to the input slope, I am blocked if and
              only if I extend in the opposite direction of the shift.
              */
              if (!myGoingInDirection) {
                populateBlockedAngles(
                    LocalFunctions::concavityIndicatorAngleCWIsCloser(
                        *this, myParam, slope, right, up),
                    getCWVerticalAngleForEndpoint(
                        input,
                        inputCritPosition ==
                            inputPerpCrits.critsAndValues.begin(),
                        true),
                    _getCWVerticalAngle(
                        rateOfChangeAtParam(myParam), slope, right, up,
                        myOtherCritPosition1->second > myCritPosition->second,
                        myOtherCritPosition1->second < myCritPosition->second),
                    outputBlockedCWVerticalAngleStart,
                    outputBlockedCWVerticalAngleEnd);
                return true;
              } else {
                return false;
              }
            }
          } else { // My intersecting endpoint is NOT a slope crit.
            const bool inputGoingInDirection = input.movesTowardsFromEndpoint(
                inputCritPosition == inputPerpCrits.critsAndValues.begin(),
                slope, right, up);
            if ((inputCritPosition == inputPerpCrits.critsAndValues.begin() &&
                 inputPerpCrits.startIsCrit) ||
                (inputCritPosition ==
                     (inputPerpCrits.critsAndValues.end() - 1) &&
                 inputPerpCrits.endIsCrit)) { // My intersecting endpoint is NOT
                                              // a slope crit, the input's is.
              /*
              Since both curves extend in the same perpendicular direction from
              the input slope and only the input is parallel to the input slope,
              I'm blocked if it extends along the input direction.
              */
              if (inputGoingInDirection) {
                populateBlockedAngles(
                    !LocalFunctions::concavityIndicatorAngleCWIsCloser(
                        input, inputParam, slope, right, up),
                    getCWVerticalAngleForEndpoint(
                        *this,
                        myCritPosition == myPerpCrits.critsAndValues.begin(),
                        false),
                    _getCWVerticalAngle(
                        input.rateOfChangeAtParam(inputParam), slope, right, up,
                        inputOtherCritPosition1
                            ->second<inputCritPosition->second,
                                     inputOtherCritPosition1->second>
                                inputCritPosition->second),
                    outputBlockedCWVerticalAngleStart,
                    outputBlockedCWVerticalAngleEnd);
                return true;
              } else {
                return false;
              }
            } else { // Neither curves' intersecting endpoints are vertical
                     // crits.
              debug() << "BezierCurveQ::curveIntersectionBlocksShift - both "
                         "curves' endpoints, neither are crits."
                      << std::endl;
              if (differentSlopesEndpointIntersectionBlocks(
                      slope, right, up, myParam,
                      myCritPosition == myPerpCrits.critsAndValues.begin(),
                      input, inputParam)) {
                populateBlockedAngles(
                    cwLowersPerpMag(slope, right, up) ==
                        (inputOtherCritPosition1->second >
                         inputCritPosition->second),
                    getCWVerticalAngleForEndpoint(
                        input,
                        inputCritPosition ==
                            inputPerpCrits.critsAndValues.begin(),
                        true),
                    getCWVerticalAngleForEndpoint(
                        *this,
                        myCritPosition == myPerpCrits.critsAndValues.begin(),
                        false),
                    outputBlockedCWVerticalAngleStart,
                    outputBlockedCWVerticalAngleEnd);
                return true;
              } else {
                return false;
              }
            }
          }
        } else if (myCritPosition == myPerpCrits.critsAndValues.begin() ||
                   myCritPosition ==
                       (myPerpCrits.critsAndValues.end() -
                        1)) { // The intersection is at one of my endpoints, it
                              // is NOT at either of the input's.
          const bool myGoingInDirection = movesTowardsFromEndpoint(
              myCritPosition == myPerpCrits.critsAndValues.begin(), slope,
              right, up);
          if ((myCritPosition == myPerpCrits.critsAndValues.begin() &&
               myPerpCrits.startIsCrit) ||
              (myCritPosition == (myPerpCrits.critsAndValues.end() - 1) &&
               myPerpCrits.endIsCrit)) { // The intersection is at a slope crit
                                         // of both curves.
            if (myGoingInDirection) {    // This curve extends in the input
                                      // direction from the intersection point
                                      // (equal slope).
              /*
              Since both curves extend in the same perpendicular direction to
              the input slope and the input curve extends both towards and away
              from the input direction, this is a block if I'm more concave, and
              therefore inside the 'horseshoe' shape formed by the input curve.
              */
              if (getConcavitySlope(myParam) >
                  input.getConcavitySlope(inputParam)) {
                const RealNum inputRate = input.rateOfChangeAtParam(inputParam);
                populateBlockedAngles(
                    LocalFunctions::concavityIndicatorAngleCWIsCloser(
                        input, inputParam, slope, right, up),
                    _getCWVerticalAngle(
                        inputRate, slope, right, up,
                        inputOtherCritPosition1
                            ->second<inputCritPosition->second,
                                     inputOtherCritPosition1->second>
                                inputCritPosition->second),
                    _getCWVerticalAngle(
                        inputRate, slope, !right, !up,
                        inputOtherCritPosition1
                            ->second<inputCritPosition->second,
                                     inputOtherCritPosition1->second>
                                inputCritPosition->second),
                    outputBlockedCWVerticalAngleStart,
                    outputBlockedCWVerticalAngleEnd);
                return true;
              } else {
                return false;
              }
            } else {
              /*
              This curve extends away from the input direction from the
              intersection point (equal slope).  Since both curves extend in the
              same perpendicular direction and the input curve extends both
              towards and away from the input direction, this must be a block.
              */
              const RealNum inputRate = input.rateOfChangeAtParam(inputParam);
              if (getConcavitySlope(myParam) >
                  input.getConcavitySlope(
                      inputParam)) { // I'm inside the horseshoe shape formed
                                     // by the input.
                populateBlockedAngles(
                    LocalFunctions::concavityIndicatorAngleCWIsCloser(
                        input, inputParam, slope, right, up),
                    _getCWVerticalAngle(
                        inputRate, slope, right, up,
                        inputOtherCritPosition1
                            ->second<inputCritPosition->second,
                                     inputOtherCritPosition1->second>
                                inputCritPosition->second),
                    _getCWVerticalAngle(
                        inputRate, slope, !right, !up,
                        inputOtherCritPosition1
                            ->second<inputCritPosition->second,
                                     inputOtherCritPosition1->second>
                                inputCritPosition->second),
                    outputBlockedCWVerticalAngleStart,
                    outputBlockedCWVerticalAngleEnd);
              } else { // The input is inside the half horseshoe shape formed
                       // by me.
                populateBlockedAngles(
                    !LocalFunctions::concavityIndicatorAngleCWIsCloser(
                        input, inputParam, slope, right, up),
                    _getCWVerticalAngle(inputRate, slope, right, up,
                                        inputOtherCritPosition1->second >
                                            inputCritPosition->second,
                                        inputOtherCritPosition1->second <
                                            inputCritPosition->second),
                    _getCWVerticalAngle(inputRate, slope, !right, !up, false,
                                        false),
                    outputBlockedCWVerticalAngleStart,
                    outputBlockedCWVerticalAngleEnd);
              }
              return true;
            }
          } else {
            /*
            The interesection is at a slope crit of the input curve but not at
            one of my slope crits; intersection is at an endpoint of this curve.
            Since both curves extend in the same perpendicular direction from
            the intersection point, only the input has an equal slope, and the
            intersection is not at an endpoint of the input, this is a block
            because this curve is inside the 'horseshoe' shape created by the
            input.
            */
            if (!myGoingInDirection ||
                input.overlapBlocksEverything(inputParam, *this, myParam, true)
                /*overlapBlocksEverything(myParam, input, inputParam)*/) {
              /*
              The overlap check above is necessary because this curve may very
              closely match the input slope as well but not be considered a crit
              because it is a long straight line with a sufficiently large
              perpendicular magnitude range.  Without the above check, this
              would return true, despite the fact that the overlap is
              sufficiently small.
              */
              const RealNum inputRate = input.rateOfChangeAtParam(inputParam);
              populateBlockedAngles(
                  LocalFunctions::concavityIndicatorAngleCWIsCloser(
                      input, inputParam, slope, right, up),
                  _getCWVerticalAngle(
                      inputRate, slope, right, up,
                      inputOtherCritPosition1
                          ->second<inputCritPosition->second,
                                   inputOtherCritPosition1->second>
                              inputCritPosition->second),
                  _getCWVerticalAngle(
                      inputRate, slope, !right, !up,
                      inputOtherCritPosition1
                          ->second<inputCritPosition->second,
                                   inputOtherCritPosition1->second>
                              inputCritPosition->second),
                  outputBlockedCWVerticalAngleStart,
                  outputBlockedCWVerticalAngleEnd);
              return true;
            } else {
              return false;
            }
          }
        } else if (inputCritPosition == inputPerpCrits.critsAndValues.begin() ||
                   inputCritPosition ==
                       (inputPerpCrits.critsAndValues.end() -
                        1)) { // The intersection is NOT at either of my
                              // endpoints, it is at one of the input's.
          const bool inputGoingInDirection = input.movesTowardsFromEndpoint(
              inputCritPosition == inputPerpCrits.critsAndValues.begin(), slope,
              right, up);
          if ((inputCritPosition == inputPerpCrits.critsAndValues.begin() &&
               inputPerpCrits.startIsCrit) ||
              (inputCritPosition == (inputPerpCrits.critsAndValues.end() - 1) &&
               inputPerpCrits.endIsCrit)) { // The intersection is at a slope
                                            // crit of both curves.
            if (!inputGoingInDirection) {
              /*
              The input curve extends away from the input direction from the
              intersection point (equal slope).  Since both curves extend in the
              same perpendicular direction and this curve extends both towards
              and away from the input direction, this is a block if the input
              curve is more concave, and therefore inside the 'horseshoe' shape
              formed by this curve.
              */
              if (input.getConcavitySlope(inputParam) >
                  getConcavitySlope(myParam)) {
                const RealNum myRate = rateOfChangeAtParam(myParam);
                populateBlockedAngles(
                    !LocalFunctions::concavityIndicatorAngleCWIsCloser(
                        *this, myParam, slope, right, up),
                    _getCWVerticalAngle(
                        myRate, slope, right, up,
                        myOtherCritPosition1->second > myCritPosition->second,
                        myOtherCritPosition1->second < myCritPosition->second),
                    _getCWVerticalAngle(
                        myRate, slope, !right, !up,
                        myOtherCritPosition1->second > myCritPosition->second,
                        myOtherCritPosition1->second < myCritPosition->second),
                    outputBlockedCWVerticalAngleStart,
                    outputBlockedCWVerticalAngleEnd);
                return true;
              } else {
                return false;
              }
            } else {
              /*
              The input curve extends towards the input direction from the
              intersection point (equal slope).  Since both curves extend in the
              same perpendicular direction and this curve extends both towards
              and away from the input direction, this must be a block.
              */
              if (getConcavitySlope(myParam) >
                  input.getConcavitySlope(
                      inputParam)) { // I'm inside the half horseshoe shape
                                     // formed by the input.
                const RealNum inputRate = input.rateOfChangeAtParam(inputParam);
                populateBlockedAngles(
                    LocalFunctions::concavityIndicatorAngleCWIsCloser(
                        *this, myParam, slope, right, up),
                    _getCWVerticalAngle(
                        inputRate, slope, right, up,
                        inputOtherCritPosition1
                            ->second<inputCritPosition->second,
                                     inputOtherCritPosition1->second>
                                inputCritPosition->second),
                    _getCWVerticalAngle(inputRate, slope, !right, !up, false,
                                        false),
                    outputBlockedCWVerticalAngleStart,
                    outputBlockedCWVerticalAngleEnd);
              } else { // The input is inside the horseshoe shape formed by me.
                const RealNum myRate = rateOfChangeAtParam(myParam);
                populateBlockedAngles(
                    !LocalFunctions::concavityIndicatorAngleCWIsCloser(
                        *this, myParam, slope, right, up),
                    _getCWVerticalAngle(
                        myRate, slope, right, up,
                        myOtherCritPosition1->second > myCritPosition->second,
                        myOtherCritPosition1->second < myCritPosition->second),
                    _getCWVerticalAngle(
                        myRate, slope, !right, !up,
                        myOtherCritPosition1->second > myCritPosition->second,
                        myOtherCritPosition1->second < myCritPosition->second),
                    outputBlockedCWVerticalAngleStart,
                    outputBlockedCWVerticalAngleEnd);
              }
              return true;
            }
          } else {
            /*
            The interesection is at a slope crit of this curve but not at one of
            the input's slope crits.  The intersection is NOT at either of my
            endpoints, it is at one of the input's.

            Since both curves extend in the same perpendicular direction from
            the intersection point, only this curve has an equal slope, and the
            intersection is not at an endpoint of this curve, this is a block
            because the input curve is inside the 'horseshoe' shape created by
            this curve.
            */
            if (inputGoingInDirection ||
                overlapBlocksEverything(myParam, input, inputParam, true)
                /*overlapBlocksEverything(myParam, input, inputParam)*/) {
              /*
              The overlap check above is necessary because the input curve may
              very closely match the input slope as well but not be considered a
              crit because it is a long straight line with a sufficiently large
              perpendicular magnitude range.  Without the above check, this
              would return true, despite the fact that the overlap is
              sufficiently small.
              */
              const RealNum myRate = rateOfChangeAtParam(myParam);
              populateBlockedAngles(
                  !LocalFunctions::concavityIndicatorAngleCWIsCloser(
                      *this, myParam, slope, right, up),
                  _getCWVerticalAngle(
                      myRate, slope, right, up,
                      myOtherCritPosition1->second > myCritPosition->second,
                      myOtherCritPosition1->second < myCritPosition->second),
                  _getCWVerticalAngle(
                      myRate, slope, !right, !up,
                      myOtherCritPosition1->second > myCritPosition->second,
                      myOtherCritPosition1->second < myCritPosition->second),
                  outputBlockedCWVerticalAngleStart,
                  outputBlockedCWVerticalAngleEnd);
              return true;
            } else {
              return false;
            }
          }
        } else { // The intersection is at neither curves' endpoints.
          /*
          This also is a perpendicular magnitude crit of both curves and they
          extend in the same perpendicular direction.
          */
          if (getConcavitySlope(myParam) >
              input.getConcavitySlope(
                  inputParam)) { // I'm inside the horseshoe shape formed by
                                 // the input.
            const RealNum inputRate = input.rateOfChangeAtParam(inputParam);
            populateBlockedAngles(
                LocalFunctions::concavityIndicatorAngleCWIsCloser(
                    input, inputParam, slope, right, up),
                _getCWVerticalAngle(
                    inputRate, slope, right, up,
                    inputOtherCritPosition1
                        ->second<inputCritPosition->second,
                                 inputOtherCritPosition1->second>
                            inputCritPosition->second),
                _getCWVerticalAngle(
                    inputRate, slope, !right, !up,
                    inputOtherCritPosition1
                        ->second<inputCritPosition->second,
                                 inputOtherCritPosition1->second>
                            inputCritPosition->second),
                outputBlockedCWVerticalAngleStart,
                outputBlockedCWVerticalAngleEnd);
          } else { // The input is inside the horseshoe shape formed by me.
            const RealNum myRate = rateOfChangeAtParam(myParam);
            populateBlockedAngles(
                !LocalFunctions::concavityIndicatorAngleCWIsCloser(
                    input, inputParam, slope, right, up),
                _getCWVerticalAngle(
                    myRate, slope, right, up,
                    myOtherCritPosition1->second > myCritPosition->second,
                    myOtherCritPosition1->second < myCritPosition->second),
                _getCWVerticalAngle(
                    myRate, slope, !right, !up,
                    myOtherCritPosition1->second > myCritPosition->second,
                    myOtherCritPosition1->second < myCritPosition->second),
                outputBlockedCWVerticalAngleStart,
                outputBlockedCWVerticalAngleEnd);
          }
          return true;
        }
      }
    } else { // At one of my slope crits, NOT at one of the input's slope
             // crits.
      if (tolerateSmallMagnitudeOverlap &&
          !testForOverlaps(
              {{myCritPosition->second, myOtherCritPosition1->second},
               {myCritPosition->second, myOtherCritPosition2->second}},
              {{inputOtherCritPosition1->second,
                inputOtherCritPosition2->second}})) {
        /*
        The perpendicular overlap between these two curves is either extremely
        small or nonexistant, meaning that no consequential overlap will result
        from attempting a shift in the input direction.
        */
        debug() << "BezierCurveQ::curveIntersectionBlocksShift - returning "
                   "false on no overlap,"
                << " myPerpCrits: " << myPerpCrits.critsAndValues
                << " inputPerpCrits: " << inputPerpCrits.critsAndValues
                << std::endl;
        return false;
      } else {
        /*
        The curves extend in the same perpendicular direction and there is a
        consequential overlap in their perpendicular distance intervals.
        */
        if ((myCritPosition == myPerpCrits.critsAndValues.begin() &&
             myPerpCrits.startIsCrit) ||
            (myCritPosition == (myPerpCrits.critsAndValues.end() - 1) &&
             myPerpCrits
                 .endIsCrit)) { // Intersection is at one of this curve's
                                // endpoints, the endpoint is a slope crit.
          const bool myGoingInDirection = movesTowardsFromEndpoint(
              myCritPosition == myPerpCrits.critsAndValues.begin(), slope,
              right, up);
          if (!myGoingInDirection) {
            /*
            This curve has a slope equal to the input direction, and the input
            curve does not.  Both curves extend in the same perpendicular
            direction.
            */
            const RealNum inputRate = input.rateOfChangeAtParam(inputParam);
            populateBlockedAngles(upAndRightLowersPerpMag(inputRate, slope) !=
                                      cwLowersPerpMag(slope, right, up),
                                  _getCWVerticalAngle(inputRate, inputRate,
                                                      true, true, false, false),
                                  _getCWVerticalAngle(inputRate, inputRate,
                                                      false, false, false,
                                                      false),
                                  outputBlockedCWVerticalAngleStart,
                                  outputBlockedCWVerticalAngleEnd);
            return true;
          } else {
            return false;
          }
        } else if (myCritPosition == myPerpCrits.critsAndValues.begin() ||
                   myCritPosition == (myPerpCrits.critsAndValues.end() - 1)) {
          /*
          Intersection is at one of this curve's endpoints, the endpoint is NOT
          a slope crit. The intersection is also NOT a slope crit of the input
          curve and at neither endpoint.
          */
          debug() << "^ BezierCurveQ::curveIntersectionBlocksShift - before "
                     "calling differentSlopesEndpointIntersectionBlocks."
                  << std::endl;
          if (differentSlopesEndpointIntersectionBlocks(
                  slope, right, up, myParam,
                  myCritPosition == myPerpCrits.critsAndValues.begin(), input,
                  inputParam)) {
            debug() << "^ BezierCurveQ::curveIntersectionBlocksShift - blocked."
                    << std::endl;
            const RealNum inputRate = input.rateOfChangeAtParam(inputParam);
            populateBlockedAngles(upAndRightLowersPerpMag(inputRate, slope) !=
                                      cwLowersPerpMag(slope, right, up),
                                  _getCWVerticalAngle(inputRate, inputRate,
                                                      true, true, false, false),
                                  _getCWVerticalAngle(inputRate, inputRate,
                                                      false, false, false,
                                                      false),
                                  outputBlockedCWVerticalAngleStart,
                                  outputBlockedCWVerticalAngleEnd);
            return true;
          } else {
            debug()
                << "^ BezierCurveQ::curveIntersectionBlocksShift - not blocked."
                << std::endl;
            return false;
          }
        } else {
          /*
          Intersection is at neither curves' endpoints, but at one of my slope
          crits.  Unequal slopes at neither curves' endpoint means that there is
          a crisscross here and whether or not it actually blocks everything
          needs to be calculated.
          */
          if (overlapBlocksEverything(myParam, input, inputParam, false)) {
            outputBlockedCWVerticalAngleStart = 0;
            outputBlockedCWVerticalAngleEnd = 360.0;
            return true;
          } else {
            /*
            There is little or no overlap caused by this crisscross
            intersection, therefore it is considered to be just a touch between
            these 2 curves with the same slopes at the intersection point.
            Determine if this touch blocks the shift direction.
            */
            // This curve has a non-endpoint crit, so it must have a concavity.
            const RealNum inputRate = input.rateOfChangeAtParam(inputParam);
            const Point2D testFulcrum(valueAt(myParam));
            const Point2D testDirection(
                testFulcrum.shift(100, slope, right, up));
            const Point2D myConcavityIndicator(getConcavityIndicator(myParam));
            Point2D inputTestPoint(
                testFulcrum.shift(100, inputRate, true, true));
            if (Point2D::getAngleBetween(inputTestPoint, myConcavityIndicator,
                                         testFulcrum, true) > 180.0) {
              inputTestPoint = testFulcrum.shift(100, inputRate, false, false);
            }
            if (Point2D::getAngleBetween(inputTestPoint, testDirection,
                                         testFulcrum, true) > 180.0) {
              populateBlockedAngles(
                  upAndRightLowersPerpMag(inputRate, slope) !=
                      cwLowersPerpMag(slope, right, up),
                  _getCWVerticalAngle(inputRate, inputRate, true, true, false,
                                      false),
                  _getCWVerticalAngle(inputRate, inputRate, false, false, false,
                                      false),
                  outputBlockedCWVerticalAngleStart,
                  outputBlockedCWVerticalAngleEnd);
              return true;
            } else {
              return false;
            }
          }
        }
      }
    }
  } else {             // NOT at one of my slope crits.
    if (atInputCrit) { // NOT at one of my slope crits, at one of the input's
                       // slope crits.
      if (tolerateSmallMagnitudeOverlap &&
          !testForOverlaps(
              {{myOtherCritPosition1->second, myOtherCritPosition2->second}},
              {{inputCritPosition->second, inputOtherCritPosition1->second},
               {inputCritPosition->second, inputOtherCritPosition2->second}})) {
        /*
        The perpendicular overlap between these two curves is either extremely
        small or nonexistant, meaning that no consequential overlap will result
        from attempting a shift in the input direction.
        */
        debug() << "BezierCurveQ::curveIntersectionBlocksShift - returning "
                   "false on no overlap -"
                << " myOtherCritPosition1->second: "
                << std::to_string(myOtherCritPosition1->second)
                << " myOtherCritPosition2->second: "
                << std::to_string(myOtherCritPosition2->second)
                << " inputCritPosition->second: "
                << std::to_string(inputCritPosition->second)
                << " inputOtherCritPosition1->second: "
                << std::to_string(inputOtherCritPosition1->second)
                << " inputOtherCritPosition2->second: "
                << std::to_string(inputOtherCritPosition2->second)
                << " inputCritPosition->first: "
                << std::to_string(inputCritPosition->first)
                << " myPerpCrits: " << myPerpCrits.critsAndValues
                << " inputPerpCrits: " << inputPerpCrits.critsAndValues
                << std::endl;
        return false;
      } else {
        /*
        The curves extend in the same perpendicular direction and there is a
        consequential overlap in their perpendicular distance intervals.
        */
        if ((inputCritPosition == inputPerpCrits.critsAndValues.begin() &&
             inputPerpCrits.startIsCrit) ||
            (inputCritPosition == (inputPerpCrits.critsAndValues.end() - 1) &&
             inputPerpCrits
                 .endIsCrit)) { // Intersection is at one of the input curve's
                                // endpoints, the endpoint is a slope crit.
          const bool inputGoingInDirection = input.movesTowardsFromEndpoint(
              inputCritPosition == inputPerpCrits.critsAndValues.begin(), slope,
              right, up);
          // May need to add the overlap check below back in.
          if (inputGoingInDirection /*&& overlapBlocksEverything(myParam, input, inputParam)*/)
            {
            const RealNum myRate = rateOfChangeAtParam(myParam);
            populateBlockedAngles(
                upAndRightLowersPerpMag(myRate, slope) !=
                    cwLowersPerpMag(slope, right, up),
                _getCWVerticalAngle(myRate, myRate, true, true, false, false),
                _getCWVerticalAngle(myRate, myRate, false, false, false, false),
                outputBlockedCWVerticalAngleStart,
                outputBlockedCWVerticalAngleEnd);
            return true;
          } else {
            return false;
          }
        } else if (inputCritPosition == inputPerpCrits.critsAndValues.begin() ||
                   inputCritPosition ==
                       (inputPerpCrits.critsAndValues.end() - 1)) {
          /*
          Intersection is at one of the input curve's endpoints, the endpoint is
          NOT a vertical crit. The intersection is also NOT a vertical crit of
          this curve and at neither endpoint.
          */
          /*
          Simply using the input to call the test method and checking for the
          negative case is not totally appropriate here. Instead, the input's
          test method is called for the OPPOSITE direction and the positive case
          is checked.  This is appropriate because in certain cases, the
          negative case actually means there is no block for either curve in
          either direction (eg. two overlapping straight lines).
          */
          if (input.differentSlopesEndpointIntersectionBlocks(
                  slope, !right, !up, inputParam,
                  inputCritPosition == inputPerpCrits.critsAndValues.begin(),
                  *this, myParam)) {
            const RealNum myRate = rateOfChangeAtParam(myParam);
            populateBlockedAngles(
                upAndRightLowersPerpMag(myRate, slope) !=
                    cwLowersPerpMag(slope, right, up),
                _getCWVerticalAngle(myRate, myRate, true, true, false, false),
                _getCWVerticalAngle(myRate, myRate, false, false, false, false),
                outputBlockedCWVerticalAngleStart,
                outputBlockedCWVerticalAngleEnd);
            return true;
          } else {
            return false;
          }
        } else {
          /*
          Intersection is at neither curves' endpoints, but at one of the
          input's slope crits. Unequal slopes at neither curves' endpoint means
          that there is a crisscross here and the overlap needs to be calculated
          to see if it blocks movement in every direction.
          */
          if (overlapBlocksEverything(myParam, input, inputParam, false)) {
            outputBlockedCWVerticalAngleStart = 0;
            outputBlockedCWVerticalAngleEnd = 360.0;
            return true;
          } else {
            /*
            There is little or no overlap caused by this crisscross
            intersection, therefore it is considered to be just a touch between
            these 2 curves with the same slopes at the intersection point.
            Determine if this touch blocks the shift direction.
            */
            // The input curve has a non-endpoint crit, so it must have a
            // concavity.
            const RealNum myRate = rateOfChangeAtParam(myParam);
            const RealNum inputRate = input.rateOfChangeAtParam(inputParam);
            const Point2D testFulcrum(input.valueAt(inputParam));
            const Point2D testDirection(
                testFulcrum.shift(100, slope, right, up));
            const Point2D inputConcavityIndicator(
                input.getConcavityIndicator(inputParam));
            Point2D myTestPoint(testFulcrum.shift(100, myRate, true, true));
            if (Point2D::getAngleBetween(myTestPoint, inputConcavityIndicator,
                                         testFulcrum, true) > 180.0) {
              myTestPoint = testFulcrum.shift(100, myRate, false, false);
            }
            if (Point2D::getAngleBetween(myTestPoint, testDirection,
                                         testFulcrum, true) < 180.0) {
              RealNum myAngleStart, myAngleEnd, inputAngleStart, inputAngleEnd;
              populateBlockedAngles(
                  upAndRightLowersPerpMag(myRate, slope) !=
                      cwLowersPerpMag(slope, right, up),
                  _getCWVerticalAngle(myRate, myRate, true, true, false, false),
                  _getCWVerticalAngle(myRate, myRate, false, false, false,
                                      false),
                  myAngleStart, myAngleEnd);
              populateBlockedAngles(
                  !LocalFunctions::concavityIndicatorAngleCWIsCloser(
                      input, inputParam, slope, right, up),
                  _getCWVerticalAngle(inputRate, inputRate, right, up, false,
                                      false),
                  _getCWVerticalAngle(inputRate, inputRate, !right, !up, false,
                                      false),
                  inputAngleStart, inputAngleEnd);
              getCWVerticalAngleSuperSet(myAngleStart, myAngleEnd,
                                         inputAngleStart, inputAngleEnd,
                                         outputBlockedCWVerticalAngleStart,
                                         outputBlockedCWVerticalAngleEnd);
              return true;
            } else {
              return false;
            }
          }
        }
      }
    } else { // At neither curves' slope crits, and therefore also at neither
             // curves' endpoints.
      if (tolerateSmallMagnitudeOverlap &&
          !testForOverlaps(
              {{myOtherCritPosition1->second, myOtherCritPosition2->second}},
              {{inputOtherCritPosition1->second,
                inputOtherCritPosition2->second}})) {
        /*
        The perpendicular overlap between these two curves is either extremely
        small or nonexistant, meaning that no consequential overlap will result
        from attempting a shift in the input direction.
        */
        return false;
      } else {
        /*
        The curves extend in both perpendicular directions from the slope at the
        intersection point, and their perpendicular overlap is not
        insignificant.  Determine if the overlap at this intersection is
        significant enough to block movement in all directions.
        */
        debug() << "^ Before testing overlapBlocksEverything for neither "
                   "curves' crit."
                << std::endl;
        if (overlapBlocksEverything(myParam, input, inputParam, false)) {
          debug() << "^ overlapBlocksEverything case -"
                  << " myParam: " << std::to_string(myParam)
                  << " inputParam: " << std::to_string(inputParam) << std::endl;
          outputBlockedCWVerticalAngleStart = 0;
          outputBlockedCWVerticalAngleEnd = 360.0;
          return true;
        } else if (sufficientlyCloseSlopes(
                       rateOfChangeAtParam(myParam),
                       input.rateOfChangeAtParam(inputParam))) {
          /*
          There is little or no overlap caused by this crisscross intersection,
          therefore it is considered to be just a touch between these 2 curves
          with the same slopes at the intersection point.  Determine if this
          touch blocks the shift direction.
          */
          debug() << "^ Not everything is blocked, slopes are close."
                  << std::endl;
          if (differentSlopesIntersectionConcavityBlocks(
                  slope, right, up, myParam, input, inputParam)) {
            debug() << "^ differentSlopesIntersectionConcavityBlocks case"
                    << std::endl;
            const RealNum inputRate = input.rateOfChangeAtParam(inputParam);
            populateBlockedAngles(upAndRightLowersPerpMag(inputRate, slope) !=
                                      cwLowersPerpMag(slope, right, up),
                                  _getCWVerticalAngle(inputRate, inputRate,
                                                      true, true, false, false),
                                  _getCWVerticalAngle(inputRate, inputRate,
                                                      false, false, false,
                                                      false),
                                  outputBlockedCWVerticalAngleStart,
                                  outputBlockedCWVerticalAngleEnd);
            return true;
          } else {
            debug()
                << "^ Not everything is blocked, intersection does not block."
                << std::endl;
            return false;
          }
        } else {
          /*
           * There is little or no overlap caused by this crisscross
           * intersection, therefore it is considered to be just a touch between
           * these 2 curves, but their slopes are NOT sufficiently close.
           * Determine if this touch blocks the shift direction by seeing which
           * side of the input curve this curve is mostly on.
           * */
          const OverlappingCurveInterval myOverlapInterval(
              getOverlappingInterval(*this, myParam, input, inputParam));
          const OverlappingCurveInterval inputOverlapInterval(
              getOverlappingInterval(input, inputParam, *this, myParam));
          debug() << "BezierCurveQ::curveIntersectionBlocksShift - different "
                     "slopes, but considered just a touch,"
                  << " my curveStartPerpMag: "
                  << std::to_string(myOverlapInterval.curveStartPerpMag)
                  << " my curveEndPerpMag: "
                  << std::to_string(myOverlapInterval.curveEndPerpMag)
                  << " my targetPointPerpMag: "
                  << std::to_string(myOverlapInterval.targetPointPerpMag)
                  << " input curveStartPerpMag: "
                  << std::to_string(inputOverlapInterval.curveStartPerpMag)
                  << " input curveEndPerpMag: "
                  << std::to_string(inputOverlapInterval.curveEndPerpMag)
                  << " input targetPointPerpMag: "
                  << std::to_string(inputOverlapInterval.targetPointPerpMag)
                  << std::endl;
          if (std::isnan(myOverlapInterval.curveStartPerpMag) &&
              std::isnan(inputOverlapInterval.curveStartPerpMag)) {
            /*
             * The curves don't quite touch at their respective parameters -
             * treat this as a a single touch between 2 equal slopes.
             * */
            debug() << "BezierCurveQ::curveIntersectionBlocksShift - curves "
                       "don't quite touch."
                    << std::endl;
            if (differentSlopesIntersectionConcavityBlocks(
                    slope, right, up, myParam, input, inputParam)) {
              debug() << "BezierCurveQ::curveIntersectionBlocksShift - curves "
                         "don't quite touch, blocked on concavity."
                      << std::endl;
              const RealNum inputRate = input.rateOfChangeAtParam(inputParam);
              populateBlockedAngles(
                  upAndRightLowersPerpMag(inputRate, slope) !=
                      cwLowersPerpMag(slope, right, up),
                  _getCWVerticalAngle(inputRate, inputRate, true, true, false,
                                      false),
                  _getCWVerticalAngle(inputRate, inputRate, false, false, false,
                                      false),
                  outputBlockedCWVerticalAngleStart,
                  outputBlockedCWVerticalAngleEnd);
              return true;
            } else {
              debug() << "BezierCurveQ::curveIntersectionBlocksShift - curves "
                         "don't quite touch, not blocked."
                      << std::endl;
              return false;
            }
          } else if ((std::isnan(myOverlapInterval.curveStartPerpMag)
                          ? 0
                          : (std::min(
                                 std::abs(myOverlapInterval.curveStartPerpMag -
                                          myOverlapInterval.targetPointPerpMag),
                                 std::abs(
                                     myOverlapInterval.curveEndPerpMag -
                                     myOverlapInterval.targetPointPerpMag)) /
                             std::abs(myOverlapInterval.curveStartPerpMag -
                                      myOverlapInterval.curveEndPerpMag))) <
                     (std::isnan(inputOverlapInterval.curveStartPerpMag)
                          ? 0
                          : (std::min(
                                 std::abs(
                                     inputOverlapInterval.curveStartPerpMag -
                                     inputOverlapInterval.targetPointPerpMag),
                                 std::abs(
                                     inputOverlapInterval.curveEndPerpMag -
                                     inputOverlapInterval.targetPointPerpMag)) /
                             std::abs(inputOverlapInterval.curveStartPerpMag -
                                      inputOverlapInterval.curveEndPerpMag)))) {
            const RealNum inputRate = input.rateOfChangeAtParam(inputParam);
            const RealNum inputDirectionPointPerpMag =
                Point2D::getPerpendicularMagnitude(
                    input.valueAt(inputParam).shift(100, slope, right, up),
                    inputRate);
            debug() << "BezierCurveQ::curveIntersectionBlocksShift - closer to "
                       "my perp mag crit,"
                    << " inputDirectionPointPerpMag: "
                    << std::to_string(inputDirectionPointPerpMag) << std::endl;
            if ((inputDirectionPointPerpMag >
                 myOverlapInterval.targetPointPerpMag) ==
                (!std::isnan(myOverlapInterval.curveStartPerpMag)
                     ? (std::abs(myOverlapInterval.curveStartPerpMag -
                                 myOverlapInterval.targetPointPerpMag) <
                        std::abs(myOverlapInterval.curveEndPerpMag -
                                 myOverlapInterval.targetPointPerpMag))
                     : ([&]() -> bool {
                         const CritsAndValues<3> myPerpMagCrits(
                             getPerpendicularMagnitudeCritsAndValues(
                                 input.rateOfChangeAtParam(inputParam)));
                         const auto myPerpMagRange(std::minmax_element(
                             myPerpMagCrits.critsAndValues.begin(),
                             myPerpMagCrits.critsAndValues.end(),
                             compareSecond));
                         return std::abs(myPerpMagRange.first->second -
                                         myOverlapInterval.targetPointPerpMag) <
                                std::abs(myPerpMagRange.second->second -
                                         myOverlapInterval.targetPointPerpMag);
                       }()))) { // This curve extends away from the touch
                                // intersection in the shift direction -
                                // therefore not blocked.
              return false;
            } else {
              const RealNum myRate = rateOfChangeAtParam(myParam);
              RealNum myAngleStart, myAngleEnd, inputAngleStart, inputAngleEnd;
              populateBlockedAngles(
                  upAndRightLowersPerpMag(myRate, slope) !=
                      cwLowersPerpMag(slope, right, up),
                  _getCWVerticalAngle(myRate, myRate, true, true, false, false),
                  _getCWVerticalAngle(myRate, myRate, false, false, false,
                                      false),
                  myAngleStart, myAngleEnd);
              populateBlockedAngles(
                  upAndRightLowersPerpMag(inputRate, slope) !=
                      cwLowersPerpMag(slope, right, up),
                  _getCWVerticalAngle(inputRate, inputRate, true, true, false,
                                      false),
                  _getCWVerticalAngle(inputRate, inputRate, false, false, false,
                                      false),
                  inputAngleStart, inputAngleEnd);
              getCWVerticalAngleSuperSet(myAngleStart, myAngleEnd,
                                         inputAngleStart, inputAngleEnd,
                                         outputBlockedCWVerticalAngleStart,
                                         outputBlockedCWVerticalAngleEnd);
              return true;
            }
          } else {
            // const OverlappingCurveInterval
            // inputOverlapInterval(getOverlappingInterval(input, inputParam,
            // *this, myParam));
            const RealNum myRate = rateOfChangeAtParam(myParam);
            const RealNum myDirectionPointPerpMag =
                Point2D::getPerpendicularMagnitude(
                    valueAt(myParam).shift(100, slope, right, up), myRate);
            debug() << "BezierCurveQ::curveIntersectionBlocksShift - input "
                       "interval endpoint matches,"
                    << " input curveStartPerpMag: "
                    << std::to_string(inputOverlapInterval.curveStartPerpMag)
                    << " input curveEndPerpMag: "
                    << std::to_string(inputOverlapInterval.curveEndPerpMag)
                    << " input targetPointPerpMag: "
                    << std::to_string(inputOverlapInterval.targetPointPerpMag)
                    << " myRate: " << std::to_string(myRate)
                    << " myDirectionPointPerpMag: "
                    << std::to_string(myDirectionPointPerpMag) << std::endl;
            if ((myDirectionPointPerpMag >
                 inputOverlapInterval.targetPointPerpMag) ==
                (!std::isnan(inputOverlapInterval.curveEndPerpMag)
                     ? (std::abs(inputOverlapInterval.curveEndPerpMag -
                                 inputOverlapInterval.targetPointPerpMag) <
                        std::abs(inputOverlapInterval.curveStartPerpMag -
                                 inputOverlapInterval.targetPointPerpMag))
                     : ([&]() -> bool {
                         const CritsAndValues<3> inputPerpMagCrits(
                             input.getPerpendicularMagnitudeCritsAndValues(
                                 rateOfChangeAtParam(myParam)));
                         const auto inputPerpMagRange(std::minmax_element(
                             inputPerpMagCrits.critsAndValues.begin(),
                             inputPerpMagCrits.critsAndValues.end(),
                             compareSecond));
                         return std::abs(
                                    inputPerpMagRange.second->second -
                                    inputOverlapInterval.targetPointPerpMag) <
                                std::abs(
                                    inputPerpMagRange.first->second -
                                    inputOverlapInterval.targetPointPerpMag);
                       }()))) { // The input curve extends away from the touch
                                // intersection opposite the shift direction -
                                // therefore not blocked.
              return false;
            } else {
              const RealNum inputRate = input.rateOfChangeAtParam(inputParam);
              RealNum myAngleStart, myAngleEnd, inputAngleStart, inputAngleEnd;
              populateBlockedAngles(
                  upAndRightLowersPerpMag(myRate, slope) !=
                      cwLowersPerpMag(slope, right, up),
                  _getCWVerticalAngle(myRate, myRate, true, true, false, false),
                  _getCWVerticalAngle(myRate, myRate, false, false, false,
                                      false),
                  myAngleStart, myAngleEnd);
              populateBlockedAngles(
                  upAndRightLowersPerpMag(inputRate, slope) !=
                      cwLowersPerpMag(slope, right, up),
                  _getCWVerticalAngle(inputRate, inputRate, true, true, false,
                                      false),
                  _getCWVerticalAngle(inputRate, inputRate, false, false, false,
                                      false),
                  inputAngleStart, inputAngleEnd);
              debug() << "BezierCurveQ::curveIntersectionBlocksShift - curve "
                         "blocked intervals,"
                      << " myAngleStart: " << std::to_string(myAngleStart)
                      << " myAngleEnd: " << std::to_string(myAngleEnd)
                      << " inputAngleStart: " << std::to_string(inputAngleStart)
                      << " inputAngleEnd: " << std::to_string(inputAngleEnd)
                      << std::endl;
              getCWVerticalAngleSuperSet(myAngleStart, myAngleEnd,
                                         inputAngleStart, inputAngleEnd,
                                         outputBlockedCWVerticalAngleStart,
                                         outputBlockedCWVerticalAngleEnd);
              return true;
            }
          }
        }
      }
    }
  }
}

/*
Returns the minimum distance that this curve must be moved in the direction
indicated by 'slope' to touch the input curve at one or an infinite number of
points.

-input - the curve that this curve is being moved to touch
-slope - the angle of the direction of movement
-right - when moving along the slope, whether or not this curve is moving to the
right -up - when moving along the slope, whether or not this curve is moving up;
only used if the slope is very large and there would be very little right/left
movement.

The output angle interval indicates the SHIFT directions that would be blocked
at this curve's output parameter if this shift were to be done.
*/
void BezierCurveQ::shiftAgainst(
    const BezierCurveQ &input, const RealNum &slope, bool right, bool up,
    BezierCurveQ::ShiftAgainstResult &output) const {
  output.distance = -1;
  output.param = -1;
  output.inputParam = -1;
  output.blockedCWVerticalAngleStart = -1;
  output.blockedCWVerticalAngleEnd = -1;
  output.infiniteInitialIntersection = false;
  output.atMyCrit = false;
  output.atInputCrit = false;
  StaticVector<Point2D, 8> myTestedPoints;
  { // Intersection conditions.
    const StaticVector<std::pair<RealNum, RealNum>, 4> myIntersections(
        pointsOfIntersection(input));
    debug() << "BezierCurveQ::shiftAgainst - myIntersections:"
            << [](const StaticVector<std::pair<RealNum, RealNum>, 4>
                      &input) -> std::string {
      std::string result;
      for (const std::pair<RealNum, RealNum> &current : input) {
        result += (result.length() > 0 ? std::string(", ") : std::string()) +
                  "(" + std::to_string(current.first) + ", " +
                  std::to_string(current.second) + ")";
      }
      return result;
    }(myIntersections) << std::endl;
    if (isIntersectionInfinite(
            myIntersections)) { // Infinite intersection means 2 of the same
                                // shaped curve or two different straight
                                // lines.
      output.infiniteInitialIntersection = true;
      const CritsAndValues<3> myPerpCrits(
          getPerpendicularMagnitudeCritsAndValues(slope));
      debug()
          << "BezierCurveQ::shiftAgainst - infinite intersection, myPerpCrits: "
          << myPerpCrits.critsAndValues << std::endl;
      if (!straightLine() && myPerpCrits.critsAndValues.size() ==
                                 3) { // I have a slope equal to the input
                                      // slope and it is not at an endpoint.
        const CritsAndValues<3> inputPerpCrits(
            input.getPerpendicularMagnitudeCritsAndValues(slope));
        debug() << "BezierCurveQ::shiftAgainst - infinite intersection, "
                   "inputPerpCrits: "
                << inputPerpCrits.critsAndValues << std::endl;
        if (inputPerpCrits.critsAndValues.size() != 3) {
          throw std::string("Infinite intersection, but the input curve does "
                            "not have a perpendicular crit like me.");
        }
        output.distance = 0;
        output.param = myPerpCrits.critsAndValues[1].first;
        output.inputParam = inputPerpCrits.critsAndValues[1].first;
        populateInfiniteIntersectionShiftBlockValues(
            slope, right, up, output.blockedCWVerticalAngleStart,
            output.blockedCWVerticalAngleEnd);
        output.atMyCrit = true;
        output.atInputCrit = true;
      }
      return;
    } else {
      /*
      I touch or cross the input curve at least once - this either blocks this
      shift entirely or not at all.
      */
      for (const std::pair<RealNum, RealNum> &currentParams : myIntersections) {
        myTestedPoints.push_back(valueAt(currentParams.first));
        RealNum currentBlockedCWVerticalAngleStart,
            currentBlockedCWVerticalAngleEnd;
        bool currentAtMyCrit, currentAtInputCrit;
        if (curveIntersectionBlocksShift(currentParams.first, slope, right, up,
                                         currentParams.second, input, true,
                                         currentBlockedCWVerticalAngleStart,
                                         currentBlockedCWVerticalAngleEnd,
                                         currentAtMyCrit, currentAtInputCrit)) {
          output.distance = 0;
          output.param = currentParams.first;
          output.inputParam = currentParams.second;
          output.blockedCWVerticalAngleStart =
              currentBlockedCWVerticalAngleStart;
          output.blockedCWVerticalAngleEnd = currentBlockedCWVerticalAngleEnd;
          output.atMyCrit = currentAtMyCrit;
          output.atInputCrit = currentAtInputCrit;
          debug() << "BezierCurveQ::shiftAgainst - blocked on intersection,"
                  << " my RoC: " << rateOfChangeAtParam(output.param)
                  << " input RoC: "
                  << input.rateOfChangeAtParam(output.inputParam) << std::endl;
          return;
        }
      }
    }
  }
  StaticVector<std::pair<RealNum, RealNum>, 3> myPerpCritsSorted;
  StaticVector<std::pair<RealNum, RealNum>, 3> inputPerpCritsSorted;
  {
    myPerpCritsSorted =
        getPerpendicularMagnitudeCritsAndValues(slope).critsAndValues;
    inputPerpCritsSorted =
        input.getPerpendicularMagnitudeCritsAndValues(slope).critsAndValues;
    for (StaticVector<std::pair<RealNum, RealNum>, 3> *current :
         {&myPerpCritsSorted, &inputPerpCritsSorted}) {
      std::sort(current->begin(), current->end(), compareSecond);
    }
    if (myPerpCritsSorted.back().second <=
            inputPerpCritsSorted.front().second ||
        myPerpCritsSorted.front().second >=
            inputPerpCritsSorted.back()
                .second) { // These curves do not lie in each other's path
                           // along the input slope.
      return;
    }
  }
  { // Endpoint conditions.
    StaticVector<std::pair<RealNum, RealNum>, 8> testParams;
    for (const RealNum &currentInputParam : {0.0, 1.0}) {
      for (const RealNum &currentParam : pointShiftAgainstParams(
               input.valueAt(currentInputParam), slope, true)) {
        testParams.push_back({currentParam, currentInputParam});
      }
    }
    for (const RealNum &currentParam : {0.0, 1.0}) {
      for (const RealNum &currentInputParam :
           input.pointShiftAgainstParams(valueAt(currentParam), slope, true)) {
        testParams.push_back({currentParam, currentInputParam});
      }
    }
    debug() << "BezierCurveQ::shiftAgainst - endpoint test params: "
            << testParams << std::endl;
    for (const std::pair<RealNum, RealNum> &currentParams : testParams) {
      const Point2D myCurrentPoint(valueAt(currentParams.first));
      const Point2D inputCurrentPoint(input.valueAt(currentParams.second));
      const RealNum distanceBetween =
          myCurrentPoint.distanceFrom(inputCurrentPoint);
      if (output.distance >= 0 && output.distance < distanceBetween) {
        continue;
      }
      const BezierCurveQ meShifted(shift(distanceBetween, slope, right, up));
      debug() << "BezierCurveQ::shiftAgainst -"
              << " current distance between: "
              << std::to_string(distanceBetween) << " shifted distance: "
              << std::to_string(meShifted.valueAt(currentParams.first)
                                    .distanceFrom(inputCurrentPoint))
              << " meShifted: " << meShifted << " input: " << input
              << std::endl;
      if (meShifted.valueAt(currentParams.first)
              .distanceFrom(inputCurrentPoint) > distanceBetween) {
        debug() << "BezierCurveQ::shiftAgainst - endpoint combo is opposite "
                   "the shift direction,"
                << " myCurrentPoint: " << myCurrentPoint
                << " inputCurrentPoint: " << inputCurrentPoint << std::endl;
        /*
        The current pair of parameters indicate a point on the input curve that
        is not in the shift direction.
        */
        continue;
      }
      debug() << "BezierCurveQ::shiftAgainst - testing endpoint -"
              << " distanceBetween: " << std::to_string(distanceBetween)
              << " my param: " << currentParams.first
              << " my point: " << valueAt(currentParams.first)
              << " input param: " << currentParams.second
              << " input point: " << input.valueAt(currentParams.second)
              << std::endl;
      RealNum currentBlockedCWVerticalAngleStart,
          currentBlockedCWVerticalAngleEnd;
      bool currentAtMyCrit, currentAtInputCrit;
      if (meShifted.curveIntersectionBlocksShift(
              currentParams.first, slope, right, up, currentParams.second,
              input, true, currentBlockedCWVerticalAngleStart,
              currentBlockedCWVerticalAngleEnd, currentAtMyCrit,
              currentAtInputCrit)) {
        debug() << "BezierCurveQ::shiftAgainst - testing endpoint - shifted is "
                   "blocked,"
                << " my shifted point: "
                << meShifted.valueAt(currentParams.first) << std::endl;
        output.distance = distanceBetween;
        output.param = currentParams.first;
        output.inputParam = currentParams.second;
        output.blockedCWVerticalAngleStart = currentBlockedCWVerticalAngleStart;
        output.blockedCWVerticalAngleEnd = currentBlockedCWVerticalAngleEnd;
        output.atMyCrit = currentAtMyCrit;
        output.atInputCrit = currentAtInputCrit;
        continue;
      }
      if (!sufficientlySmall(distanceBetween) &&
          isIntersectionInfinite(meShifted.pointsOfIntersection(input))) {
        //          const RealNum overlapSize =
        //          std::min(myPerpCritsSorted.back().second,
        //            inputPerpCritsSorted.back().second)-std::max(myPerpCritsSorted.front().second,
        //            inputPerpCritsSorted.front().second);
        debug() << "BezierCurveQ::shiftAgainst - testing endpoint - shifted "
                   "has infinite intersection."
                << " start RoC: " << input.rateOfChangeAtParam(0)
                << " end RoC: "
                << input.rateOfChangeAtParam(1)
                //<< " overlapSize: " << std::to_string(overlapSize)
                << std::endl;
        if (!sufficientlyClose(myPerpCritsSorted.back().second,
                               inputPerpCritsSorted.front().second) &&
            !sufficientlyClose(myPerpCritsSorted.front().second,
                               inputPerpCritsSorted.back().second))
        // if (overlapSize > 0 && !sufficientlySmall(overlapSize))
        {
          output.distance = distanceBetween;
          output.param = currentParams.first;
          output.inputParam = currentParams.second;
          infiniteIntersectionBlockedInterval(
              input, slope, right, up, output.blockedCWVerticalAngleStart,
              output.blockedCWVerticalAngleEnd);
          {
            const CritsAndValues<3> myPerpCrits(
                getPerpendicularMagnitudeCritsAndValues(slope));
            typename StaticVector<std::pair<RealNum, RealNum>,
                                  3>::const_iterator myCritPosition,
                myOtherCritPosition1, myOtherCritPosition2;
            const bool atMyCrit = getCritPosition(
                currentParams.first, PerpMagCritCalculator(*this, slope),
                myPerpCrits, myCritPosition, myOtherCritPosition1,
                myOtherCritPosition2);
            output.atMyCrit = atRealCrit(myCritPosition, atMyCrit, myPerpCrits);
          }
          output.atInputCrit = output.atMyCrit;
        }
        continue;
      }
    }
  }
  { // Straight line conditions.
    const bool myStraightLine = straightLine();
    const bool inputStraightLine = input.straightLine();
    if (myStraightLine &&
        inputStraightLine) { // One straight line shifting against another will
                             // be limited by an endpoint (covered above).
      return;
    }
    for (int i = 0; i < 2; i++) {
      if ((i == 0 && !myStraightLine) || (i == 1 && !inputStraightLine)) {
        continue;
      }
      const StaticVector<std::pair<RealNum, RealNum>, 3> *const perpCrits =
          i == 0 ? &myPerpCritsSorted : &inputPerpCritsSorted;
      if (sufficientlyClose(perpCrits->front().second,
                            perpCrits->back().second)) {
        /*
        One of the curves is a straight line with slope equal to the input - it
        can only be blocked by an endpoint (covered above).
        */
        return;
      }
    }
  }
  { // Slope between matches input slope, same curve slope conditions.
    bool solutionsAreInputParams;
    PolynomialFunction<5> shiftRelation({});
    PolynomialFunction<2> shiftSubValueNumerator({});
    PolynomialFunction<2> shiftSubValueDenominator({});
    {
      const bool swapSlope = std::abs(slope) > 1;
      const RealNum relationSlope = swapSlope ? ((1.0) / slope) : slope;
      const PolynomialFunction<3> *const myX =
          swapSlope ? &(getParaFormY()) : &(getParaFormX());
      const PolynomialFunction<3> *const myY =
          swapSlope ? &(getParaFormX()) : &(getParaFormY());
      const PolynomialFunction<3> *const inputX =
          swapSlope ? &(input.getParaFormY()) : &(input.getParaFormX());
      const PolynomialFunction<3> *const inputY =
          swapSlope ? &(input.getParaFormX()) : &(input.getParaFormY());
      PolynomialFunction<5> shiftRelation1(getShiftRelationForSlopePF(
          *myX, *myY, *inputX, *inputY, relationSlope));
      PolynomialFunction<2> shiftSubValueNumerator1(
          shiftSubValueNumeratorPF(*myX, *myY, *inputX, *inputY));
      PolynomialFunction<2> shiftSubValueDenominator1(
          shiftSubValueDenominatorPF(*myX, *myY, *inputX, *inputY));
      PolynomialFunction<5> shiftRelation2(getShiftRelationForSlopePF(
          *inputX, *inputY, *myX, *myY, relationSlope));
      PolynomialFunction<2> shiftSubValueNumerator2(
          shiftSubValueNumeratorPF(*inputX, *inputY, *myX, *myY));
      PolynomialFunction<2> shiftSubValueDenominator2(
          shiftSubValueDenominatorPF(*inputX, *inputY, *myX, *myY));
      if (shiftRelation1.isConstant(0, 1)) {
        if (!shiftRelation2.isConstant(0, 1)) {
          solutionsAreInputParams = false;
          shiftRelation = shiftRelation2;
          shiftSubValueNumerator = shiftSubValueNumerator2;
          shiftSubValueDenominator = shiftSubValueDenominator2;
        } else {
          /*
          Infinite solutions indicates that the curves have the same shape,
          stick to endpoint results.
          */
          return;
        }
      } else if (!shiftRelation2.isConstant(0, 1) &&
                 ((shiftSubValueDenominator1.isConstant(0, 1) &&
                   sufficientlySmall(shiftSubValueDenominator1.valueAt(0))) ||
                  (!shiftSubValueDenominator1.isConstant(0, 1) &&
                   shiftSubValueDenominator1.getRoots(0, 1).size() > 0)) &&
                 ((!shiftSubValueDenominator2.isConstant(0, 1) ||
                   !sufficientlySmall(shiftSubValueDenominator2.valueAt(0))) &&
                  (shiftSubValueDenominator2.isConstant(0, 1) ||
                   shiftSubValueDenominator2.getRoots(0, 1).size() == 0))) {
        solutionsAreInputParams = false;
        shiftRelation = shiftRelation2;
        shiftSubValueNumerator = shiftSubValueNumerator2;
        shiftSubValueDenominator = shiftSubValueDenominator2;
      } else {
        solutionsAreInputParams = true;
        shiftRelation = shiftRelation1;
        shiftSubValueNumerator = shiftSubValueNumerator1;
        shiftSubValueDenominator = shiftSubValueDenominator1;
      }
    }
    const Point2D myStart(valueAt(0));
    const Point2D myEnd(valueAt(1));
    const Point2D inputStart(input.valueAt(0));
    const Point2D inputEnd(input.valueAt(1));
    for (const RealNum &currentShiftRelationRoot :
         shiftRelation.getRoots(0, 1)) {
      const RealNum subValue =
          shiftSubValueNumerator.valueAt(currentShiftRelationRoot) /
          shiftSubValueDenominator.valueAt(currentShiftRelationRoot);
      if (subValue < 0 ||
          subValue > 1) { // The shift relation root did not produce a valid
                          // parameter for both curves.
        continue;
      }
      const RealNum myCurrentParam =
          solutionsAreInputParams ? subValue : currentShiftRelationRoot;
      const RealNum inputCurrentParam =
          !solutionsAreInputParams ? subValue : currentShiftRelationRoot;
      const Point2D myCurrentPoint(valueAt(myCurrentParam));
      const Point2D inputCurrentPoint(input.valueAt(inputCurrentParam));
      if (myCurrentPoint == myStart || myCurrentPoint == myEnd ||
          inputCurrentPoint == inputStart ||
          inputCurrentPoint == inputEnd) { // Endpoint cases have been handled
                                           // in a previous step.
        continue;
      }
      const RealNum distanceBetween =
          myCurrentPoint.distanceFrom(inputCurrentPoint);
      if (sufficientlySmall(distanceBetween)) {
        if (std::find(myTestedPoints.begin(), myTestedPoints.end(),
                      myCurrentPoint) == myTestedPoints.end()) {
          myTestedPoints.push_back(myCurrentPoint);
          RealNum currentBlockedCWVerticalAngleStart,
              currentBlockedCWVerticalAngleEnd;
          bool currentAtMyCrit, currentAtInputCrit;
          if (curveIntersectionBlocksShift(
                  myCurrentParam, slope, right, up, inputCurrentParam, input,
                  true, currentBlockedCWVerticalAngleStart,
                  currentBlockedCWVerticalAngleEnd, currentAtMyCrit,
                  currentAtInputCrit)) {
            output.distance = 0;
            output.param = myCurrentParam;
            output.inputParam = inputCurrentParam;
            output.blockedCWVerticalAngleStart =
                currentBlockedCWVerticalAngleStart;
            output.blockedCWVerticalAngleEnd = currentBlockedCWVerticalAngleEnd;
            output.atMyCrit = currentAtMyCrit;
            output.atInputCrit = currentAtInputCrit;
            return;
          }
        }
        continue;
      }
      if (output.distance >= 0 && output.distance <= distanceBetween) {
        /*
        The distance between the two points is larger than the current smallest
        distance, meaning that it will be irrelevant.
        */
        continue;
      }
      if (myCurrentPoint.shift(distanceBetween, slope, right, up)
              .distanceFrom(inputCurrentPoint) > distanceBetween) {
        // The input point does not lie the in the shift direction from my
        // point.
        continue;
      }
      const RealNum inputRate = input.rateOfChangeAtParam(inputCurrentParam);

      debug() << "BezierCurveQ::shiftAgainst - testing overlaps for non-zero "
                 "distance,"
              << " sufficientlyCloseSlopes(inputRate, slope): "
              << sufficientlyCloseSlopes(inputRate, slope)
              << " myPerpCritsSorted.front().second: "
              << myPerpCritsSorted.front().second
              << " myPerpCritsSorted.back().second: "
              << myPerpCritsSorted.back().second
              << " inputPerpCritsSorted.front().second: "
              << inputPerpCritsSorted.front().second
              << " inputPerpCritsSorted.back().second: "
              << inputPerpCritsSorted.back().second << std::endl;

      if (sufficientlyCloseSlopes(inputRate, slope) &&
          (sufficientlyClose(myPerpCritsSorted.back().second,
                             inputPerpCritsSorted.front().second) ||
           sufficientlyClose(myPerpCritsSorted.front().second,
                             inputPerpCritsSorted.back().second))) {
        /*
        If one curve's minimum perpendicular magnitude is sufficiently close to
        the other's maximum or vice versa, there will be no significant overlap
        caused by a shift in this direction.
        */
        continue;
      }
      /*
      At this point, the distance between the test points is smaller than any
      current smallest distance, it is not a slope crit (equal to the input
      slope) for either curve, the curves are not touching (or almost touching),
      and the test points are not endpoints of either curve. Therefore, a block
      definitely occurs here.
      */
      debug() << "^^ Adding non-zero distance -"
              << " distanceBetween: " << std::to_string(distanceBetween)
              << " myCurrentParam: " << std::to_string(myCurrentParam)
              << " inputCurrentParam: " << std::to_string(inputCurrentParam)
              << " inputRate: " << std::to_string(inputRate) << " my RoC: "
              << std::to_string(rateOfChangeAtParam(myCurrentParam))
              << " slope: " << std::to_string(slope) << " CWV angle 1: "
              << std::to_string(_getCWVerticalAngle(inputRate, inputRate, right,
                                                    up, false, false))
              << " CWV angle 2: "
              << std::to_string(_getCWVerticalAngle(inputRate, inputRate,
                                                    !right, !up, false, false))
              << std::endl;
      {
        RealNum tempBlockedCWVerticalAngleStart, tempBlockedCWVerticalAngleEnd;
        bool tempAtMyCrit, tempAtInputCrit;
        if (shift(distanceBetween, slope, right, up)
                .curveIntersectionBlocksShift(myCurrentParam, slope, right, up,
                                              inputCurrentParam, input, true,
                                              tempBlockedCWVerticalAngleStart,
                                              tempBlockedCWVerticalAngleEnd,
                                              tempAtMyCrit, tempAtInputCrit)) {
          /*
          This is not considered a block under certain circumstances, hence the
          test in this if statement.  If the curves have a small overlap, they
          would have 2 intersection points; if the points that result in the two
          curves having the same slope AND a slope betwen them matching the
          input slope are between these 2 intersection points on their
          respective curves, this could return a false positive since the
          initial tests in this method already verified that the intersection
          overlaps are not significant enough to constitute a block.
          */
          output.distance = distanceBetween;
          output.param = myCurrentParam;
          output.inputParam = inputCurrentParam;
          output.blockedCWVerticalAngleStart = tempBlockedCWVerticalAngleStart;
          output.blockedCWVerticalAngleEnd = tempBlockedCWVerticalAngleEnd;
          output.atMyCrit = tempAtMyCrit;
          output.atInputCrit = tempAtInputCrit;
        }
      }
    }
  }
}

StaticVector<std::pair<RealNum, RealNum>, 4>
BezierCurveQ::pointsOfIntersection(const BezierCurveQ &input)
    const { // Returns a list containing NaN when there are an infinite number
  // of intersection points.
  debug() << "BezierCurveQ::pointsOfIntersection" << std::endl;
  if ((input.getMaxXExtent() < getMinXExtent() &&
       !sufficientlyClose(input.getMaxXExtent(), getMinXExtent())) ||
      (input.getMinXExtent() > getMaxXExtent() &&
       !sufficientlyClose(input.getMinXExtent(), getMaxXExtent())) ||
      (input.getMinYExtent() > getMaxYExtent() &&
       !sufficientlyClose(input.getMinYExtent(), getMaxYExtent())) ||
      (input.getMaxYExtent() < getMinYExtent() &&
       !sufficientlyClose(input.getMaxYExtent(), getMinYExtent()))) {
    return {};
  }
  StaticVector<std::pair<RealNum, RealNum>, 4> result;
  const Point2D myStart(valueAt(0));
  const Point2D myEnd(valueAt(1));
  const Point2D inputStart(input.valueAt(0));
  const Point2D inputEnd(input.valueAt(1));
  if (getControl() == input.getControl() && myStart == inputStart &&
      myEnd == inputEnd) {
    result = INFINITE_INTERSECTION;
    result.push_back({0, 0});
    result.push_back({1, 1});
  } else if (getControl() == input.getControl() && myStart == inputEnd &&
             myEnd == inputStart) {
    result = INFINITE_INTERSECTION;
    result.push_back({0, 1});
    result.push_back({1, 0});
  } else if (straightLine() && input.straightLine()) {
    struct LocalFunctions {
      static bool straightLinePerpMagTouch(const Point2D &lineTestPoint,
                                           const RealNum &lineSlope,
                                           const Point2D &inputStart,
                                           const Point2D &inputEnd) {
        const RealNum lineTestPointPerpMag =
            Point2D::getPerpendicularMagnitude(lineTestPoint, lineSlope);
        const RealNum inputStartPerpMag =
            Point2D::getPerpendicularMagnitude(inputStart, lineSlope);
        const RealNum inputEndPerpMag =
            Point2D::getPerpendicularMagnitude(inputEnd, lineSlope);
        return sufficientlyClose(lineTestPointPerpMag, inputStartPerpMag) ||
               sufficientlyClose(lineTestPointPerpMag, inputEndPerpMag) ||
               (inputStartPerpMag > lineTestPointPerpMag) !=
                   (inputEndPerpMag > lineTestPointPerpMag);
      }
    };
    const RealNum mySlope = rateOfChangeAtParam(0.5);
    const RealNum inputSlope = input.rateOfChangeAtParam(0.5);
    const Point2D myStart(valueAt(0));
    const Point2D inputStart(input.valueAt(0));
    const Point2D myEnd(valueAt(1));
    const Point2D inputEnd(input.valueAt(1));
    if (sufficientlyCloseSlopes(mySlope, inputSlope) && [&]() -> bool {
          const RealNum myStartPerpMag =
              Point2D::getPerpendicularMagnitude(myStart, mySlope);
          const RealNum inputStartPerpMag =
              Point2D::getPerpendicularMagnitude(inputStart, inputSlope);
          return sufficientlyClose(
                     myStartPerpMag,
                     Point2D::getPerpendicularMagnitude(inputStart, mySlope)) &&
                 sufficientlyClose(
                     myStartPerpMag,
                     Point2D::getPerpendicularMagnitude(inputEnd, mySlope)) &&
                 sufficientlyClose(
                     inputStartPerpMag,
                     Point2D::getPerpendicularMagnitude(myStart, inputSlope)) &&
                 sufficientlyClose(
                     inputStartPerpMag,
                     Point2D::getPerpendicularMagnitude(myEnd, inputSlope));
        }()) {
      const Point2D *testPoints[]{&myStart, &myEnd, &inputStart, &inputEnd};
      const int arrayLength = sizeof(testPoints) / sizeof(testPoints[0]);
      std::sort(
          testPoints, testPoints + arrayLength,
          std::abs(mySlope) > 1
          ? [](const Point2D *input1, const Point2D *input2)
                -> bool { return input1->getY() < input2->getY(); }
          : [](const Point2D *input1, const Point2D *input2) -> bool {
              return input1->getX() < input2->getX();
            });
      if ((testPoints[0] == &myStart || testPoints[0] == &myEnd) ==
          (testPoints[1] == &myStart ||
           testPoints[1] == &myEnd)) { // Curves do not overlap.
        if (*(testPoints[1]) ==
            *(testPoints[2])) { // Curves touch at an endpoint.
          result = {
              {testPoints[1] == &myStart || testPoints[2] == &myStart ? 0.0
                                                                      : 1.0,
               testPoints[1] == &inputStart || testPoints[2] == &inputStart
                   ? 0.0
                   : 1.0}};
        }
        // Otherwise no touch.
      } else if ((testPoints[1] == &myStart || testPoints[1] == &myEnd) ==
                 (testPoints[2] == &myStart ||
                  testPoints[2] ==
                      &myEnd)) { // One curve entirely within another.
        result = INFINITE_INTERSECTION;
        if (testPoints[1] == &myStart ||
            testPoints[1] ==
                &myEnd) { // This curve is entirely within the input.
          result.push_back(
              {0, input.straightLineParamForPoint(inputSlope, myStart)});
          result.push_back(
              {1, input.straightLineParamForPoint(inputSlope, myEnd)});
        } else { // The input curve is entirely within this curve.
          const RealNum myParam1 =
              straightLineParamForPoint(mySlope, inputStart);
          const RealNum myParam2 = straightLineParamForPoint(mySlope, inputEnd);
          if (myParam1 < myParam2) {
            result.push_back({myParam1, 0});
            result.push_back({myParam2, 1});
          } else {
            result.push_back({myParam2, 1});
            result.push_back({myParam1, 0});
          }
        }
      } else if (*(testPoints[1]) ==
                 *(testPoints[2])) { // Curves partially overlap, but the
                                     // overlap length is sufficiently small.
        result = {
            {(testPoints[0] == &myEnd || testPoints[3] == &myEnd) ? 0.0 : 1.0,
             (testPoints[0] == &inputEnd || testPoints[3] == &inputEnd) ? 0.0
                                                                        : 1.0}};
      } else { // Curves partially overlap one another.
        result = INFINITE_INTERSECTION;
        if (testPoints[0] == &myStart) {
          result.push_back(
              {straightLineParamForPoint(mySlope, *(testPoints[1])),
               testPoints[1] == &inputStart ? 0.0 : 1.0});
          result.push_back({1, input.straightLineParamForPoint(
                                   inputSlope, *(testPoints[2]))});
        } else if (testPoints[0] == &myEnd) {
          result.push_back({0, input.straightLineParamForPoint(
                                   inputSlope, *(testPoints[2]))});
          result.push_back(
              {straightLineParamForPoint(mySlope, *(testPoints[1])),
               testPoints[1] == &inputStart ? 0.0 : 1.0});
        } else if (testPoints[0] == &inputStart) {
          if (testPoints[1] == &myStart) {
            result.push_back(
                {0, input.straightLineParamForPoint(inputSlope, myStart)});
            result.push_back({straightLineParamForPoint(mySlope, inputEnd), 1});
          } else {
            result.push_back({straightLineParamForPoint(mySlope, inputEnd), 1});
            result.push_back(
                {1, input.straightLineParamForPoint(inputSlope, myEnd)});
          }
        } else { // First entry is the input curve's end.
          if (testPoints[1] == &myStart) {
            result.push_back(
                {0, input.straightLineParamForPoint(inputSlope, myStart)});
            result.push_back(
                {straightLineParamForPoint(mySlope, inputStart), 0});
          } else {
            result.push_back(
                {straightLineParamForPoint(mySlope, inputStart), 0});
            result.push_back(
                {1, input.straightLineParamForPoint(inputSlope, myEnd)});
          }
        }
      }
    } else if (LocalFunctions::straightLinePerpMagTouch(myStart, mySlope,
                                                        inputStart, inputEnd) &&
               LocalFunctions::straightLinePerpMagTouch(
                   inputStart, inputSlope, myStart,
                   myEnd)) { // These straight lines are not parallel, though
                             // they may have very close slopes.
      /*
       * -my start: (a, b)
       * -input start: (c, d)
       * -my slope: e
       * -input slope: f
       * -intersection point: (g, h)
       *
       * (b-h)/(a-g)=e => b-h=e(a-g) => h=b-e(a-g)
       * (d-h)/(c-g)=f => d-h=f(c-g) => h=d-f(c-g)
       *
       * b-e(a-g)=d-f(c-g) => b-ea+eg-d+fc-fg=0 => g=(ea+d-fc-b)/(e-f)
       * */
      RealNum intersectionXCoord;
      if (std::abs(mySlope) <= 1 && std::abs(inputSlope) <= 1) {
        intersectionXCoord =
            ((mySlope * myStart.getX()) - (inputSlope * inputStart.getX()) -
             myStart.getY() + inputStart.getY()) /
            (mySlope - inputSlope);
      } else if (std::abs(mySlope) > std::abs(inputSlope)) {
        const RealNum slopeQuotient =
            inputSlope / mySlope; // -1 < slopeQuotient < 1
        intersectionXCoord =
            (myStart.getX() - (slopeQuotient * inputStart.getX()) +
             ((inputStart.getY() - myStart.getY()) / mySlope)) /
            (1.0 - slopeQuotient);
      } else {
        const RealNum slopeQuotient =
            mySlope / inputSlope; // -1 < slopeQuotient < 1
        intersectionXCoord =
            ((slopeQuotient * myStart.getX()) - inputStart.getX() +
             ((inputStart.getY() - myStart.getY()) / inputSlope)) /
            (slopeQuotient - 1.0);
      }
      const Point2D intersectionPoint(
          intersectionXCoord,
          std::abs(mySlope) < std::abs(inputSlope)
              ? myStart.getY() +
                    ((intersectionXCoord - myStart.getX()) * mySlope)
              : inputStart.getY() +
                    ((intersectionXCoord - inputStart.getX()) * inputSlope));
      const RealNum myParam =
          straightLineParamForPoint(mySlope, intersectionPoint);
      const RealNum inputParam =
          input.straightLineParamForPoint(inputSlope, intersectionPoint);
      if (!std::isnan(myParam) && !std::isnan(inputParam) &&
          valueAt(myParam) == input.valueAt(inputParam)) {
        result.push_back({myParam, inputParam});
      }
    }
  } else { // At least one of this and the input curve are not straight lines.
    const StaticVector<std::pair<RealNum, RealNum>, 4> systemSolution(
        solveSystem(getParaFormX(), input.getParaFormX(), getParaFormY(),
                    input.getParaFormY(), 0, 1, 0, 1));
    debug() << "system solution: " << systemSolution << std::endl;
    if (isSolutionInfinite(systemSolution)) {
      if (sufficientlyClose(getMaxXExtent(), input.getMinXExtent())) {
        result = {{getMaxXPara(), input.getMinXPara()}};
      } else if (sufficientlyClose(getMinXExtent(), input.getMaxXExtent())) {
        result = {{getMinXPara(), input.getMaxXPara()}};
      } else if (sufficientlyClose(getMaxYExtent(), input.getMinYExtent())) {
        result = {{getMaxYPara(), input.getMinYPara()}};
      } else if (sufficientlyClose(getMinYExtent(), input.getMaxYExtent())) {
        result = {{getMinYPara(), input.getMaxYPara()}};
      } else if (getMaxXExtent() > input.getMinXExtent() &&
                 getMinXExtent() < input.getMaxXExtent() &&
                 getMaxYExtent() > input.getMinYExtent() &&
                 getMinYExtent() < input.getMaxYExtent()) {
        result = INFINITE_INTERSECTION;
        result.push_back(*(systemSolution.begin() + 1));
        result.push_back(*(systemSolution.begin() + 2));
      }
    } else {
      result = systemSolution;
    }
  }
  return result;
}

const Point2D &BezierCurveQ::getControl() const { return control; }

Point2D BezierCurveQ::valueAt(const RealNum &parameter) const {
  checkParam(parameter);
  return curveValueAt(parameter, paraFormX, paraFormY);
}

const PolynomialFunction<3> &BezierCurveQ::getParaFormX() const {
  return paraFormX;
}

const PolynomialFunction<3> &BezierCurveQ::getParaFormY() const {
  return paraFormY;
}

std::string BezierCurveQ::toString() const {
  return valueAt(0).toString() + "-" + getControl().toString() + "-" +
         valueAt(1).toString();
}

BezierCurveQ BezierCurveQ::longStraightLine(const RealNum &slope,
                                            const Point2D &point) {
  return BezierCurveQ(point.shift(25000, slope, false, false),
                      point.shift(25000, slope, true, true), point);
}

/*
Returns a long straight line that starts at a given point and passes through
another given point.
*/
BezierCurveQ BezierCurveQ::longStraightLine(const Point2D &start,
                                            const Point2D &point) {
  const RealNum slope(Point2D::getSlopeBetween(start, point));
  return straightLine(start,
                      start.shift(25000, slope, point.getX() >= start.getX(),
                                  point.getY() >= start.getY()));
}

BezierCurveQ BezierCurveQ::straightLine(const Point2D &s, const Point2D &e) {
  return BezierCurveQ(
      s, e, Point2D((s.getX() + e.getX()) / 2.0, (s.getY() + e.getY()) / 2.0));
}

bool BezierCurveQ::isIntersectionInfinite(
    const StaticVector<std::pair<RealNum, RealNum>, 4> &intersection) {
  return intersection.size() == 3 && std::isnan(intersection.front().first) &&
         std::isnan(intersection.front().second);
}

RealNum BezierCurveQ::minMaxDistance(const Point2D &input, bool max,
                                     const RealNum &minParam,
                                     const RealNum &maxParam) const {
  const CritsAndValues<5> distanceCrits(getDistanceCritsAndValues(input));
  RealNum result;
  RealNum resultDistance;
  {
    RealNum minParamDistance = valueAt(minParam).distanceFrom(input);
    RealNum maxParamDistance = valueAt(maxParam).distanceFrom(input);
    if (minParamDistance > maxParamDistance) {
      result = max ? minParam : maxParam;
      resultDistance = max ? minParamDistance : maxParamDistance;
    } else {
      result = max ? maxParam : minParam;
      resultDistance = max ? maxParamDistance : minParamDistance;
    }
  }
  for (const std::pair<RealNum, RealNum> &currentCrit :
       distanceCrits.critsAndValues) {
    if (currentCrit.first < minParam) {
      continue;
    }
    if (currentCrit.first > maxParam) {
      break;
    }
    if ((max && resultDistance < currentCrit.second) ||
        (!max && resultDistance > currentCrit.second)) {
      result = currentCrit.first;
      resultDistance = currentCrit.second;
    }
  }
  return result;
}

void BezierCurveQ::checkParam(const RealNum &parameter) {
  if (parameter > 1 || parameter < 0) {
    throw std::string("Invalid bezier parameter: ") + std::to_string(parameter);
  }
}

void BezierCurveQ::endpointRotationConditions(
    const CritsAndValues<5> &myDistanceCrits, const BezierCurveQ &input,
    const CritsAndValues<5> &inputDistanceCrits, const Point2D &fulcrum,
    bool clockwise, BezierCurveQ::RotateAgainstResult &output) const {
  struct TestParamPair {
    RealNum myParam;
    RealNum inputParam;
  };
  struct LocalFunctions {
    static StaticVector<RealNum, 4>
    getTestParams(const RealNum &targetDistance,
                  const bool increasingFromTargetDistance,
                  const BezierCurveQ &matchingCurve,
                  const CritsAndValues<5> &matchingCurveDistanceCrits,
                  const Point2D &fulcrum) {
      StaticVector<RealNum, 4> result;
      if (sufficientlySmall(targetDistance)) {
        return result;
      }
      debug() << "BezierCurveQ::endpointRotationConditions - getTestParams,"
              << " targetDistance: " << std::to_string(targetDistance)
              << " matchingCurveDistanceCrits: "
              << matchingCurveDistanceCrits.critsAndValues << std::endl;
      for (const RealNum &currentMatchingParam :
           matchingCurve.getCurveDistanceParams(fulcrum, targetDistance, 0,
                                                1)) {
        debug() << "BezierCurveQ::endpointRotationConditions - getTestParams,"
                << " currentMatchingParam: "
                << std::to_string(currentMatchingParam) << std::endl;
        const auto matchingCrit = [&]()
            -> StaticVector<std::pair<RealNum, RealNum>, 5>::const_iterator {
          StaticVector<std::pair<RealNum, RealNum>, 5>::const_iterator
              outputVal = matchingCurveDistanceCrits.critsAndValues.end();
          RealNum smallestDifference = std::numeric_limits<RealNum>::infinity();
          for (StaticVector<std::pair<RealNum, RealNum>, 5>::const_iterator i =
                   matchingCurveDistanceCrits.critsAndValues.begin();
               i != matchingCurveDistanceCrits.critsAndValues.end(); i++) {
            if (sufficientlyClose(currentMatchingParam, i->first) &&
                matchingCurve.sufficientlyCloseAlongCurve(currentMatchingParam,
                                                          i->first) &&
                std::abs(currentMatchingParam - (i->first)) <
                    smallestDifference) {
              outputVal = i;
              smallestDifference = std::abs(currentMatchingParam - (i->first));
            }
          }
          return outputVal;
        }();
        if (matchingCrit == matchingCurveDistanceCrits.critsAndValues.end() ||
            (increasingFromTargetDistance ==
             (matchingCrit->second <
              (matchingCrit == matchingCurveDistanceCrits.critsAndValues.begin()
                   ? (matchingCrit + 1)
                   : (matchingCrit - 1))
                  ->second))) {
          result.push_back(currentMatchingParam);
        }
      }
      return result;
    }
  };
  output.param = -1;
  output.inputParam = -1;
  output.angle = -1;
  output.blockedCWVerticalAngleStart = -1;
  output.blockedCWVerticalAngleEnd = -1;
  output.infiniteInitialIntersection = false;
  output.atMyCrit = false;
  output.atInputCrit = false;
  debug() << "BezierCurveQ::endpointRotationConditions -"
          << " myDistanceCrits: " << myDistanceCrits.critsAndValues
          << " inputDistanceCrits: " << inputDistanceCrits.critsAndValues
          << std::endl;
  StaticVector<TestParamPair, 16> testParams;
  for (const RealNum &current : LocalFunctions::getTestParams(
           myDistanceCrits.critsAndValues.begin()->second,
           myDistanceCrits.critsAndValues.begin()->second <
               (myDistanceCrits.critsAndValues.begin() + 1)->second,
           input, inputDistanceCrits, fulcrum)) {
    testParams.push_back({0, current});
  }
  for (const RealNum &current : LocalFunctions::getTestParams(
           (myDistanceCrits.critsAndValues.end() - 1)->second,
           (myDistanceCrits.critsAndValues.end() - 1)->second <
               (myDistanceCrits.critsAndValues.end() - 2)->second,
           input, inputDistanceCrits, fulcrum)) {
    testParams.push_back({1, current});
  }
  for (const RealNum &current : LocalFunctions::getTestParams(
           inputDistanceCrits.critsAndValues.begin()->second,
           inputDistanceCrits.critsAndValues.begin()->second <
               (inputDistanceCrits.critsAndValues.begin() + 1)->second,
           *this, myDistanceCrits, fulcrum)) {
    testParams.push_back({current, 0});
  }
  for (const RealNum &current : LocalFunctions::getTestParams(
           (inputDistanceCrits.critsAndValues.end() - 1)->second,
           (inputDistanceCrits.critsAndValues.end() - 1)->second <
               (inputDistanceCrits.critsAndValues.end() - 2)->second,
           *this, myDistanceCrits, fulcrum)) {
    testParams.push_back({current, 1});
  }
  for (const TestParamPair &currentParams : testParams) {
    const Point2D myPoint(valueAt(currentParams.myParam));
    const RealNum angleBetween = Point2D::getAngleBetween(
        myPoint, input.valueAt(currentParams.inputParam), fulcrum, clockwise);
    debug()
        << "BezierCurveQ::endpointRotationConditions - current param pair,"
        << " myParam: " << std::to_string(currentParams.myParam)
        << " my point: " << valueAt(currentParams.myParam)
        << " my point distance: "
        << std::to_string(valueAt(currentParams.myParam).distanceFrom(fulcrum))
        << " inputParam: " << std::to_string(currentParams.inputParam)
        << " input point: " << input.valueAt(currentParams.inputParam)
        << " input point distance: "
        << std::to_string(
               input.valueAt(currentParams.inputParam).distanceFrom(fulcrum))
        << " angleBetween: " << std::to_string(angleBetween) << " me rotated: "
        << rotate(fulcrum, angleBetween * (clockwise ? (-1.0) : 1.0))
        << std::endl;
    if ((output.angle < 0 || output.angle > angleBetween)) {
      const BezierCurveQ meRotated(
          rotate(fulcrum, angleBetween * (clockwise ? (-1.0) : 1.0)));
      RealNum blockedCWVerticalAngleStart, blockedCWVerticalAngleEnd;
      bool atMyCrit, atInputCrit;
      if (meRotated.curveIntersectionBlocksRotate(
              currentParams.myParam, myDistanceCrits, input,
              currentParams.inputParam, inputDistanceCrits, fulcrum, clockwise,
              blockedCWVerticalAngleStart, blockedCWVerticalAngleEnd, atMyCrit,
              atInputCrit)) {
        debug() << "BezierCurveQ::endpointRotationConditions - found block,"
                << " angleBetween: " << std::to_string(angleBetween)
                << " blockedCWVerticalAngleStart: "
                << std::to_string(blockedCWVerticalAngleStart)
                << " blockedCWVerticalAngleEnd: "
                << std::to_string(blockedCWVerticalAngleEnd) << std::endl;
        output.param = currentParams.myParam;
        output.inputParam = currentParams.inputParam;
        output.angle = angleBetween;
        output.blockedCWVerticalAngleStart = blockedCWVerticalAngleStart;
        output.blockedCWVerticalAngleEnd = blockedCWVerticalAngleEnd;
        output.atMyCrit = atMyCrit;
        output.atInputCrit = atInputCrit;
      } else if (meRotated.getConcavityIndicator(currentParams.myParam) ==
                     input.getConcavityIndicator(currentParams.inputParam) &&
                 sufficientlyCloseSlopes(
                     meRotated.getDistanceConcavity(
                         currentParams.myParam,
                         meRotated.valueAt(currentParams.myParam)
                             .shift(100,
                                    (-1.0) / meRotated.rateOfChangeAtParam(
                                                 currentParams.myParam),
                                    true, true)),
                     input.getDistanceConcavity(
                         currentParams.inputParam,
                         input.valueAt(currentParams.inputParam)
                             .shift(100,
                                    (-1.0) / input.rateOfChangeAtParam(
                                                 currentParams.inputParam),
                                    true, true))) &&
                 isIntersectionInfinite(
                     meRotated.pointsOfIntersection(input)) &&
                 !isIntersectionInfinite(pointsOfIntersection(input))) {
        debug() << "BezierCurveQ::endpointRotationConditions - infinite "
                   "rotated intersection."
                << std::endl;
        output.param = currentParams.myParam;
        output.inputParam = currentParams.inputParam;
        output.angle = angleBetween;
        {
          const Point2D myCurrentPointRotated(
              meRotated.valueAt(currentParams.myParam));
          infiniteIntersectionBlockedInterval(
              input,
              (-1.0) / Point2D::getSlopeBetween(fulcrum, myCurrentPointRotated),
              clockwise == (myCurrentPointRotated.getY() > fulcrum.getY()),
              clockwise == (myCurrentPointRotated.getX() < fulcrum.getX()),
              output.blockedCWVerticalAngleStart,
              output.blockedCWVerticalAngleEnd);
        }
        {
          StaticVector<std::pair<RealNum, RealNum>, 5>::const_iterator
              myCritPosition,
              myOtherCritPosition1, myOtherCritPosition2;
          const bool atMyCrit = getCritPosition(
              currentParams.myParam, DistanceCritCalculator(*this, fulcrum),
              myDistanceCrits, myCritPosition, myOtherCritPosition1,
              myOtherCritPosition2);
          output.atMyCrit =
              atRealCrit(myCritPosition, atMyCrit, myDistanceCrits);
        }
        {
          StaticVector<std::pair<RealNum, RealNum>, 5>::const_iterator
              inputCritPosition,
              inputOtherCritPosition1, inputOtherCritPosition2;
          const bool atinputCrit = getCritPosition(
              currentParams.inputParam, DistanceCritCalculator(input, fulcrum),
              inputDistanceCrits, inputCritPosition, inputOtherCritPosition1,
              inputOtherCritPosition2);
          output.atInputCrit =
              atRealCrit(inputCritPosition, atinputCrit, inputDistanceCrits);
        }
      } else {
        debug() << "BezierCurveQ::endpointRotationConditions - no block."
                << std::endl;
      }
    }
  }
}

/*
Returns the smallest angle of rotation about the fulcrum point that causes this
curve to touch the input curve at one or an infinite number of points.

Special rules apply when the curves intersect/overlap at the fulcrum, resulting
in negative intervals (indicating invalid).
*/
void BezierCurveQ::rotateAgainst(
    const BezierCurveQ &input, const Point2D &fulcrum, bool clockwise,
    BezierCurveQ::RotateAgainstResult &output) const {
  struct LocalFunctions {
    static bool testSolution(const BezierCurveQ &moving,
                             const CritsAndValues<5> &movingDistanceCrits,
                             const BezierCurveQ &stationary,
                             const CritsAndValues<5> &stationaryDistanceCrits,
                             const Point2D &fulcrum, const bool clockwise,
                             const RealNum &angle, const RealNum &movingParam,
                             const RealNum &stationaryParam) {
      const BezierCurveQ movingRotated(
          moving.rotate(fulcrum, angle * (clockwise ? (-1.0) : 1.0)));
      const Point2D movingRotatedStart(movingRotated.valueAt(0));
      const Point2D movingRotatedEnd(movingRotated.valueAt(1));
      const Point2D stationaryStart(stationary.valueAt(0));
      const Point2D stationaryEnd(stationary.valueAt(1));
      const Point2D movingRotatedTestPoint(movingRotated.valueAt(movingParam));
      const Point2D stationaryTestPoint(stationary.valueAt(stationaryParam));
      const StaticVector<std::pair<RealNum, RealNum>, 4> intersection(
          movingRotated.pointsOfIntersection(stationary));
      for (const std::pair<RealNum, RealNum> &currentIntersection :
           intersection) {
        const Point2D currentMovingPoint(
            movingRotated.valueAt(currentIntersection.first));
        const Point2D currentStationaryPoint(
            stationary.valueAt(currentIntersection.second));
        RealNum placeholder1, placeholder2;
        bool placeholder3, placeholder4;
        if (currentMovingPoint != movingRotatedStart &&
            currentMovingPoint != movingRotatedEnd &&
            currentStationaryPoint != stationaryStart &&
            currentStationaryPoint != stationaryEnd &&
            currentMovingPoint != movingRotatedTestPoint &&
            currentStationaryPoint != stationaryTestPoint &&
            movingRotated.curveIntersectionBlocksRotate(
                currentIntersection.first, movingDistanceCrits, stationary,
                currentIntersection.second, stationaryDistanceCrits, fulcrum,
                !clockwise, placeholder1, placeholder2, placeholder3,
                placeholder4)) {
          return false;
        }
      }
      return true;
    }

    static void
    populateBlockedAngleIntervalsForInfiniteIntersectionAtNonFulcrumCrit(
        const BezierCurveQ &curve, const RealNum &curveParam,
        const Point2D &fulcrum, const bool clockwise,
        RealNum &outputBlockedCWVerticalAngleStart,
        RealNum &outputBlockedCWVerticalAngleEnd) {
      const Point2D curvePoint(curve.valueAt(curveParam));
      const RealNum perpSlope =
          (-1.0) / Point2D::getSlopeBetween(curvePoint, fulcrum);
      RealNum straightLineSlope;
      if (!curve.straightLine(straightLineSlope) ||
          !sufficientlyCloseSlopes(straightLineSlope, perpSlope)) {
        /*
        The straightLine result for both curves is the same if they have
        infinite instersection.
        */
        curve.populateInfiniteIntersectionShiftBlockValues(
            perpSlope, clockwise == (curvePoint.getY() > fulcrum.getY()),
            clockwise == (curvePoint.getX() < fulcrum.getX()),
            outputBlockedCWVerticalAngleStart, outputBlockedCWVerticalAngleEnd);
      } else {
        /*
        This and the input curve are both straight lines that are perpendicular
        to the slope formed between the blocking point and the rotation fulcrum.
        The blocked intervals cannot be determined here since it could arguably
        block the interval along the curves' slope away from the fulcrum or
        towards it.
        */
        outputBlockedCWVerticalAngleStart = -1;
        outputBlockedCWVerticalAngleEnd = -1;
      }
    }
  };
  /************************************************************************************************************************/
  output.angle = -1.0;
  output.param = -1.0;
  output.inputParam = -1.0;
  output.blockedCWVerticalAngleStart = -1;
  output.blockedCWVerticalAngleEnd = -1;
  output.infiniteInitialIntersection = false;
  output.atMyCrit = false;
  output.atInputCrit = false;
  const Point2D myStart(valueAt(0));
  const Point2D myEnd(valueAt(1));
  const Point2D inputStart(input.valueAt(0));
  const Point2D inputEnd(input.valueAt(1));
  const CritsAndValues<5> myDistanceCrits(getDistanceCritsAndValues(fulcrum));
  const CritsAndValues<5> inputDistanceCrits(
      input.getDistanceCritsAndValues(fulcrum));
  { // Intersection conditions.
    const StaticVector<std::pair<RealNum, RealNum>, 4> myIntersections(
        pointsOfIntersection(input));
    {
      const auto printDistanceCrits =
          [](const CritsAndValues<5> &input) -> std::string {
        std::string result;
        for (const auto &current : input.critsAndValues) {
          result += result.length() > 0 ? ", " : "";
          result += "(" + std::to_string(current.first) + ", " +
                    std::to_string(current.second) + ")";
        }
        return result;
      };
      debug() << "BezierCurveQ::rotateAgainst -"
              << " my intersections: " << myIntersections
              << " myDistanceCrits: " << printDistanceCrits(myDistanceCrits)
              << " inputDistanceCrits: "
              << printDistanceCrits(inputDistanceCrits) << std::endl;
    }
    if (isIntersectionInfinite(myIntersections)) {
      output.infiniteInitialIntersection = true;
      {
        struct DistanceCritPair {
          RealNum myParam;
          StaticVector<std::pair<RealNum, RealNum>, 5> inputParams;
        };
        StaticVector<DistanceCritPair, 5> matchingDistanceCrits;
        for (const std::pair<RealNum, RealNum> &myCurrentCrit :
             myDistanceCrits.critsAndValues) {
          if (&myCurrentCrit == &(myDistanceCrits.critsAndValues.front()) ||
              &myCurrentCrit == &(myDistanceCrits.critsAndValues.back()) ||
              (myCurrentCrit.first < myIntersections[1].first &&
               !sufficientlyCloseAlongCurve(myCurrentCrit.first,
                                            myIntersections[1].first)) ||
              (myCurrentCrit.first > myIntersections[2].first &&
               !sufficientlyCloseAlongCurve(myCurrentCrit.first,
                                            myIntersections[2].first))) {
            continue;
          }
          const Point2D myCurrentPoint(valueAt(myCurrentCrit.first));
          matchingDistanceCrits.push_back(
              DistanceCritPair({myCurrentCrit.first, {}}));
          for (const std::pair<RealNum, RealNum> &inputCurrentCrit :
               inputDistanceCrits.critsAndValues) {
            const RealNum inputLowerIntersectionParam =
                std::min(myIntersections[1].second, myIntersections[2].second);
            const RealNum inputHigherIntersectionParam =
                std::max(myIntersections[1].second, myIntersections[2].second);
            if (&inputCurrentCrit ==
                    &(inputDistanceCrits.critsAndValues.front()) ||
                &inputCurrentCrit ==
                    &(inputDistanceCrits.critsAndValues.back()) ||
                (inputCurrentCrit.first < inputLowerIntersectionParam &&
                 !input.sufficientlyCloseAlongCurve(
                     inputCurrentCrit.first, inputLowerIntersectionParam)) ||
                (inputCurrentCrit.first > inputHigherIntersectionParam &&
                 !input.sufficientlyCloseAlongCurve(
                     inputCurrentCrit.first, inputHigherIntersectionParam))) {
              continue;
            }
            matchingDistanceCrits.back().inputParams.push_back(
                {inputCurrentCrit.first, input.valueAt(inputCurrentCrit.first)
                                             .distanceFrom(myCurrentPoint)});
          }
        }
        bool infiniteIntersectionAtFulcrum = false;
        for (const DistanceCritPair &currentCritPair : matchingDistanceCrits) {
          if (currentCritPair.inputParams.empty()) {
            continue;
          }
          /*
           * Testing crit point equivalence here is not sufficient since certain
           * curves (i.e. straight lines) have some latitude in determining if
           * they infinitely intersect.  If two straight lines are very close to
           * each other, they will report an infinite intersection, but their
           * associated points will not quite be equivalent.
           *
           * This approach finds the closest crit in the input curve to my
           * current crit, after filtering out the points on each curve that are
           * not considered common/overlapping between the two curves.
           * */
          const RealNum myCurrentCrit = currentCritPair.myParam;
          const RealNum inputCurrentCrit =
              std::minmax_element(currentCritPair.inputParams.begin(),
                                  currentCritPair.inputParams.end(),
                                  compareSecond)
                  .first->first;
          const Point2D myCurrentPoint(valueAt(myCurrentCrit));
          const Point2D inputCurrentPoint(input.valueAt(inputCurrentCrit));
          if (myCurrentPoint == fulcrum || inputCurrentPoint == fulcrum) {
            static const auto updateRotationAngle =
                [](const Point2D &movingPoint, const Point2D &stationaryPoint,
                   const Point2D &fulcrum, const bool clockwise,
                   RealNum &currentAngleBetween) -> void {
              const RealNum angleBetween = Point2D::getAngleBetween(
                  movingPoint, stationaryPoint, fulcrum, clockwise);
              if (!sufficientlyClose(angleBetween, 0) &&
                  !sufficientlyClose(angleBetween, 360)) {
                currentAngleBetween =
                    std::min(currentAngleBetween, angleBetween);
              }
            };
            RealNum rotationAngle = std::numeric_limits<RealNum>::infinity();
            const bool useInputStart = inputStart != fulcrum;
            const bool useInputEnd = inputEnd != fulcrum;
            if (myStart != fulcrum) {
              if (useInputStart) {
                updateRotationAngle(myStart, inputStart, fulcrum, clockwise,
                                    rotationAngle);
              }
              if (useInputEnd) {
                updateRotationAngle(myStart, inputEnd, fulcrum, clockwise,
                                    rotationAngle);
              }
            }
            if (myEnd != fulcrum) {
              if (useInputStart) {
                updateRotationAngle(myEnd, inputStart, fulcrum, clockwise,
                                    rotationAngle);
              }
              if (useInputEnd) {
                updateRotationAngle(myEnd, inputEnd, fulcrum, clockwise,
                                    rotationAngle);
              }
            }
            const RealNum adjustAngle =
                std::isinf(rotationAngle) ? 0 : (rotationAngle / 2.0);
            BezierCurveQ::RotateAgainstResult endpointOutput;
            // The rotation ensures that the endpoints' initial intersections
            // are not erroneously found to be blocking.
            rotate(fulcrum, adjustAngle * (clockwise ? (-1.0) : 1.0))
                .endpointRotationConditions(myDistanceCrits, input,
                                            inputDistanceCrits, fulcrum,
                                            clockwise, endpointOutput);
            debug() << "BezierCurveQ::rotateAgainst - rotating to calculate "
                       "endpoint angles,"
                    << " adjustAngle: " << std::to_string(adjustAngle)
                    << " recursive result: "
                    << std::to_string(endpointOutput.angle) << std::endl;
            if (endpointOutput.angle >= 0) {
              endpointOutput.angle += adjustAngle;
            }
            output = endpointOutput;
            output.infiniteInitialIntersection = true;
            infiniteIntersectionAtFulcrum = true;
          } else {
            output.angle = 0;
            output.param = myCurrentCrit;
            output.inputParam = inputCurrentCrit;
            LocalFunctions::
                populateBlockedAngleIntervalsForInfiniteIntersectionAtNonFulcrumCrit(
                    *this, myCurrentCrit, fulcrum, clockwise,
                    output.blockedCWVerticalAngleStart,
                    output.blockedCWVerticalAngleEnd);
            output.atMyCrit = true;
            output.atInputCrit = true;
            return;
          }
        }
        if (infiniteIntersectionAtFulcrum) {
          debug() << "BezierCurveQ::rotateAgainst - "
                     "infiniteIntersectionAtFulcrum case - "
                  << std::to_string(output.angle) << std::endl;
          /*
          If there was an infinite intersection between these 2 curves, no other
          tests will yield a smaller angle.
          */
          return;
        }
        // Infinite intersection, at least one curve does not touch the fulcrum,
        // there is no common distance crit.
        for (StaticVector<std::pair<RealNum, RealNum>, 5>::const_iterator i =
                 myDistanceCrits.critsAndValues.begin() + 1;
             i != myDistanceCrits.critsAndValues.end(); i++) {
          const RealNum myFarther = std::max((i - 1)->second, i->second);
          const RealNum myCloser = std::min((i - 1)->second, i->second);
          for (StaticVector<std::pair<RealNum, RealNum>, 5>::const_iterator j =
                   inputDistanceCrits.critsAndValues.begin() + 1;
               j != inputDistanceCrits.critsAndValues.end(); j++) {
            const RealNum inputFarther = std::max((j - 1)->second, j->second);
            const RealNum inputCloser = std::min((j - 1)->second, j->second);
            const RealNum overlapSize = std::min(myFarther, inputFarther) -
                                        std::max(myCloser, inputCloser);
            if (overlapSize > 0 && !sufficientlySmall(overlapSize)) {
              RealNum myParam, inputParam;
              if (myFarther < inputFarther) {
                myParam =
                    (i - 1)->second > i->second ? (i - 1)->first : i->first;
                StaticVector<RealNum, 4> inputParams(
                    input.getCurveDistanceParams(
                        fulcrum, valueAt(myParam).distanceFrom(fulcrum),
                        (j - 1)->first, j->first));
                if (inputParams.size() != 1) {
                  throw std::string(
                      "UNEXPECTED - input curve cannot match distance in "
                      "overlapping distance interval.");
                }
                inputParam = inputParams[0];
              } else {
                inputParam =
                    (j - 1)->second > j->second ? (j - 1)->first : j->first;
                StaticVector<RealNum, 4> myParams(getCurveDistanceParams(
                    fulcrum, input.valueAt(inputParam).distanceFrom(fulcrum),
                    (i - 1)->first, i->first));
                if (myParams.size() != 1) {
                  throw std::string(
                      "UNEXPECTED - this curve cannot match distance in "
                      "overlapping distance interval.");
                }
                myParam = myParams[0];
              }
              const Point2D myPoint(valueAt(myParam));
              output.angle = 360;
              output.param = myParam;
              output.inputParam = inputParam;
              infiniteIntersectionBlockedInterval(
                  input, (-1.0) / Point2D::getSlopeBetween(fulcrum, myPoint),
                  clockwise == (myPoint.getY() > fulcrum.getY()),
                  clockwise == (myPoint.getX() < fulcrum.getX()),
                  output.blockedCWVerticalAngleStart,
                  output.blockedCWVerticalAngleEnd);
              output.atMyCrit = false;
              output.atInputCrit = false;
              return;
            }
          }
        }
      }
    } else {
      for (const std::pair<RealNum, RealNum> &currentParam : myIntersections) {
        if (isFulcrumIntersection(*this, input, currentParam.first,
                                  currentParam.second, myDistanceCrits,
                                  inputDistanceCrits)) {
          debug() << "BezierCurveQ::rotateAgainst - intersection at fulcrum."
                  << std::endl;
          RealNum testBlockedCWVerticalAngleStart,
              testBlockedCWVerticalAngleEnd;
          bool testAtMyCrit, testAtInputCrit;
          if (curveIntersectionBlocksRotate(
                  currentParam.first, myDistanceCrits, input,
                  currentParam.second, inputDistanceCrits, fulcrum, clockwise,
                  testBlockedCWVerticalAngleStart,
                  testBlockedCWVerticalAngleEnd, testAtMyCrit,
                  testAtInputCrit)) {
            debug() << "BezierCurveQ::rotateAgainst - blocked at fulcrum."
                    << std::endl;
            output.angle = 0;
            output.param = currentParam.first;
            output.inputParam = currentParam.second;
            output.blockedCWVerticalAngleStart =
                testBlockedCWVerticalAngleStart;
            output.blockedCWVerticalAngleEnd = testBlockedCWVerticalAngleEnd;
            output.infiniteInitialIntersection = false;
            output.atMyCrit = testAtMyCrit;
            output.atInputCrit = testAtInputCrit;
            return;
          }
          static const struct {
            bool operator()(const PolynomialFunction<3> &input,
                            const RealNum &inputParam,
                            bool testFunctionIncrease,
                            bool onlyTestParamIncrease,
                            bool onlyTestParamDecrease) const {
              if (onlyTestParamIncrease && onlyTestParamDecrease) {
                throw std::string("Invalid checkDirection parameters.");
              }
              const PolynomialFunction<2> inputDerivative(
                  input.getDerivative());
              {
                RealNum derivativeValue = inputDerivative.valueAt(inputParam);
                if (!sufficientlySmall(derivativeValue)) {
                  bool result = testFunctionIncrease ? derivativeValue > 0
                                                     : derivativeValue < 0;
                  return (result && !onlyTestParamDecrease) ||
                         (!result && !onlyTestParamIncrease);
                }
              }
              { // Since the input function is always degree 2 or lower, a
                // critical point above means this must be a point that the
                // curve changes vertical direction.
                RealNum secondDerivativeValue =
                    inputDerivative.getDerivative().valueAt(inputParam);
                if (!sufficientlySmall(secondDerivativeValue)) {
                  return testFunctionIncrease ? secondDerivativeValue > 0
                                              : secondDerivativeValue < 0;
                }
              }
              return false;
            }
          } checkDirection;
          const bool atMyStart =
              sufficientlyCloseAlongCurve(currentParam.first, 0);
          const bool atMyEnd =
              sufficientlyCloseAlongCurve(currentParam.first, 1);
          const bool atInputStart =
              input.sufficientlyCloseAlongCurve(currentParam.second, 0);
          const bool atInputEnd =
              input.sufficientlyCloseAlongCurve(currentParam.second, 1);
          const bool myGoingUp = checkDirection(
              getParaFormY(), currentParam.first, true, atMyStart, atMyEnd);
          const bool myGoingDown = checkDirection(
              getParaFormY(), currentParam.first, false, atMyStart, atMyEnd);
          const bool myGoingRight = checkDirection(
              getParaFormX(), currentParam.first, true, atMyStart, atMyEnd);
          const bool myGoingLeft = checkDirection(
              getParaFormX(), currentParam.first, false, atMyStart, atMyEnd);
          const bool inputGoingUp =
              checkDirection(input.getParaFormY(), currentParam.second, true,
                             atInputStart, atInputEnd);
          const bool inputGoingDown =
              checkDirection(input.getParaFormY(), currentParam.second, false,
                             atInputStart, atInputEnd);
          const bool inputGoingRight =
              checkDirection(input.getParaFormX(), currentParam.second, true,
                             atInputStart, atInputEnd);
          const bool inputGoingLeft =
              checkDirection(input.getParaFormX(), currentParam.second, false,
                             atInputStart, atInputEnd);
          const RealNum mySlope = rateOfChangeAtParam(currentParam.first);
          const RealNum inputSlope =
              input.rateOfChangeAtParam(currentParam.second);
          // If the intersection point is an endpoint the two test points from a
          // curve will represent the same point.  This is ok.
          const Point2D myPoint(valueAt(currentParam.first));
          const Point2D inputPoint(input.valueAt(currentParam.second));
          const Point2D myTestPoint1(
              myPoint.shift(100, mySlope, myGoingRight, myGoingUp));
          const Point2D myTestPoint2(
              myPoint.shift(100, mySlope, !myGoingLeft, !myGoingDown));
          const Point2D inputTestPoint1(
              inputPoint.shift(100, inputSlope, inputGoingRight, inputGoingUp));
          const Point2D inputTestPoint2(inputPoint.shift(
              100, inputSlope, !inputGoingLeft, !inputGoingDown));
          debug() << "BezierCurveQ::rotateAgainst -"
                  << " myTestPoint1: " << myTestPoint1
                  << " myTestPoint2: " << myTestPoint2 << " mySlope "
                  << std::to_string(mySlope)
                  << " inputTestPoint1: " << inputTestPoint1
                  << " inputTestPoint2: " << inputTestPoint2 << " inputSlope "
                  << std::to_string(inputSlope) << std::endl;
          const struct {
            bool operator()(const Point2D &startPoint, const Point2D endPoint,
                            const Point2D &fulcrum, bool clockwise,
                            RealNum &currentAngleBetween) const {
              const RealNum angleBetween = Point2D::getAngleBetween(
                  startPoint, endPoint, fulcrum, clockwise);
              if (sufficientlyClose(angleBetween, 360) ||
                  sufficientlySmall(angleBetween)) {
                currentAngleBetween = 0;
                return true;
              }
              if (currentAngleBetween < 0 ||
                  currentAngleBetween > angleBetween) {
                currentAngleBetween = angleBetween;
              }
              return false;
            }
          } smallestRotationAngle;
          RealNum currentAngleBetween = -1;
          const bool stopHere =
              smallestRotationAngle(myTestPoint1, inputTestPoint1, fulcrum,
                                    clockwise, currentAngleBetween) ||
              smallestRotationAngle(myTestPoint1, inputTestPoint2, fulcrum,
                                    clockwise, currentAngleBetween) ||
              smallestRotationAngle(myTestPoint2, inputTestPoint1, fulcrum,
                                    clockwise, currentAngleBetween) ||
              smallestRotationAngle(myTestPoint2, inputTestPoint2, fulcrum,
                                    clockwise, currentAngleBetween);
          if (currentAngleBetween >= 0 &&
              (output.angle < 0 || currentAngleBetween < output.angle)) {
            output.param = currentParam.first;
            output.inputParam = currentParam.second;
            output.angle = currentAngleBetween;
            /*
            The input curve blocks the rotation along it's slope from both sides
            in this case. However, if this rotation were executed the curves'
            slopes would match and a shift block cannot be determined here.
            Therefore, leave the output angle intervals as -1.
            */
            output.atMyCrit = true;
            output.atInputCrit = true;
          }
          debug() << "BezierCurveQ::rotateAgainst - intersection at fulcrum "
                     "after tests,"
                  << " output.angle: " << std::to_string(output.angle)
                  << " output.blockedCWVerticalAngleStart: "
                  << output.blockedCWVerticalAngleStart
                  << " output.blockedCWVerticalAngleEnd: "
                  << output.blockedCWVerticalAngleEnd
                  << " stopHere: " << stopHere << std::endl;
          if (stopHere) {
            return;
          }
        } else {
          const Point2D myPoint(valueAt(currentParam.first));
          const Point2D inputPoint(input.valueAt(currentParam.second));
          debug() << "BezierCurveQ::rotateAgainst - non-fulcrum intersection,"
                  << " my param: " << std::to_string(currentParam.first)
                  << " input param: " << std::to_string(currentParam.second)
                  << " my point: " << myPoint << " input point: " << inputPoint
                  << std::endl;
          RealNum testBlockedCWVerticalAngleStart,
              testBlockedCWVerticalAngleEnd;
          bool testAtMyCrit, testAtInputCrit;
          if (curveIntersectionBlocksRotate(
                  currentParam.first, myDistanceCrits, input,
                  currentParam.second, inputDistanceCrits, fulcrum, clockwise,
                  testBlockedCWVerticalAngleStart,
                  testBlockedCWVerticalAngleEnd, testAtMyCrit,
                  testAtInputCrit)) {
            output.angle = 0;
            output.param = currentParam.first;
            output.inputParam = currentParam.second;
            output.blockedCWVerticalAngleStart =
                testBlockedCWVerticalAngleStart;
            output.blockedCWVerticalAngleEnd = testBlockedCWVerticalAngleEnd;
            output.atMyCrit = testAtMyCrit;
            output.atInputCrit = testAtInputCrit;
            return;
          }
        }
      }
    }
  }
  /*const */ RealNum maxAngle = [&]() -> RealNum {
    StaticVector<std::pair<RealNum, RealNum>, 4> distanceIntervals;
    for (StaticVector<std::pair<RealNum, RealNum>, 5>::const_iterator
             myCurrentCrit = myDistanceCrits.critsAndValues.begin() + 1;
         myCurrentCrit != myDistanceCrits.critsAndValues.end();
         myCurrentCrit++) {
      const RealNum myMin =
          std::min((myCurrentCrit - 1)->second, myCurrentCrit->second);
      const RealNum myMax =
          std::max((myCurrentCrit - 1)->second, myCurrentCrit->second);
      for (StaticVector<std::pair<RealNum, RealNum>, 5>::const_iterator
               inputCurrentCrit = inputDistanceCrits.critsAndValues.begin() + 1;
           inputCurrentCrit != inputDistanceCrits.critsAndValues.end();
           inputCurrentCrit++) {
        const RealNum minCommon =
            std::max(myMin, std::min((inputCurrentCrit - 1)->second,
                                     inputCurrentCrit->second));
        const RealNum maxCommon =
            std::min(myMax, std::max((inputCurrentCrit - 1)->second,
                                     inputCurrentCrit->second));
        if (minCommon < maxCommon && !sufficientlyClose(minCommon, maxCommon)) {
          const StaticVector<std::pair<RealNum, RealNum>, 4>::iterator
              searchResult = std::find_if(
                  distanceIntervals.begin(), distanceIntervals.end(),
                  [&minCommon, &maxCommon](
                      const std::pair<RealNum, RealNum> &input) -> bool {
                    return input.first < maxCommon &&
                           input.second > minCommon &&
                           !sufficientlyClose(input.first, maxCommon) &&
                           !sufficientlyClose(input.second, minCommon);
                  });
          if (searchResult != distanceIntervals.end()) {
            searchResult->first = std::max(searchResult->first, minCommon);
            searchResult->second = std::min(searchResult->second, maxCommon);
          } else {
            distanceIntervals.push_back({minCommon, maxCommon});
          }
        }
      }
    }
    RealNum result = -1;
    for (const std::pair<RealNum, RealNum> &currentInterval :
         distanceIntervals) {
      if (sufficientlySmall(currentInterval.first)) {
        const auto myMinMaxDistanceCrits(std::minmax_element(
            myDistanceCrits.critsAndValues.begin(),
            myDistanceCrits.critsAndValues.end(), compareSecond));
        const auto inputMinMaxDistanceCrits(std::minmax_element(
            inputDistanceCrits.critsAndValues.begin(),
            inputDistanceCrits.critsAndValues.end(), compareSecond));
        if (fuzzyEquals(std::max(myMinMaxDistanceCrits.first->second,
                                 inputMinMaxDistanceCrits.first->second),
                        currentInterval.first)) {
          result = result > 360 ? 360 : result;

          continue;
        }
      }
      const RealNum targetDistance =
          (currentInterval.first + currentInterval.second) / 2.0;
      const StaticVector<RealNum, 4> inputDistanceParams(
          input.getCurveDistanceParams(fulcrum, targetDistance, 0, 1));
      for (const RealNum &myCurrentParam :
           getCurveDistanceParams(fulcrum, targetDistance, 0, 1)) {
        const Point2D myPoint(valueAt(myCurrentParam));
        for (const RealNum &inputCurrentParam : inputDistanceParams) {
          const Point2D inputPoint(input.valueAt(inputCurrentParam));
          const RealNum angleBetween =
              Point2D::getAngleBetween(myPoint, inputPoint, fulcrum, clockwise);
          RealNum blockedCWVerticalAngleStart, blockedCWVerticalAngleEnd;
          bool critPlaceholder1, critPlaceholder2;
          if ((result < 0 || angleBetween < result) &&
              rotate(fulcrum, angleBetween * (clockwise ? (-1.0) : 1.0))
                  .curveIntersectionBlocksRotate(
                      myCurrentParam, myDistanceCrits, input, inputCurrentParam,
                      inputDistanceCrits, fulcrum, clockwise,
                      blockedCWVerticalAngleStart, blockedCWVerticalAngleEnd,
                      critPlaceholder1, critPlaceholder2)) {
            result = angleBetween;
          }
        }
      }
    }
    debug() << "BezierCurveQ::rotateAgainst -"
            << " distance interval count: " << distanceIntervals.size()
            << " max angle: " << std::to_string(result)
            << " current output angle: " << std::to_string(output.angle)
            << std::endl;
    return result;
  }();
  { // Aligned distance crit conditions.
    /*
     * When one curve's distance crit is slightly inside the other's (i.e.
     * closer/farther from the fulcrum in the direction of both curves'
     * concavity) but less concave, these cases become relevant.  The initial
     * intersection of the curves may not cause sufficient overlap to cause a
     * block, and they may both be in the same distance crit interval on one
     * curve, also not resulting in a block.  This means there will always be an
     * overlap in the curves, they will never have the same distane and angular
     * slope, and therefore will not match the search conditions employed later
     * in this function.
     *
     * The goal here is to detect where this inevitable overlap reaches a point
     * that 'balances' the overlapping distance crit intervals between the
     * curves - a simple method for doing this is searching for, aliging, and
     * testing non-endpoint distance crits in both curves that are concave in
     * the same direction.
     * */
    for (const std::pair<RealNum, RealNum> &myCurrentCrit :
         myDistanceCrits.critsAndValues) {
      const Point2D myCurrentCritPoint(valueAt(myCurrentCrit.first));
      for (const std::pair<RealNum, RealNum> &inputCurrentCrit :
           inputDistanceCrits.critsAndValues) {
        if (!sufficientlyClose(myCurrentCrit.second,
                               inputCurrentCrit.second)) { // Same crit distance
                                                           // for both curves.
          continue;
        }
        if ((sufficientlySmall(myCurrentCrit.second) ||
             sufficientlySmall(inputCurrentCrit.second)) &&
            isFulcrumIntersection(*this, input, myCurrentCrit.first,
                                  inputCurrentCrit.first, myDistanceCrits,
                                  inputDistanceCrits)) {
          const RealNum mySlope = rateOfChangeAtParam(myCurrentCrit.first);
          const RealNum inputSlope =
              input.rateOfChangeAtParam(inputCurrentCrit.first);
          const RealNum angleBetweenSlopes = [&]() -> RealNum {
            if (sufficientlyCloseSlopes(mySlope, inputSlope)) {
              return 0;
            }
            const RealNum acuteAngleBetween = radiansToDegrees(std::atan(
                std::tan(std::atan(inputSlope) - std::atan(mySlope))));
            return (clockwise == (acuteAngleBetween < 0))
                       ? acuteAngleBetween
                       : (acuteAngleBetween + (clockwise ? (-180.0) : 180.0));
          }();
          for (const RealNum &currentTestAngle :
               {angleBetweenSlopes,
                angleBetweenSlopes + (clockwise ? (-180.0) : 180.0)}) {
            if (output.angle >= 0 &&
                std::abs(currentTestAngle) > output.angle) {
              break;
            }
            RealNum testBlockedCWVerticalAngleStart,
                testBlockedCWVerticalAngleEnd;
            bool testAtMyCrit, testAtInputCrit;
            if (rotate(fulcrum, currentTestAngle)
                    .curveIntersectionBlocksRotate(
                        myCurrentCrit.first, myDistanceCrits, input,
                        inputCurrentCrit.first, inputDistanceCrits, fulcrum,
                        clockwise, testBlockedCWVerticalAngleStart,
                        testBlockedCWVerticalAngleEnd, testAtMyCrit,
                        testAtInputCrit)) {
              output.angle = std::abs(currentTestAngle);
              output.param = myCurrentCrit.first;
              output.inputParam = inputCurrentCrit.first;
              output.blockedCWVerticalAngleStart =
                  testBlockedCWVerticalAngleStart;
              output.blockedCWVerticalAngleEnd = testBlockedCWVerticalAngleEnd;
              output.atMyCrit = testAtMyCrit;
              output.atInputCrit = testAtInputCrit;
              if (output.angle <= maxAngle) {
                return;
              }
              break;
            }
          }
        } else {
          const Point2D inputCurrentCritPoint(
              input.valueAt(inputCurrentCrit.first));
          const RealNum angleBetween = [&]() -> RealNum {
            const RealNum testAngle = Point2D::getAngleBetween(
                myCurrentCritPoint, inputCurrentCritPoint, fulcrum, clockwise);
            return (myCurrentCritPoint == inputCurrentCritPoint &&
                    testAngle > 180)
                       ? 0
                       : testAngle;
          }();
          if (output.angle < 0 || angleBetween < output.angle) {
            RealNum testBlockedCWVerticalAngleStart,
                testBlockedCWVerticalAngleEnd;
            bool testAtMyCrit, testAtInputCrit;
            const BezierCurveQ rotated(
                rotate(fulcrum, angleBetween * (clockwise ? (-1.0) : 1.0)));
            const StaticVector<std::pair<RealNum, RealNum>, 4>
                rotatedIntersection(rotated.pointsOfIntersection(input));
            debug()
                << "BezierCurveQ::rotateAgainst - testing equal distance crits,"
                << " my param: " << std::to_string(myCurrentCrit.first)
                << " input param: " << std::to_string(inputCurrentCrit.first)
                << " my point: " << myCurrentCritPoint
                << " input point: " << input.valueAt(inputCurrentCrit.first)
                << " my distance: " << std::to_string(myCurrentCrit.second)
                << " input distance: "
                << std::to_string(inputCurrentCrit.second)
                << " angle: " << std::to_string(angleBetween)
                << " rotatedIntersection: " << rotatedIntersection << std::endl;
            if (BezierCurveQ::isIntersectionInfinite(rotatedIntersection)) {
              if (&myCurrentCrit != &(myDistanceCrits.critsAndValues.front()) &&
                  &myCurrentCrit != &(myDistanceCrits.critsAndValues.back()) &&
                  &inputCurrentCrit !=
                      &(inputDistanceCrits.critsAndValues.front()) &&
                  &inputCurrentCrit != &(inputDistanceCrits.critsAndValues
                                             .back())) { // Non-endpoints.
                output.angle = angleBetween;
                output.param = myCurrentCrit.first;
                output.inputParam = inputCurrentCrit.first;
                LocalFunctions::
                    populateBlockedAngleIntervalsForInfiniteIntersectionAtNonFulcrumCrit(
                        *this, myCurrentCrit.first, fulcrum, clockwise,
                        output.blockedCWVerticalAngleStart,
                        output.blockedCWVerticalAngleEnd);
                output.atMyCrit = true;
                output.atInputCrit = true;
              }
            } else if (rotated.curveIntersectionBlocksRotate(
                           myCurrentCrit.first, myDistanceCrits, input,
                           inputCurrentCrit.first, inputDistanceCrits, fulcrum,
                           clockwise, testBlockedCWVerticalAngleStart,
                           testBlockedCWVerticalAngleEnd, testAtMyCrit,
                           testAtInputCrit)) {
              /*
               * If the rotated intersection had no entries or one entry, then
               * my crit was concave in the same distance direction as the
               * input, at the same distance from the fulcrum or at a
               * smaller/greater distance in the direction I'm concave, and I am
               * at least as concave as the input.  This means any rotation
               * block will be found by other logic in this function.
               * */
              output.angle = angleBetween;
              output.param = myCurrentCrit.first;
              output.inputParam = inputCurrentCrit.first;
              output.blockedCWVerticalAngleStart =
                  testBlockedCWVerticalAngleStart;
              output.blockedCWVerticalAngleEnd = testBlockedCWVerticalAngleEnd;
              output.atMyCrit = true;
              output.atInputCrit = true;
            }
          }
        }
      }
    }
  }
  { // Endpoint conditions.
    RotateAgainstResult tempOutput;
    endpointRotationConditions(myDistanceCrits, input, inputDistanceCrits,
                               fulcrum, clockwise, tempOutput);
    if (tempOutput.angle >= 0 &&
        (output.angle < 0 || tempOutput.angle < output.angle)) {
      output = tempOutput;
    }
    if (straightLine() &&
        input.straightLine()) { // Both curves are straight lines - this means
                                // that any valid result will be at an endpoint
                                // (covered above).
      return;
    }
  }
  { // Same distance from fulcrum, same 'angular slope' conditions.
    // It is still necessary to call 'testSolution' here because the rates of
    // change may not be equal for endpoint blocks.
    if (maxAngle < 0 ||
        ((output.angle >= 0 && output.angle <= maxAngle) &&
         (fuzzyEquals(output.angle, 0) ||
          LocalFunctions::testSolution(
              *this, myDistanceCrits, input, inputDistanceCrits, fulcrum,
              clockwise, output.angle, output.param, output.inputParam)))) {
      debug() << "BezierCurveQ::rotateAgainst - returning early: "
              << output.angle << std::to_string(output.param) << std::endl;
      return;
    }
    /*
     * Some of the previous tests may assign an output value that is less than
     * anything found in the search below.  Since that result was found to be
     * invalid, clear it out and search for a matching distance and 'angular
     * slope'.
     * */
    output.angle = -1;
    std::array<std::array<RealNum, 4>, 4> slopeDifferenceRelation;
    std::array<std::array<RealNum, 4>, 3> slopeDifferenceRelationTDerivative;
    std::array<std::array<RealNum, 3>, 4> slopeDifferenceRelationUDerivative;
    std::array<std::array<RealNum, 5>, 5> distanceDifferenceRelation;
    PolynomialFunction<4> distanceDifferenceRelationTDerivative({});
    PolynomialFunction<4> distanceDifferenceRelationUDerivative({});
    curveRotationRelations(
        getParaFormX().getCoefficient<2>(), getParaFormX().getCoefficient<1>(),
        getParaFormX().getCoefficient<0>(), getParaFormY().getCoefficient<2>(),
        getParaFormY().getCoefficient<1>(), getParaFormY().getCoefficient<0>(),
        input.getParaFormX().getCoefficient<2>(),
        input.getParaFormX().getCoefficient<1>(),
        input.getParaFormX().getCoefficient<0>(),
        input.getParaFormY().getCoefficient<2>(),
        input.getParaFormY().getCoefficient<1>(),
        input.getParaFormY().getCoefficient<0>(), fulcrum.getX(),
        fulcrum.getY(), slopeDifferenceRelation,
        slopeDifferenceRelationTDerivative, slopeDifferenceRelationUDerivative,
        distanceDifferenceRelation, distanceDifferenceRelationTDerivative,
        distanceDifferenceRelationUDerivative);
    static const StaticVector<StaticVector<RealNum, 64>, 7> testCoeffs(
        {{0.5},
         {0.25, 0.75},
         {0.0, 0.125, 0.375, 0.625, 0.875, 1.0},
         {0.0625, 0.1875, 0.3125, 0.4375, 0.5625, 0.6875, 0.8125, 0.9375},
         {0.03125, 0.09375, 0.15625, 0.21875, 0.28125, 0.34375, 0.40625,
          0.46875, 0.53125, 0.59375, 0.65625, 0.71875, 0.78125, 0.84375,
          0.90625, 0.96875},
         {0.015625, 0.046875, 0.078125, 0.109375, 0.140625, 0.171875, 0.203125,
          0.234375, 0.265625, 0.296875, 0.328125, 0.359375, 0.390625, 0.421875,
          0.453125, 0.484375, 0.515625, 0.546875, 0.578125, 0.609375, 0.640625,
          0.671875, 0.703125, 0.734375, 0.765625, 0.796875, 0.828125, 0.859375,
          0.890625, 0.921875, 0.953125, 0.984375},
         {0.0078125, 0.0234375, 0.0390625, 0.0546875, 0.0703125, 0.0859375,
          0.1015625, 0.1171875, 0.1328125, 0.1484375, 0.1640625, 0.1796875,
          0.1953125, 0.2109375, 0.2265625, 0.2421875, 0.2578125, 0.2734375,
          0.2890625, 0.3046875, 0.3203125, 0.3359375, 0.3515625, 0.3671875,
          0.3828125, 0.3984375, 0.4140625, 0.4296875, 0.4453125, 0.4609375,
          0.4765625, 0.4921875, 0.5078125, 0.5234375, 0.5390625, 0.5546875,
          0.5703125, 0.5859375, 0.6015625, 0.6171875, 0.6328125, 0.6484375,
          0.6640625, 0.6796875, 0.6953125, 0.7109375, 0.7265625, 0.7421875,
          0.7578125, 0.7734375, 0.7890625, 0.8046875, 0.8203125, 0.8359375,
          0.8515625, 0.8671875, 0.8828125, 0.8984375, 0.9140625, 0.9296875,
          0.9453125, 0.9609375, 0.9765625, 0.9921875}});
    for (const StaticVector<RealNum, 64> &currentCoeffSet : testCoeffs) {
      for (const RealNum &myCurrentParam : currentCoeffSet) {
        for (const RealNum &inputCurrentParam : currentCoeffSet) {
          RealNum myCurrentGuess, inputCurrentGuess;
          {
            const std::pair<RealNum, RealNum> newtonResult(twoVarNewton<10>(
                myCurrentParam, inputCurrentParam, slopeDifferenceRelation,
                slopeDifferenceRelationTDerivative,
                slopeDifferenceRelationUDerivative, distanceDifferenceRelation,
                distanceDifferenceRelationTDerivative,
                distanceDifferenceRelationUDerivative));
            myCurrentGuess = newtonResult.first;
            inputCurrentGuess = newtonResult.second;
          }
          {
            RealNum previousDistanceDifferenceRelationVal =
                std::numeric_limits<RealNum>::infinity();
            RealNum previousSlopeDifferenceRelationVal =
                std::numeric_limits<RealNum>::infinity();
            RealNum myRefinedGuess = myCurrentGuess;
            RealNum inputRefinedGuess = inputCurrentGuess;
            for (;;) {
              const RealNum distanceDifferenceRelationNextVal =
                  TwoVarFunctionEvaluator::eval(distanceDifferenceRelation,
                                                myRefinedGuess,
                                                inputRefinedGuess);
              const RealNum slopeDifferenceRelationNextVal =
                  TwoVarFunctionEvaluator::eval(slopeDifferenceRelation,
                                                myRefinedGuess,
                                                inputRefinedGuess);
              if (std::abs(distanceDifferenceRelationNextVal) +
                      std::abs(slopeDifferenceRelationNextVal) >=
                  std::abs(previousDistanceDifferenceRelationVal) +
                      std::abs(previousSlopeDifferenceRelationVal)) {
                break;
              } else {
                myCurrentGuess = myRefinedGuess;
                inputCurrentGuess = inputRefinedGuess;
                previousDistanceDifferenceRelationVal =
                    distanceDifferenceRelationNextVal;
                previousSlopeDifferenceRelationVal =
                    slopeDifferenceRelationNextVal;
                {
                  const std::pair<RealNum, RealNum> newtonResult(
                      twoVarNewton<3>(myCurrentGuess, inputCurrentGuess,
                                      slopeDifferenceRelation,
                                      slopeDifferenceRelationTDerivative,
                                      slopeDifferenceRelationUDerivative,
                                      distanceDifferenceRelation,
                                      distanceDifferenceRelationTDerivative,
                                      distanceDifferenceRelationUDerivative));
                  myRefinedGuess = newtonResult.first;
                  inputRefinedGuess = newtonResult.second;
                }
              }
            }
          }
          if (myCurrentGuess > 0 && myCurrentGuess < 1 &&
              inputCurrentGuess > 0 && inputCurrentGuess < 1) {
            const Point2D myTestPoint(valueAt(myCurrentGuess));
            const Point2D inputTestPoint(input.valueAt(inputCurrentGuess));
            const RealNum angleBetween = Point2D::getAngleBetween(
                myTestPoint, inputTestPoint, fulcrum, clockwise);
            const RealNum myRate = rateOfChangeAtParam(myCurrentGuess);
            const RealNum inputRate =
                input.rateOfChangeAtParam(inputCurrentGuess);
            debug()
                << "BezierCurveQ::rotateAgainst - testing solution,"
                << " myCurrentGuess: " << std::to_string(myCurrentGuess)
                << " inputCurrentGuess: " << std::to_string(inputCurrentGuess)
                << " distance difference relation: "
                << std::to_string(TwoVarFunctionEvaluator::eval(
                       distanceDifferenceRelation, myCurrentGuess,
                       inputCurrentGuess))
                << " slope difference relation: "
                << std::to_string(TwoVarFunctionEvaluator::eval(
                       slopeDifferenceRelation, myCurrentGuess,
                       inputCurrentGuess))
                << " distance difference t RoC: "
                << std::to_string(distanceDifferenceRelationTDerivative.valueAt(
                       myCurrentGuess))
                << " distance difference u RoC: "
                << std::to_string(distanceDifferenceRelationUDerivative.valueAt(
                       inputCurrentGuess))
                << " slope difference t RoC: "
                << std::to_string(TwoVarFunctionEvaluator::eval(
                       slopeDifferenceRelationTDerivative, myCurrentGuess,
                       inputCurrentGuess))
                << " slope difference u RoC: "
                << std::to_string(TwoVarFunctionEvaluator::eval(
                       slopeDifferenceRelationUDerivative, myCurrentGuess,
                       inputCurrentGuess))
                << " angleBetween: " << std::to_string(angleBetween)
                << " my slope (rotated) "
                << std::to_string(rotateSlope(
                       myRate, angleBetween * (clockwise ? (-1.0) : 1.0)))
                << " input slope: " << std::to_string(inputRate)
                << " inputTestPoint: " << inputTestPoint
                << " input param for my rotated slope: "
                << std::to_string(input.paramForSlope(rotateSlope(
                       myRate, angleBetween * (clockwise ? (-1.0) : 1.0))))
                << " my param for input rotated slope: "
                << std::to_string(paramForSlope(rotateSlope(
                       inputRate, angleBetween * (!clockwise ? (-1.0) : 1.0))))
                << " my distance: "
                << std::to_string(myTestPoint.distanceFrom(fulcrum))
                << " input distance: "
                << std::to_string(inputTestPoint.distanceFrom(fulcrum))
                << std::endl;
            if (angleBetween <= maxAngle &&
                (output.angle < 0 || output.angle > angleBetween) &&
                (sufficientlyCloseSlopes(
                     rotateSlope(myRate,
                                 angleBetween * (clockwise ? (-1.0) : 1.0)),
                     inputRate) ||
                 sufficientlyCloseAlongCurve(
                     myCurrentGuess,
                     paramForSlope(rotateSlope(
                         inputRate,
                         angleBetween * (!clockwise ? (-1.0) : 1.0)))) ||
                 input.sufficientlyCloseAlongCurve(
                     inputCurrentGuess,
                     input.paramForSlope(rotateSlope(
                         myRate,
                         angleBetween * (clockwise ? (-1.0) : 1.0))))) &&
                sufficientlyClose(myTestPoint.distanceFrom(fulcrum),
                                  inputTestPoint.distanceFrom(fulcrum))) {
              RealNum testBlockedCWVerticalAngleStart,
                  testBlockedCWVerticalAngleEnd;
              bool testAtMyCrit, testAtInputCrit;
              if (rotate(fulcrum, angleBetween * (clockwise ? (-1.0) : 1.0))
                      .curveIntersectionBlocksRotate(
                          myCurrentGuess, myDistanceCrits, input,
                          inputCurrentGuess, inputDistanceCrits, fulcrum,
                          clockwise, testBlockedCWVerticalAngleStart,
                          testBlockedCWVerticalAngleEnd, testAtMyCrit,
                          testAtInputCrit)) {
                debug() << "BezierCurveQ::rotateAgainst - valid solution found,"
                        << " angleBetween: " << std::to_string(angleBetween)
                        << std::endl;
                output.angle = angleBetween;
                output.param = myCurrentGuess;
                output.inputParam = inputCurrentGuess;
                output.blockedCWVerticalAngleStart =
                    testBlockedCWVerticalAngleStart;
                output.blockedCWVerticalAngleEnd =
                    testBlockedCWVerticalAngleEnd;
                output.atMyCrit = testAtMyCrit;
                output.atInputCrit = testAtInputCrit;
                if (LocalFunctions::testSolution(
                        *this, myDistanceCrits, input, inputDistanceCrits,
                        fulcrum, clockwise, output.angle, myCurrentGuess,
                        inputCurrentGuess)) {
                  return;
                }
              }
            }
          }
        }
      }
    }
    {
      const std::string errorMsg(
          std::string("Could not find acceptable solution, fulcrum: ") +
          fulcrum.toString() + " clockwise: " + (clockwise ? "TRUE" : "FALSE") +
          " this: " + toString() + " input: " + input.toString());
      throw errorMsg;
    }
  }
}

/*
Returns the parameter for this curve that results in a point at the specified
distance from a fulcrum within the specified interval.
*/
StaticVector<RealNum, 4> BezierCurveQ::getCurveDistanceParams(
    const Point2D &fulcrum, const RealNum &distance, const RealNum &startParam,
    const RealNum &endParam) const {
  const PolynomialFunction<5> distanceEquation(
      getDistanceSquaredPF(getParaFormX(), getParaFormY(), fulcrum)
          .subtract(PolynomialFunction<1>({distance * distance})));
  StaticVector<RealNum, 4> roots(
      distanceEquation.getRoots(startParam, endParam));
  debug() << "BezierCurveQ::getCurveDistanceParams -"
          << " this: " << *this << " fulcrum: " << fulcrum
          << " distance: " << std::to_string(distance) << " roots: " << roots
          << " distanceEquation: " << distanceEquation << std::endl;
  if (roots.size() == 0) {
    debug() << "BezierCurveQ::getCurveDistanceParams - no roots found."
            << std::endl;
    if (sufficientlyClose(valueAt(startParam).distanceFrom(fulcrum),
                          distance)) {
      return {startParam};
    }
    if (sufficientlyClose(valueAt(endParam).distanceFrom(fulcrum), distance)) {
      return {endParam};
    }
  }
  StaticVector<RealNum, 4> result;
  for (const RealNum &currentRoot : roots) {
    const RealNum currentDistance(valueAt(currentRoot).distanceFrom(fulcrum));
    if (sufficientlyClose(distance, currentDistance)) {
      result.push_back(currentRoot);
    } else {
      /*
       * Bad roots that are NOT the result of simply being slightly off the [0,
       * 1] range appear to be the result of the fulcrum being along a tangent
       * to the curve at a parameter that was landed on in the root finding.
       * This can result in an extremely large RoC of the distance squared
       * relation, causing inaccurate roots to be accepted.
       *
       * This attempts to fix this situation by assuming that the curve's slope
       * at the current point very closely matches the slope between the current
       * point and the fulcrum - simply move the parameter accordingly, its
       * distance from the target distance divided by the speed along the curve
       * relative to the parameter.
       * */
      RealNum refinedRoot = currentRoot;
      RealNum refinedDistance = currentDistance;
      const PolynomialFunction<4> distanceSquaredDerivative(
          distanceEquation.getDerivative());
      for (int i = 0; i < 3; i++) {
        const RealNum curveSpeed = tangentSpeed(refinedRoot);
        const RealNum derivativeValue =
            distanceSquaredDerivative.valueAt(refinedRoot);
        refinedRoot = std::max(
            std::min(
                refinedRoot +
                    ((std::abs(refinedDistance - distance) / curveSpeed) *
                     (((derivativeValue > 0) == (refinedDistance < distance))
                          ? 1.0
                          : (-1.0))),
                1.0),
            0.0);
        refinedDistance = valueAt(refinedRoot).distanceFrom(fulcrum);
      }
      if (sufficientlyClose(distance, refinedDistance)) {
        result.push_back(refinedRoot);
        debug()
            << "BezierCurveQ::getCurveDistanceParams - accepted refined root,"
            << " currentDistance diff: "
            << std::to_string(std::abs(currentDistance - distance))
            << " refinedDistance diff: "
            << std::to_string(std::abs(refinedDistance - distance))
            << " refinedRoot: " << std::to_string(refinedRoot) << std::endl;
      } else {
        debug() << "BezierCurveQ::getCurveDistanceParams - rejecting root,"
                << " currentRoot: " << std::to_string(currentRoot)
                << " distance relation value: "
                << std::to_string(distanceEquation.valueAt(currentRoot))
                << " distance: "
                << std::to_string(valueAt(currentRoot).distanceFrom(fulcrum))
                << " target: " << std::to_string(distance)
                << " distanceEquation: " << distanceEquation
                << " distanceEquation derivative: "
                << distanceEquation.getDerivative()
                << std::to_string(
                       distanceEquation.getDerivative().valueAt(currentRoot))
                << std::endl;
      }
    }
  }
  return result;
}

CritsAndValues<5>
BezierCurveQ::getDistanceCritsAndValues(const Point2D &input) const {
  CritsAndValues<5> result;
  result.startIsCrit = false;
  result.endIsCrit = false;
  const StaticVector<RealNum, 3> distanceSquaredCrits(
      [&]() -> StaticVector<RealNum, 3> {
        const RealNum a = getParaFormX().getCoefficient<2>();
        const RealNum b = getParaFormX().getCoefficient<1>();
        const RealNum c = getParaFormX().getCoefficient<0>();
        const RealNum d = getParaFormY().getCoefficient<2>();
        const RealNum e = getParaFormY().getCoefficient<1>();
        const RealNum f = getParaFormY().getCoefficient<0>();
        const RealNum g = input.getX();
        const RealNum h = input.getY();
        const RealNum FmH = f - h;
        const RealNum CmG = c - g;
        const PolynomialFunction<4> perpendicularSlope(
            {(e * FmH) + (b * CmG),
             (RealNum(2.0) * ((d * FmH) + (a * CmG))) + (e * e) + (b * b),
             RealNum(3) * ((d * e) + (a * b)),
             RealNum(2) * ((d * d) + (a * a))});
        return perpendicularSlope.getRoots(0, 1);
      }());
  result.critsAndValues.push_back({0, valueAt(0).distanceFrom(input)});
  for (const RealNum &currentCrit : distanceSquaredCrits) {
    if (&currentCrit == &(distanceSquaredCrits.front()) ||
        &currentCrit == &(distanceSquaredCrits.back())) {
      if (sufficientlyCloseAlongCurve(0, currentCrit)) {
        result.startIsCrit = true;
      } else if (sufficientlyCloseAlongCurve(1, currentCrit)) {
        result.endIsCrit = true;
      } else {
        result.critsAndValues.push_back(
            {currentCrit, valueAt(currentCrit).distanceFrom(input)});
      }
    } else {
      result.critsAndValues.push_back(
          {currentCrit, valueAt(currentCrit).distanceFrom(input)});
    }
  }
  result.critsAndValues.push_back({1, valueAt(1).distanceFrom(input)});
  return result;
}

PolynomialFunction<5>
BezierCurveQ::getCurveDistanceSquared(const Point2D &fulcrum) const {
  return getDistanceSquaredPF(getParaFormX(), getParaFormY(), fulcrum);
}

Point2D BezierCurveQ::getDirectionIndicator(const RealNum &parameter) const {
  checkParam(parameter);
  const PolynomialFunction<2> xDerivative(getParaFormX().getDerivative());
  const PolynomialFunction<2> yDerivative(getParaFormY().getDerivative());
  const RealNum slope =
      rateOfChangeAtCurveParam(parameter, xDerivative, yDerivative);
  return valueAt(parameter).shift(100, slope,
                                  xDerivative.valueAt(parameter) >= 0,
                                  yDerivative.valueAt(parameter) >= 0);
}

/*
Returns a ratio representing the curve's concavity at a given parameter.  A
value of 0 would indicate that this curve perfectly models a circle arc (not
possible with quadratic Bezier curves).  The result is always positive, curves
with sharper turn rates at the input parameter return higher values than those
that are closer to a straight line (though perfectly straight lines also return
a positive value).

Return value is always positive.
*/
RealNum BezierCurveQ::getConcavitySlope(const RealNum &parameter) const {
  checkParam(parameter);
  const RealNum SHIFT_DISTANCE = 100;
  const RealNum perpSlope = (-1.0) / rateOfChangeAtParam(parameter);
  const Point2D point(valueAt(parameter));
  const RealNum testDistanceConcavity = getDistanceConcavity(
      parameter, point.shift(SHIFT_DISTANCE, perpSlope, true, true));
  return testDistanceConcavity <
                 getStraightLineDistanceConcavity(SHIFT_DISTANCE)
             ? getDistanceConcavity(
                   parameter,
                   point.shift(SHIFT_DISTANCE, perpSlope, false, false))
             : testDistanceConcavity;
}

/*
Returns the result of shifting the point at the input parameter at a
perpendicular slope to the curve in the direction in which the curve is concave.
Returns the point at the input parameter if there is no concavity.
*/
Point2D BezierCurveQ::getConcavityIndicator(const RealNum &parameter) const {
  checkParam(parameter);
  const Point2D point(valueAt(parameter));
  if (straightLine()) {
    return point;
  } else {
    const RealNum SHIFT_DISTANCE = 1;
    const RealNum perpSlope = (-1.0) / rateOfChangeAtParam(parameter);
    const bool upAndRight =
        getDistanceConcavity(
            parameter, point.shift(SHIFT_DISTANCE, perpSlope, true, true)) <
        getStraightLineDistanceConcavity(SHIFT_DISTANCE);
    return point.shift(100, perpSlope, upAndRight, upAndRight);
  }
}

bool BezierCurveQ::straightLine() const {
  RealNum outputSlope;
  return straightLine(outputSlope);
}

bool BezierCurveQ::straightLine(RealNum &outputSlope) const {
  const Point2D start(valueAt(0));
  const Point2D end(valueAt(1));
  bool straightLine = Point2D::colinear(start, end, getControl());
  outputSlope = straightLine ? Point2D::getSlopeBetween(start, end)
                             : std::numeric_limits<RealNum>::quiet_NaN();
  return straightLine;
}

/*
-populates the blocked interval if a shift was attempted against a curve with
which this has an infinite intersection -this assumes that a shift is not
possible because both curves change perpendicular direction relative to the
direction of the shift
*/
void BezierCurveQ::populateInfiniteIntersectionShiftBlockValues(
    const RealNum &slope, bool right, bool up,
    RealNum &outputBlockedCWVerticalAngleStart,
    RealNum &outputBlockedCWVerticalAngleEnd) const {
  const Point2D myStart(valueAt(0));
  const Point2D myEnd(valueAt(1));
  const RealNum slope1 = Point2D::getSlopeBetween(myStart, getControl());
  const RealNum slope2 = Point2D::getSlopeBetween(myEnd, getControl());
  Point2D testPoints[]{myStart.shift(100, slope1, true, true),
                       myStart.shift(100, slope1, false, false),
                       myStart.shift(100, slope2, true, true),
                       myStart.shift(100, slope2, false, false),
                       myStart.shift(200, slope, right, up)};
  const int arrayLength = sizeof(testPoints) / sizeof(testPoints[0]);
  const Point2D referencePoint(
      myStart.shift(100,
                    Point2D::getSlopeBetween(
                        myStart.shift(myStart.distanceFrom(myEnd) / 2.0,
                                      Point2D::getSlopeBetween(myStart, myEnd),
                                      myEnd.getX() >= myStart.getX(),
                                      myEnd.getY() >= myStart.getY()),
                        getControl()),
                    true, true));
  std::sort(
      testPoints, testPoints + arrayLength,
      [&referencePoint, &myStart](const Point2D &input1,
                                  const Point2D &input2) -> bool {
        return Point2D::getAngleBetween(referencePoint, input1, myStart, true) <
               Point2D::getAngleBetween(referencePoint, input2, myStart, true);
      });
  for (int i = 0; i < arrayLength; i++) {
    if (sufficientlyClose(testPoints[i].distanceFrom(myStart), 200)) {
      if (i == 0 || i == arrayLength - 1) {
        throw std::string("Blocked shift on infinite intersection does not "
                          "change perpendicular magnitude.");
      }
      const Point2D verticalPositiveReferencePoint(myStart.getX(),
                                                   myStart.getY() + 100);
      outputBlockedCWVerticalAngleStart = Point2D::getAngleBetween(
          verticalPositiveReferencePoint, testPoints[i - 1], myStart, true);
      outputBlockedCWVerticalAngleEnd = Point2D::getAngleBetween(
          verticalPositiveReferencePoint, testPoints[i + 1], myStart, true);
      return;
    }
  }
  throw std::string("UNEXPECTED - test shift point was not found.");
}

void BezierCurveQ::getCWAngleIntervals(
    const Point2D &fulcrum, StaticVector<CWAngleInterval, 4> &outputFirstSet,
    StaticVector<CWAngleInterval, 4> &outputSecondSet) const {
  outputFirstSet.clear();
  outputSecondSet.clear();
  const PolynomialFunction<3> slopeRelation(
      curveSlopeMatchesFulcrumSlopePF(getParaFormX(), getParaFormY(), fulcrum));
  const Point2D vertical(fulcrum.getX(), fulcrum.getY() + 100);
  if (slopeRelation.isConstant(0, 1) &&
      sufficientlySmall(slopeRelation.valueAt(
          0))) { // Straight line, pointing at or crossing the fulcrum.
    const Point2D start(valueAt(0));
    const Point2D end(valueAt(1));
    if (start == fulcrum) { // Endpoint touches the fulcrum.
      const RealNum angle =
          Point2D::getAngleBetween(vertical, end, fulcrum, true);
      outputSecondSet = {{0, angle, 1, angle}};
    } else if (end == fulcrum) { // Endpoint touches the fulcrum.
      const RealNum angle =
          Point2D::getAngleBetween(vertical, start, fulcrum, true);
      outputFirstSet = {{0, angle, 1, angle}};
    } else if (start.distanceFrom(fulcrum) <
               start.distanceFrom(end)) { // Fulcrum lies along the line.
      const RealNum startAngle =
          Point2D::getAngleBetween(vertical, start, fulcrum, true);
      const RealNum endAngle =
          Point2D::getAngleBetween(vertical, end, fulcrum, true);
      const RealNum fulcrumParam = minDistance(fulcrum);
      outputFirstSet = {{0, startAngle, fulcrumParam, startAngle}};
      outputSecondSet = {{fulcrumParam, endAngle, 1, endAngle}};
    } else { // Line does not touch the fulcrum.
      const RealNum angle =
          Point2D::getAngleBetween(vertical, end, fulcrum, true);
      outputFirstSet = {{0, angle, 1, angle}};
    }
    return;
  }
  StaticVector<RealNum, 4> testParams;
  {
    // At most, 2 roots.
    const StaticVector<RealNum, 2> relationRoots(
        !slopeRelation.isConstant(0, 1) ? slopeRelation.getRoots(0, 1)
                                        : StaticVector<RealNum, 2>());
    testParams.push_back(0);
    if (!relationRoots.empty()) {
      const Point2D start(valueAt(0));
      const Point2D end(valueAt(1));
      const Point2D firstRootPoint(valueAt(relationRoots.front()));
      if (firstRootPoint != start && firstRootPoint != end) {
        testParams.push_back(relationRoots.front());
      }
      const Point2D lastRootPoint(valueAt(relationRoots.back()));
      if (firstRootPoint != lastRootPoint && lastRootPoint != end) {
        testParams.push_back(relationRoots.back());
      }
    }
    testParams.push_back(1);
  }
  const struct {
    bool operator()(const Point2D &fulcrum, const RealNum &param,
                    const BezierCurveQ &curve, bool useLowerParam,
                    Point2D &outputPoint) const {
      const Point2D point(curve.valueAt(param));
      if (point != fulcrum) {
        outputPoint = point;
        return false;
      }
      const Point2D directionIndicator(curve.getDirectionIndicator(param));
      if (!useLowerParam) {
        outputPoint = directionIndicator;
      } else {
        outputPoint = directionIndicator.rotate(point, 180.0);
      }
      return true;
    }
  } correctForFulcrumPoint;
  int clockwise = 0;
  StaticVector<CWAngleInterval, 4> *destination = &outputFirstSet;
  for (StaticVector<RealNum, 4>::iterator i = testParams.begin() + 1;
       i != testParams.end(); i++) {
    Point2D point1;
    if (correctForFulcrumPoint(fulcrum, *(i - 1), *this, false, point1)) {
      destination = &outputSecondSet;
    } else {
      clockwise *= -1;
    }
    Point2D point2;
    correctForFulcrumPoint(fulcrum, *i, *this, true, point2);
    if (clockwise == 0) {
      const RealNum angleBetween =
          Point2D::getAngleBetween(point1, point2, fulcrum, true);
      const Point2D halfWay(valueAt(((*(i - 1)) + (*i)) / 2.0));
      const RealNum halfWayCWAngle =
          Point2D::getAngleBetween(point1, halfWay, fulcrum, true);
      clockwise = halfWayCWAngle < angleBetween ? 1 : -1;
    }
    if (clockwise == 1) {
      destination->push_back(
          {*(i - 1), Point2D::getAngleBetween(vertical, point1, fulcrum, true),
           *i, Point2D::getAngleBetween(vertical, point2, fulcrum, true)});
    } else {
      destination->push_back(
          {*i, Point2D::getAngleBetween(vertical, point2, fulcrum, true),
           *(i - 1),
           Point2D::getAngleBetween(vertical, point1, fulcrum, true)});
    }
  }
}

RealNum BezierCurveQ::tangentSpeed(const RealNum &parameter) const {
  return std::sqrt(
      std::pow(getParaFormX().getDerivative().valueAt(parameter), 2.0) +
      std::pow(getParaFormY().getDerivative().valueAt(parameter), 2.0));
}

/*
 * Does not necessarily return a value in the [0, 1] range.
 * */
RealNum BezierCurveQ::paramForSlope(const RealNum &slope) const {
  return std::abs(slope) > 1
             ? (getParaFormX().getCoefficient<1>() -
                (getParaFormY().getCoefficient<1>() / slope)) /
                   (2.0 * ((getParaFormY().getCoefficient<2>() / slope) -
                           getParaFormX().getCoefficient<2>()))
             : ((getParaFormX().getCoefficient<1>() * slope) -
                getParaFormY().getCoefficient<1>()) /
                   (2.0 * (getParaFormY().getCoefficient<2>() -
                           (getParaFormX().getCoefficient<2>() * slope)));
}

RealNum BezierCurveQ::straightLineParamForPoint(const RealNum &lineSlope,
                                                const Point2D &targetPoint)
    const { // Assumes that curve is a straight line, and that the target point
  // lies along that line.
  const StaticVector<RealNum, 2> roots(
      std::abs(lineSlope) > 1
          ? getParaFormY()
                .subtract(PolynomialFunction<1>({targetPoint.getY()}))
                .getRoots(0, 1)
          : getParaFormX()
                .subtract(PolynomialFunction<1>({targetPoint.getX()}))
                .getRoots(0, 1));
  if (roots.empty()) {
    return std::numeric_limits<RealNum>::quiet_NaN();
  } else if (roots.size() > 1) {
    throw std::string("Multiple points found in straight line, ensure that the "
                      "control is not outside the start and end points.");
  }
  return roots[0];
}

/*
The speed that this curve moves along the circle formed by the fulcrum at the
curve's point's distance, relative to the curve parameter.  Positive is
clockwise.
*/
RealNum BezierCurveQ::getRotationTangentSpeed(const Point2D &fulcrum,
                                              const RealNum &param) const {
  const Point2D curvePoint(valueAt(param));
  const RealNum tangentSlope((-1.0) /
                             Point2D::getSlopeBetween(curvePoint, fulcrum));
  const Point2D cwTangentPoint(
      curvePoint.shift(100, tangentSlope, curvePoint.getY() >= fulcrum.getY(),
                       curvePoint.getX() <= fulcrum.getX()));
  const Point2D directionIndicator(getDirectionIndicator(param));
  const RealNum angleBetweenCurveAndTangent = Point2D::getAngleBetween(
      cwTangentPoint, directionIndicator, curvePoint, true);
  return tangentSpeed(param) *
         std::cos(degreesToRadians(
             angleBetweenCurveAndTangent)); // Positive is clockwise.
}

/*
 * Rate at which the curve's distance from the fulcrum changes relative to the
 * speed that this curve moves along the circle formed by the fulcrum at the
 * curve's point's distance (see 'getRotationTangentSpeed').
 *
 * A return value of 0 indicates that there is no distance change at this point
 * (curve distance crit). An infinite return value indicates that the curve's
 * slope matches that of the slope between its point and the fulcrum and
 * therefore does not move about the fulcrum at the input parameter.
 *
 * A positive return value means one or more of the following (depending on
 * whether or not the parameter is a curve endpoint):
 *
 * -the curve's distance is increasing in a clockwise direction about the
 * fulcrum. -the curve's distance is decreasing in a counterclockwise direction
 * about the fulcrum.
 *
 * Negative return values are the converse of the positive values.
 */
RealNum
BezierCurveQ::getDistanceSlope(const Point2D &fulcrum, const RealNum &param,
                               const RealNum &rotationTangentSpeed) const {
  const PolynomialFunction<5> distanceSquared(
      getDistanceSquaredPF(getParaFormX(), getParaFormY(), fulcrum));
  const RealNum distanceSquaredRoC =
      distanceSquared.getDerivative().valueAt(param);
  const RealNum distanceSquaredVal = distanceSquared.valueAt(param);
  return distanceSquaredRoC /
         (2.0 * std::sqrt(distanceSquaredVal) * rotationTangentSpeed);
}

RealNum BezierCurveQ::getDistanceConcavity(const RealNum &param,
                                           const Point2D &fulcrum) const {
  const RealNum a = getParaFormX().getCoefficient<2>();
  const RealNum d = getParaFormY().getCoefficient<2>();
  const RealNum xDerivativeVal = getParaFormX().getDerivative().valueAt(param);
  const RealNum yDerivativeVal = getParaFormY().getDerivative().valueAt(param);
  const RealNum curveTangentSpeedSquared =
      std::pow(xDerivativeVal, 2) + std::pow(yDerivativeVal, 2);
  const RealNum curveTangentSpeedVal =
      std::sqrt(curveTangentSpeedSquared); // RESULT
  const RealNum curveTangentSpeedDerivativeVal =
      (2.0 * ((d * yDerivativeVal) + (a * xDerivativeVal))) /
      curveTangentSpeedVal; // RESULT

  const RealNum curveSlope = yDerivativeVal / xDerivativeVal;
  const Point2D point(valueAt(param));
  const RealNum xDiff = point.getX() - fulcrum.getX();
  const RealNum yDiff = point.getY() - fulcrum.getY();
  const RealNum fulcrumSlope = yDiff / xDiff;
  const RealNum curveSlopeCircleTangentSlopeAngle =
      std::atan((-1.0) / fulcrumSlope) - std::atan(curveSlope);
  const RealNum curveDistanceSquared = std::pow(xDiff, 2) + std::pow(yDiff, 2);
  const RealNum curveTangentCoeff =
      std::cos(curveSlopeCircleTangentSlopeAngle); // RESULT
  const RealNum curveTangentCoeffDerivativeVal =
      (-1.0) *
      (((2.0 * (d * xDerivativeVal - (a * yDerivativeVal))) /
        curveTangentSpeedSquared) +
       (((xDerivativeVal * yDiff) - (yDerivativeVal * xDiff)) /
        curveDistanceSquared)) *
      std::sin(curveSlopeCircleTangentSlopeAngle); // RESULT

  const RealNum rotationTangentSpeedVal =
      curveTangentSpeedVal * curveTangentCoeff;
  const RealNum rotationTangentSpeedDerivativeVal =
      (curveTangentSpeedVal * curveTangentCoeffDerivativeVal) +
      (curveTangentCoeff * curveTangentSpeedDerivativeVal); // Product rule.

  const RealNum curveDistance = std::sqrt(curveDistanceSquared);
  const RealNum curveDistanceRoCVal =
      ((yDerivativeVal * yDiff) + (xDerivativeVal * xDiff)) /
      curveDistance; // RESULT
  const RealNum curveDistanceRoCDerivativeVal =
      ((4.0 * curveDistanceSquared *
        (curveTangentSpeedSquared + (2.0 * d * yDiff) + (2.0 * a * xDiff))) -
       std::pow(2.0 * ((yDerivativeVal * yDiff) + (xDerivativeVal * xDiff)),
                2)) /
      (4.0 * std::pow(curveDistance, 3)); // RESULT
  /*
  The distance slope is the curve's distance's rate of change
  (curveDistanceRoCVal) divided by the speed of the curve along the circle
  formed by the fulcrum and the curve's current point
  (curveRotationTangentSpeedVal).  The goal is to find the derivative of this
  relationship.
  */
  const RealNum curveDistanceConcavityVal =
      ((curveDistanceRoCDerivativeVal * rotationTangentSpeedVal) -
       (curveDistanceRoCVal * rotationTangentSpeedDerivativeVal)) /
      std::pow(rotationTangentSpeedVal, 2); // Quotient rule.
  /*
  Since the above value is a rate of change relative to the curve parameter,
  divide by the rotation tangent speed relative to the parameter to get a
  distance-oriented result.
  */
  return curveDistanceConcavityVal / rotationTangentSpeedVal;
}

void BezierCurveQ::shiftAgainstCircleArc(const CircleArc &circleArc,
                                         const RealNum &slope, bool right,
                                         bool up, RealNum &outputDistance,
                                         RealNum &outputParam,
                                         Point2D &outputCirclePoint) const {
  if (std::abs(slope) > 2.4 || std::abs(slope) < 0.4) {
    const RealNum rotationAngle = Point2D::getAngleBetween(
        circleArc.getFulcrum().shift(100, slope, right, up),
        circleArc.getFulcrum().shift(100, 1, true, true),
        circleArc.getFulcrum(), false);
    rotate(circleArc.getFulcrum(), rotationAngle)
        .shiftAgainstCircleArc(
            circleArc.rotate(circleArc.getFulcrum(), rotationAngle), 1, true,
            true, outputDistance, outputParam, outputCirclePoint);
    if (outputDistance >= 0) {
      outputCirclePoint = outputCirclePoint.rotate(circleArc.getFulcrum(),
                                                   (-1.0) * rotationAngle);
    }
    return;
  }
  outputDistance = -1;
  outputParam = -1;
  for (const RealNum &currentParam : {0.0, 1.0}) { // My endpoint conditions.
    const Point2D currentEndpoint(valueAt(currentParam));
    const RealNum directionDistance = 100.0;
    const Point2D directionPoint(
        currentEndpoint.shift(directionDistance, slope, right, up));
    const PolynomialFunction<3> circleYCoordinate(
        pointShiftAgainstCircleYCoordinate(
            circleArc.getRadius(), slope, currentEndpoint.getX(),
            currentEndpoint.getY(), circleArc.getFulcrum().getX(),
            circleArc.getFulcrum().getY()));
    for (const RealNum &currentCircleYCoord : circleYCoordinate.getRoots(
             circleArc.getFulcrum().getY() - circleArc.getRadius(),
             circleArc.getFulcrum().getY() +
                 circleArc.getRadius())) { // Calculation of the circle point
                                           // requires a non-zero slope.
      const Point2D circlePoint((currentCircleYCoord +
                                 (slope * currentEndpoint.getX()) -
                                 currentEndpoint.getY()) /
                                    slope,
                                currentCircleYCoord);
      RealNum distanceBetween = currentEndpoint.distanceFrom(circlePoint);
      if (directionPoint.distanceFrom(circlePoint) > directionDistance) {
        if (sufficientlySmall(distanceBetween)) {
          distanceBetween = 0;
        } else { // The circle point is not in the shift direction.
          continue;
        }
      }
      if (outputDistance >= 0 &&
          outputDistance < distanceBetween) { // A shorter blocking distance
                                              // has already been found.
        continue;
      }
      const CircleArc::PointInArc pointInArc =
          circleArc.liesWithinArc(circlePoint);
      if (pointInArc ==
          CircleArc::PointInArc::NOT_IN_ARC) { // The calculated circle point
                                               // does not lie on the input
                                               // arc.
        continue;
      }
      {
        RealNum placeholder1, placeholder2;
        bool placeholder3, placeholder4;
        // Circle fulcrum intentionally shifted opposite the input direction.
        if (curveIntersectionBlocksShift(
                currentParam, slope, right, up,
                pointInArc == CircleArc::PointInArc::INSIDE ? 0.5 : 0.0,
                pointInArc == CircleArc::PointInArc::INSIDE
                    ? getTestCircleArc(currentParam,
                                       circleArc.getFulcrum().shift(
                                           distanceBetween, slope, !right, !up))
                    : getTestCircleArcForEndpoint(
                          currentParam,
                          circleArc.getFulcrum().shift(distanceBetween, slope,
                                                       !right, !up),
                          pointInArc == CircleArc::PointInArc::START),
                true, placeholder1, placeholder2, placeholder3, placeholder4)) {
          outputDistance = distanceBetween;
          outputParam = currentParam;
          outputCirclePoint = circlePoint;
        }
      }
    }
  }
  if (!circleArc.isFullCircle()) { // Circle arc endpoint conditions.
    const auto arcEndPointConditions =
        [this, &circleArc, &slope, right, up, &outputDistance, &outputParam,
         &outputCirclePoint](bool arcStart) -> void {
      const Point2D arcEndPoint(arcStart ? circleArc.getArcStartPoint()
                                         : circleArc.getArcEndPoint());
      for (const RealNum currentParam :
           pointShiftAgainstParams(arcEndPoint, slope, false)) {
        const Point2D myPoint(valueAt(currentParam));
        RealNum distanceBetween = myPoint.distanceFrom(arcEndPoint);
        const Point2D directionPoint(myPoint.shift(100.0, slope, right, up));
        if (directionPoint.distanceFrom(arcEndPoint) > 100.0) {
          if (sufficientlySmall(distanceBetween)) {
            distanceBetween = 0;
          } else {
            continue;
          }
        }
        if (outputDistance >= 0 && outputDistance <= distanceBetween) {
          continue;
        }
        {
          RealNum placeholder1, placeholder2;
          bool placeholder3, placeholder4;
          // Circle fulcrum intentionally shifted opposite the input direction.
          if (curveIntersectionBlocksShift(
                  currentParam, slope, right, up, 0.0,
                  getTestCircleArcForEndpoint(
                      currentParam,
                      circleArc.getFulcrum().shift(distanceBetween, slope,
                                                   !right, !up),
                      arcStart),
                  true, placeholder1, placeholder2, placeholder3,
                  placeholder4)) {
            outputDistance = distanceBetween;
            outputParam = currentParam;
            outputCirclePoint = arcEndPoint;
          }
        }
      }
    };
    arcEndPointConditions(true);
    arcEndPointConditions(false);
  }
  { // Matching curve slope, matching input slope between points conditions.
    PolynomialFunction<4> shiftAgainstCircleXCoordinateNumerator({});
    PolynomialFunction<4> shiftAgainstCircleYCoordinateNumerator({});
    PolynomialFunction<2> shiftAgainstCircleCoordinateDenominator({});
    PolynomialFunction<8> shiftAgainstCircleCurveParam({});
    shiftAgainstCircle(
        getParaFormX().getCoefficient<2>(), getParaFormX().getCoefficient<1>(),
        getParaFormX().getCoefficient<0>(), getParaFormY().getCoefficient<2>(),
        getParaFormY().getCoefficient<1>(), getParaFormY().getCoefficient<0>(),
        circleArc.getRadius(), slope, circleArc.getFulcrum().getX(),
        circleArc.getFulcrum().getY(), shiftAgainstCircleXCoordinateNumerator,
        shiftAgainstCircleYCoordinateNumerator,
        shiftAgainstCircleCoordinateDenominator, shiftAgainstCircleCurveParam);
    const Point2D curveStart(valueAt(0));
    const Point2D curveEnd(valueAt(1));
    if (!shiftAgainstCircleCurveParam.isConstant(0, 1)) {
      for (const RealNum &currentCurveParam :
           shiftAgainstCircleCurveParam.getRoots(0, 1)) {
        const Point2D curvePoint(valueAt(currentCurveParam));
        const RealNum slopeBetween =
            Point2D::getSlopeBetween(circleArc.getFulcrum(), curvePoint);
        StaticVector<Point2D, 2> circlePoints;
        const RealNum curvePointPerpMag =
            Point2D::getPerpendicularMagnitude(curvePoint, slope);
        const RealNum circleFulcrumPerpMag =
            Point2D::getPerpendicularMagnitude(circleArc.getFulcrum(), slope);
        const RealNum perpMagDiff =
            std::abs(curvePointPerpMag - circleFulcrumPerpMag);
        if (sufficientlySmall(perpMagDiff) &&
            sufficientlyCloseSlopes(slope, slopeBetween)) {
          /*
          When the slope between the curve point and the circle fulcrum matches
          the input shift slope, the calculation of the circle point gets messed
          up because there are 2 points on the circle where the curve slope
          matches the circle slope.
          */
          {
            const RealNum circleFulcrumAngleWithSlope = radiansToDegrees(
                std::asin(perpMagDiff / circleArc.getRadius()));
            const Point2D testCirlePoint(
                circleArc.getFulcrum()
                    .shift(circleArc.getRadius(), slope, true, true)
                    .rotate(circleArc.getFulcrum(),
                            circleFulcrumAngleWithSlope));
            const RealNum testCirclePointPerpMag(
                Point2D::getPerpendicularMagnitude(testCirlePoint, slope));
            if ((testCirclePointPerpMag < circleFulcrumPerpMag) ==
                (curvePointPerpMag < circleFulcrumPerpMag)) {
              circlePoints.push_back(testCirlePoint);
              circlePoints.push_back(testCirlePoint.rotate(
                  circleArc.getFulcrum(),
                  180.0 - (circleFulcrumAngleWithSlope * 2.0)));
            } else {
              circlePoints.push_back(
                  testCirlePoint.rotate(circleArc.getFulcrum(), 180.0));
              circlePoints.push_back(
                  testCirlePoint.rotate(circleArc.getFulcrum(),
                                        circleFulcrumAngleWithSlope * (-2.0)));
            }
          }
        } else {
          const RealNum coordinateDenominator =
              shiftAgainstCircleCoordinateDenominator.valueAt(
                  currentCurveParam);
          const RealNum xCoord = shiftAgainstCircleXCoordinateNumerator.valueAt(
                                     currentCurveParam) /
                                 coordinateDenominator;
          const RealNum yCoord = shiftAgainstCircleYCoordinateNumerator.valueAt(
                                     currentCurveParam) /
                                 coordinateDenominator;
          if (!std::isinf(xCoord) && !std::isnan(xCoord) &&
              !std::isinf(yCoord) && !std::isnan(yCoord)) {
            const Point2D targetPoint(xCoord, yCoord);
            if (sufficientlyClose(
                    targetPoint.distanceFrom(circleArc.getFulcrum()),
                    circleArc.getRadius())) {
              circlePoints.push_back(targetPoint);
            }
          }
        }
        for (const Point2D &circlePoint : circlePoints) {
          RealNum distanceBetween = curvePoint.distanceFrom(circlePoint);
          {
            const Point2D directionPoint(
                curvePoint.shift(100.0, slope, right, up));
            if (directionPoint.distanceFrom(circlePoint) > 100.0) {
              if (sufficientlySmall(distanceBetween)) {
                distanceBetween = 0;
              } else { // The circle point is not in the shift direction.
                continue;
              }
            }
          }
          if (outputDistance >= 0 &&
              outputDistance <=
                  distanceBetween) { // A shorter blocking distance has already
                                     // been found.
            continue;
          }
          if (curvePoint == curveStart || curvePoint == curveEnd ||
              circleArc.liesWithinArc(circlePoint) !=
                  CircleArc::PointInArc::INSIDE) { // Endpoints are handled
                                                   // above.
            continue;
          }
          {
            RealNum placeholder1, placeholder2;
            bool placeholder3, placeholder4;
            // Circle fulcrum intentionally shifted opposite the input
            // direction.
            if (curveIntersectionBlocksShift(
                    currentCurveParam, slope, right, up, 0.5,
                    getTestCircleArc(currentCurveParam,
                                     circleArc.getFulcrum().shift(
                                         distanceBetween, slope, !right, !up)),
                    true, placeholder1, placeholder2, placeholder3,
                    placeholder4)) {
              outputDistance = distanceBetween;
              outputParam = currentCurveParam;
              outputCirclePoint = circlePoint;
            }
          }
        }
      }
    }
  }
}

void BezierCurveQ::rotateAgainstCircleArc(const CircleArc &circleArc,
                                          const Point2D &fulcrum,
                                          bool clockwise, RealNum &outputAngle,
                                          RealNum &outputParam,
                                          Point2D &outputCirclePoint) const {
  struct LocalFunctions {
    static std::pair<Point2D, Point2D>
    circleSlopeMatchesFulcrumSlope(const Point2D &circleFulcrum,
                                   const RealNum &circleRadius,
                                   const Point2D &rotationFulcrum) {
      const RealNum fulcrumsDistance =
          rotationFulcrum.distanceFrom(circleFulcrum);
      if (fulcrumsDistance < circleRadius) {
        throw std::string("Rotation fulcrum is inside circle.");
      }
      const RealNum angleWithLineBetween =
          radiansToDegrees(std::acos(circleRadius / fulcrumsDistance));
      const Point2D alongLineBetweenAtCircleRadius(circleFulcrum.shift(
          circleRadius,
          Point2D::getSlopeBetween(circleFulcrum, rotationFulcrum),
          rotationFulcrum.getX() >= circleFulcrum.getX(),
          rotationFulcrum.getY() >= circleFulcrum.getY()));
      return {alongLineBetweenAtCircleRadius.rotate(circleFulcrum,
                                                    angleWithLineBetween),
              alongLineBetweenAtCircleRadius.rotate(
                  circleFulcrum, (-1.0) * angleWithLineBetween)};
    }

    static RealNum getAngleBetweenForEndpointIntersectionAtFulcrum(
        const BezierCurveQ &curve, const bool atCurveStart,
        const CircleArc &circleArc, const bool atCircleArcStart,
        const Point2D &fulcrum, const bool clockwise) {
      return Point2D::getAngleBetween(
          curve.getTestPointForEndIntersection(atCurveStart),
          fulcrum.shift(100,
                        (-1.0) / Point2D::getSlopeBetween(
                                     fulcrum, circleArc.getFulcrum()),
                        atCircleArcStart
                            ? fulcrum.getY() >= circleArc.getFulcrum().getY()
                            : fulcrum.getY() < circleArc.getFulcrum().getY(),
                        atCircleArcStart
                            ? fulcrum.getX() <= circleArc.getFulcrum().getX()
                            : fulcrum.getX() > circleArc.getFulcrum().getX()),
          fulcrum, clockwise);
    }
  };
  outputAngle = -1;
  outputParam = -1;
  const Point2D myStart(valueAt(0));
  const Point2D myEnd(valueAt(1));
  const CritsAndValues<5> myDistanceCrits(getDistanceCritsAndValues(fulcrum));
  { // Intersection conditions.
    for (const RealNum &currentPotentialIntersection : getCurveDistanceParams(
             circleArc.getFulcrum(), circleArc.getRadius(), 0, 1)) {
      const Point2D myCurrentPoint(valueAt(currentPotentialIntersection));
      const CircleArc::PointInArc pointInArc =
          circleArc.liesWithinArc(myCurrentPoint);
      if (myCurrentPoint == fulcrum &&
          pointInArc != CircleArc::PointInArc::NOT_IN_ARC) {
        RealNum endpointsIntersectionAngleBetween = -1;
        if (myCurrentPoint == myStart) {
          if (pointInArc == CircleArc::PointInArc::START) {
            endpointsIntersectionAngleBetween =
                LocalFunctions::getAngleBetweenForEndpointIntersectionAtFulcrum(
                    *this, true, circleArc, true, fulcrum, clockwise);
          } else if (pointInArc == CircleArc::PointInArc::END) {
            endpointsIntersectionAngleBetween =
                LocalFunctions::getAngleBetweenForEndpointIntersectionAtFulcrum(
                    *this, true, circleArc, false, fulcrum, clockwise);
          }
        } else if (myCurrentPoint == myEnd) {
          if (pointInArc == CircleArc::PointInArc::START) {
            endpointsIntersectionAngleBetween =
                LocalFunctions::getAngleBetweenForEndpointIntersectionAtFulcrum(
                    *this, false, circleArc, true, fulcrum, clockwise);
          } else if (pointInArc == CircleArc::PointInArc::END) {
            endpointsIntersectionAngleBetween =
                LocalFunctions::getAngleBetweenForEndpointIntersectionAtFulcrum(
                    *this, false, circleArc, false, fulcrum, clockwise);
          }
        }
        if (endpointsIntersectionAngleBetween >=
            0) { // Intersection is at an endpoint of both curves.
          if (outputAngle < 0 ||
              outputAngle > endpointsIntersectionAngleBetween) {
            outputAngle = endpointsIntersectionAngleBetween;
            outputParam = currentPotentialIntersection;
            outputCirclePoint = myCurrentPoint;
          }
        } else { // Intersection is not at the endpoint of this curve and/or
                 // the circle arc.
          const RealNum slopeAngleBetween =
              std::atan((-1.0) / Point2D::getSlopeBetween(
                                     myCurrentPoint, circleArc.getFulcrum())) -
              std::atan(rateOfChangeAtParam(currentPotentialIntersection));
          const RealNum rotationAngle = radiansToDegrees(std::abs(
              (slopeAngleBetween == 0 || clockwise == (slopeAngleBetween < 0))
                  ? slopeAngleBetween
                  : (std::abs(slopeAngleBetween) - M_PI)));
          if (outputAngle < 0 || outputAngle > rotationAngle) {
            outputAngle = rotationAngle;
            outputParam = currentPotentialIntersection;
            outputCirclePoint = myCurrentPoint;
          }
        }
      } else if (pointInArc ==
                 CircleArc::PointInArc::INSIDE) { // Not at fulcrum.
        const BezierCurveQ testCircleArc(getTestCircleArc(
            currentPotentialIntersection, circleArc.getFulcrum()));
        RealNum placeholder1, placeholder2;
        bool placeholder3, placeholder4;
        if (curveIntersectionBlocksRotate(
                currentPotentialIntersection, myDistanceCrits, testCircleArc,
                0.5, testCircleArc.getDistanceCritsAndValues(fulcrum), fulcrum,
                clockwise, placeholder1, placeholder2, placeholder3,
                placeholder4)) {
          outputAngle = 0;
          outputParam = currentPotentialIntersection;
          outputCirclePoint = myCurrentPoint;
          return;
        }
      }
    }
  }
  for (const RealNum &currentParam : {0.0, 1.0}) { // My endpoint conditions.
    const Point2D myEndPoint(valueAt(currentParam));
    PolynomialFunction<3> pointRotateAgainstCircleYCoordinate({});
    PolynomialFunction<2> pointRotateAgainstCircleXCoordinateNumerator({});
    RealNum pointRotateAgainstCircleXCoordinateDenominator;
    pointRotateAgainstCircle(
        fulcrum.getX(), fulcrum.getY(), circleArc.getRadius(),
        myEndPoint.getX(), myEndPoint.getY(), circleArc.getFulcrum().getX(),
        circleArc.getFulcrum().getY(), pointRotateAgainstCircleYCoordinate,
        pointRotateAgainstCircleXCoordinateNumerator,
        pointRotateAgainstCircleXCoordinateDenominator);
    for (const RealNum &currentCircleYCoord :
         pointRotateAgainstCircleYCoordinate.getRoots(
             circleArc.getFulcrum().getY() - circleArc.getRadius(),
             circleArc.getFulcrum().getY() + circleArc.getRadius())) {
      const Point2D circlePoint(
          pointRotateAgainstCircleXCoordinateNumerator.valueAt(
              currentCircleYCoord) /
              pointRotateAgainstCircleXCoordinateDenominator,
          currentCircleYCoord);
      if (circlePoint == fulcrum) {
        continue;
      }
      RealNum angleBetween =
          Point2D::getAngleBetween(myEndPoint, circlePoint, fulcrum, clockwise);
      if (sufficientlyClose(angleBetween, 360.0)) {
        angleBetween = 0;
      }
      if ((outputAngle >= 0 && outputAngle < angleBetween)) {
        continue;
      }
      CircleArc::PointInArc arcTest = circleArc.liesWithinArc(circlePoint);
      if (arcTest != CircleArc::PointInArc::NOT_IN_ARC) {
        // Intentionally rotated opposite the input direction.
        const Point2D circleFulcrumRotated(circleArc.getFulcrum().rotate(
            fulcrum, angleBetween * (clockwise ? 1.0 : (-1.0))));
        const BezierCurveQ testCircleArc(
            arcTest == CircleArc::PointInArc::INSIDE
                ? getTestCircleArc(currentParam, circleFulcrumRotated)
                : getTestCircleArcForEndpoint(
                      currentParam, circleFulcrumRotated,
                      arcTest == CircleArc::PointInArc::START));
        RealNum placeholder1, placeholder2;
        bool placeholder3, placeholder4;
        if (curveIntersectionBlocksRotate(
                currentParam, myDistanceCrits, testCircleArc,
                arcTest == CircleArc::PointInArc::INSIDE ? 0.5 : 0.0,
                testCircleArc.getDistanceCritsAndValues(fulcrum), fulcrum,
                clockwise, placeholder1, placeholder2, placeholder3,
                placeholder4)) {
          outputAngle = angleBetween;
          outputParam = currentParam;
          outputCirclePoint = circlePoint;
        }
      }
    }
  }
  if (!circleArc.isFullCircle()) { // Circle arc endpoint conditions.
    const auto arcEndPointConditions =
        [this, &fulcrum, clockwise, &circleArc, &myDistanceCrits, &outputAngle,
         &outputParam, &outputCirclePoint](bool arcStart) -> void {
      const Point2D arcEndPoint(arcStart ? circleArc.getArcStartPoint()
                                         : circleArc.getArcEndPoint());
      for (const RealNum myCurrentParam : getCurveDistanceParams(
               fulcrum, arcEndPoint.distanceFrom(fulcrum), 0, 1)) {
        const Point2D myPoint(valueAt(myCurrentParam));
        const RealNum angleBetween =
            Point2D::getAngleBetween(myPoint, arcEndPoint, fulcrum, clockwise);
        if (outputAngle >= 0 && outputAngle <= angleBetween) {
          continue;
        }
        // Circle fulcrum intentionally rotated opposite the input direction.
        const BezierCurveQ testCircleArc(getTestCircleArcForEndpoint(
            myCurrentParam,
            circleArc.getFulcrum().rotate(
                fulcrum, angleBetween * (clockwise ? 1.0 : (-1.0))),
            arcStart));
        RealNum placeholder1, placeholder2;
        bool placeholder3, placeholder4;
        if (curveIntersectionBlocksRotate(
                myCurrentParam, myDistanceCrits, testCircleArc, 0.0,
                testCircleArc.getDistanceCritsAndValues(fulcrum), fulcrum,
                clockwise, placeholder1, placeholder2, placeholder3,
                placeholder4)) {
          outputAngle = angleBetween;
          outputParam = myCurrentParam;
          outputCirclePoint = arcEndPoint;
        }
      }
    };
    arcEndPointConditions(true);
    arcEndPointConditions(false);
  }
  { // Rotated curve slope matches circle slope at radius conditions.
    const RealNum distanceBetweenFulcrums =
        circleArc.getFulcrum().distanceFrom(fulcrum);
    if (straightLine() &&
        Point2D::colinear(myStart, myEnd,
                          fulcrum)) { // I am a straight line, pointing at or
                                      // crossing the fulcrum.
      if (distanceBetweenFulcrums > circleArc.getRadius() &&
          !sufficientlyClose(distanceBetweenFulcrums, circleArc.getRadius())) {
        /*
         * If the rotation fulcrum is on the circle arc the only blocks will
         * come from preexisting intersections, or endpoint cases, both of which
         * have been handled above.  This is also the case if the rotation
         * fulcrum is inside the circle (whether or not the arc is complete).
         * */
        const std::pair<Point2D, Point2D> circleTouchPoints(
            LocalFunctions::circleSlopeMatchesFulcrumSlope(
                circleArc.getFulcrum(), circleArc.getRadius(), fulcrum));
        const RealNum circlePointDistance =
            circleTouchPoints.first.distanceFrom(fulcrum);
        for (const RealNum &myCurrentParam :
             getCurveDistanceParams(fulcrum, circlePointDistance, 0, 1)) {
          const Point2D myCurrentPoint(valueAt(myCurrentParam));
          for (const Point2D *const currentCircleTouchPoint :
               {&circleTouchPoints.first, &circleTouchPoints.second}) {
            if (circleArc.liesWithinArc(*currentCircleTouchPoint) ==
                CircleArc::PointInArc::INSIDE) { // Endpoint cases were handled
                                                 // previously.
              const RealNum angleBetween = Point2D::getAngleBetween(
                  myCurrentPoint, *currentCircleTouchPoint, fulcrum, clockwise);
              if ((outputAngle < 0 || angleBetween < outputAngle)) {
                // Intentionally rotated opposite the input direction.
                const Point2D circleFulcrumRotated(
                    circleArc.getFulcrum().rotate(
                        fulcrum, angleBetween * (clockwise ? 1.0 : (-1.0))));
                const BezierCurveQ testCircleArc(
                    getTestCircleArc(myCurrentParam, circleFulcrumRotated));
                RealNum placeholder1, placeholder2;
                bool placeholder3, placeholder4;
                if (curveIntersectionBlocksRotate(
                        myCurrentParam, myDistanceCrits, testCircleArc, 0.5,
                        testCircleArc.getDistanceCritsAndValues(fulcrum),
                        fulcrum, clockwise, placeholder1, placeholder2,
                        placeholder3, placeholder4)) {
                  outputAngle = angleBetween;
                  outputParam = myCurrentParam;
                  outputCirclePoint = *currentCircleTouchPoint;
                }
              }
            }
          }
        }
      }
    } else { // Normal case.
      const PolynomialFunction<15> relation(curveRotateTouchesCircle(
          getParaFormX().getCoefficient<2>(),
          getParaFormX().getCoefficient<1>(),
          getParaFormX().getCoefficient<0>(),
          getParaFormY().getCoefficient<2>(),
          getParaFormY().getCoefficient<1>(),
          getParaFormY().getCoefficient<0>(), circleArc.getFulcrum().getX(),
          circleArc.getFulcrum().getY(), circleArc.getRadius(), fulcrum.getX(),
          fulcrum.getY()));
      if (!relation.isConstant(
              0, 1)) { // The degree 0 appears to happen when this curve is
                       // already touching the circle arc.
        for (const RealNum &currentSolution : relation.getRoots(0, 1)) {
          const Point2D curvePoint(valueAt(currentSolution));
          if (curvePoint != myStart && curvePoint != myEnd) {
            const RealNum curveSlope(rateOfChangeAtParam(currentSolution));
            StaticVector<Point2D, 2> targetCircleFulcrums;
            if (sufficientlyCloseSlopes(curveSlope, Point2D::getSlopeBetween(
                                                        curvePoint, fulcrum))) {
              const RealNum perpSlope = (-1.0) / curveSlope;
              for (const Point2D &current :
                   {curvePoint.shift(circleArc.getRadius(), perpSlope, true,
                                     true),
                    curvePoint.shift(circleArc.getRadius(), perpSlope, false,
                                     false)}) {
                if (sufficientlyClose(current.distanceFrom(fulcrum),
                                      distanceBetweenFulcrums)) {
                  targetCircleFulcrums.push_back(current);
                }
              }
            } else {
              RealNum targetFulcrumX, targetFulcrumY;
              getTargetCircleFulcrumForCurveRotate(
                  circleArc.getRadius(), distanceBetweenFulcrums,
                  curvePoint.getX(), curvePoint.getY(), fulcrum.getX(),
                  fulcrum.getY(), curveSlope, targetFulcrumX, targetFulcrumY);
              targetCircleFulcrums.push_back(
                  Point2D(targetFulcrumX, targetFulcrumY));
            }
            for (const Point2D &currentTargetCircleFulcrum :
                 targetCircleFulcrums) {
              const RealNum angleBetween = Point2D::getAngleBetween(
                  circleArc.getFulcrum(), currentTargetCircleFulcrum, fulcrum,
                  !clockwise);
              if ((outputAngle < 0 || outputAngle > angleBetween) &&
                  circleArc
                          .rotate(fulcrum,
                                  angleBetween * (clockwise ? 1.0 : (-1.0)))
                          .liesWithinArc(curvePoint) ==
                      CircleArc::PointInArc::INSIDE) {
                const RealNum curvePointDistance =
                    curvePoint.distanceFrom(fulcrum);
                StaticVector<std::pair<RealNum, RealNum>, 5>::const_iterator
                    myCritPosition,
                    myOtherCritPosition1, myOtherCritPosition2;
                const bool atMyCrit = getCritPosition(
                    currentSolution, DistanceCritCalculator(*this, fulcrum),
                    myDistanceCrits, myCritPosition, myOtherCritPosition1,
                    myOtherCritPosition2);
                RealNum blockedCWVerticalAngleStart, blockedCWVerticalAngleEnd;
                bool placeholder1, placeholder2;
                if ((!atMyCrit // It is ok to use the shift block test instead
                               // of the rotate block test because we are not
                               // at a distance crit.
                     &&
                     curveIntersectionBlocksShift(
                         currentSolution,
                         (-1.0) / Point2D::getSlopeBetween(curvePoint, fulcrum),
                         clockwise == (curvePoint.getY() >= fulcrum.getY()),
                         clockwise == (curvePoint.getX() <= fulcrum.getX()),
                         0.5,
                         getTestCircleArc(currentSolution,
                                          currentTargetCircleFulcrum),
                         true, blockedCWVerticalAngleStart,
                         blockedCWVerticalAngleEnd, placeholder1,
                         placeholder2)) ||
                    (atMyCrit // At neither curves' endpoint, at a distance
                              // crit of both.
                     &&
                     (myCritPosition->second > myOtherCritPosition1->second) ==
                         (curvePointDistance > distanceBetweenFulcrums))) {
                  outputAngle = angleBetween;
                  outputParam = currentSolution;
                  outputCirclePoint = curvePoint.rotate(
                      fulcrum, angleBetween * (clockwise ? (-1.0) : 1.0));
                }
              }
            }
          }
        }
      }
    }
  }
}

BezierCurveQ
BezierCurveQ::getTestCircleArc(const RealNum &parameter,
                               const Point2D &circleFulcrum) const {
  const struct {
    RealNum operator()(const RealNum &startCoord, const RealNum &endCoord,
                       const RealNum &halfCoord) const {
      return ((4.0 * halfCoord) - startCoord - endCoord) / 2.0;
    }
  } controlCoord;
  const PolynomialFunction<4> distanceSquaredDer(
      getCurveDistanceSquared(circleFulcrum).getDerivative());
  /*
  The flag representing if the returned curve should be more concave than the
  circle arc at the input point or less concave.

  TRUE when:

    Either:

    This curve's slope does not match the circle's slope at the input point -
  since the returned curve is only intended to be compare slopes/concavity with
  a circle arc, a curve that simply matches the circle's slope and point is
  sufficient.

    This curve's slope matches that of the circle at the input point, but it is
  less 'concave' than the circle here - return the same value for when the
  slopes don't match because that curve is more concave than the real circle
  arc, meeting this additional requirement.

  FALSE when:

    This curve's slope closely matches that of the circle and it is more
  'concave' than the circle here - return a curve that matches the circle's
  point and slope but is less concave than this curve.
  */
  const bool returnMoreConcave =
      (!sufficientlySmall(distanceSquaredDer.valueAt(parameter)) ||
       distanceSquaredDer.getDerivative().valueAt(parameter) > 0);
  const Point2D curvePoint(valueAt(parameter));
  const RealNum circleRadius = curvePoint.distanceFrom(circleFulcrum);
  const RealNum circleSlope =
      (-1.0) / Point2D::getSlopeBetween(curvePoint, circleFulcrum);
  const Point2D outputStart(circleFulcrum.shift(
      circleRadius * (returnMoreConcave ? 1.0 : 2.0), circleSlope, true, true));
  const Point2D outputEnd(
      circleFulcrum.shift(circleRadius * (returnMoreConcave ? 1.0 : 2.0),
                          circleSlope, false, false));
  return BezierCurveQ(outputStart, outputEnd,
                      Point2D(controlCoord(outputStart.getX(), outputEnd.getX(),
                                           curvePoint.getX()),
                              controlCoord(outputStart.getY(), outputEnd.getY(),
                                           curvePoint.getY())));
}

BezierCurveQ
BezierCurveQ::getTestCircleArcForEndpoint(const RealNum &parameter,
                                          const Point2D &circleFulcrum,
                                          bool clockwiseFromEndpoint) const {
  const Point2D curvePoint(valueAt(parameter));
  const PolynomialFunction<4> distanceSquaredDer(
      getCurveDistanceSquared(circleFulcrum).getDerivative());
  /*
  The flag representing if the returned curve should be less concave than the
  circle arc at the input point or less concave.

  TRUE when:

    Either:

    This curve's slope does not match the circle's slope at the input point -
  since the returned curve is only intended to be compare slopes/concavity with
  a circle arc, a curve that simply matches the circle's slope and point is
  sufficient.

    This curve's slope matches that of the circle at the input point, but it is
  more 'concave' than the circle here - return the same value for when the
  slopes don't match because that curve is less concave than the real circle
  arc, meeting this additional requirement.

  FALSE when:

    This curve's slope closely matches that of the circle and it is less
  'concave' than the circle here - return a curve that matches the circle's
  point and slope but is more concave than this curve.
  */
  const bool returnLessConcave =
      (!sufficientlySmall(distanceSquaredDer.valueAt(parameter)) ||
       distanceSquaredDer.getDerivative().valueAt(parameter) < 0);
  if (returnLessConcave) {
    return BezierCurveQ(
        curvePoint,
        curvePoint.rotate(circleFulcrum,
                          90.0 * (clockwiseFromEndpoint ? (-1.0) : 1.0)),
        circleFulcrum.rotate(curvePoint,
                             90.0 * (!clockwiseFromEndpoint ? (-1.0) : 1.0)));
  } else {
    const RealNum circleSlope =
        (-1.0) / Point2D::getSlopeBetween(curvePoint, circleFulcrum);
    const RealNum halfCircleRadius =
        curvePoint.distanceFrom(circleFulcrum) / 2.0;
    const Point2D circleArcDirection(curvePoint.rotate(
        circleFulcrum, 90.0 * (clockwiseFromEndpoint ? (-1.0) : 1.0)));
    return BezierCurveQ(
        curvePoint,
        circleArcDirection.shift(
            halfCircleRadius, circleSlope,
            circleFulcrum.getX() >= circleArcDirection.getX(),
            circleFulcrum.getY() >= circleArcDirection.getY()),
        circleFulcrum
            .rotate(curvePoint, 90.0 * (!clockwiseFromEndpoint ? (-1.0) : 1.0))
            .shift(halfCircleRadius, circleSlope,
                   circleFulcrum.getX() >= circleArcDirection.getX(),
                   circleFulcrum.getY() >= circleArcDirection.getY()));
  }
}

StaticVector<RealNum, 2> BezierCurveQ::pointShiftAgainstParams(
    const Point2D &input, const RealNum &slope, bool skipIntersections) const {
  debug() << "BezierCurveQ::pointShiftAgainstParams -"
          << " this: " << *this << " input: " << input << " slope: " << slope
          << std::endl;

  const bool swapSlope = std::abs(slope) > 1;
  const RealNum calcSlope = swapSlope ? (1.0 / slope) : slope;
  const PolynomialFunction<3> *const xPara =
      swapSlope ? &(getParaFormY()) : &(getParaFormX());
  const PolynomialFunction<3> *const yPara =
      swapSlope ? &(getParaFormX()) : &(getParaFormY());
  const RealNum yCoord = swapSlope ? input.getX() : input.getY();
  const RealNum xCoord = swapSlope ? input.getY() : input.getX();
  const PolynomialFunction<3> relationship(
      {yPara->getCoefficient<0>() - yCoord +
           (calcSlope * (xCoord - xPara->getCoefficient<0>())),
       yPara->getCoefficient<1>() - (calcSlope * xPara->getCoefficient<1>()),
       yPara->getCoefficient<2>() - (calcSlope * xPara->getCoefficient<2>())});
  /*
  A degree of 0 implies that either the input point lies along this curve, or
  there is no possible point along the input slope that the input point touches
  this curve.
  */
  StaticVector<RealNum, 2> result;
  if (relationship.isConstant(0, 1)) {
    if (skipIntersections) {
      return result;
    }
    if (getParaFormX().effectiveDegree(0, 1) >
        getParaFormY().effectiveDegree(0, 1)) {
      for (const RealNum &currentParam :
           getParaFormX()
               .subtract(PolynomialFunction<1>({input.getX()}))
               .getRoots(0, 1)) {
        if (valueAt(currentParam) == input) {
          result.push_back(currentParam);
        }
      }
    } else {
      for (const RealNum &currentParam :
           getParaFormY()
               .subtract(PolynomialFunction<1>({input.getY()}))
               .getRoots(0, 1)) {
        if (valueAt(currentParam) == input) {
          result.push_back(currentParam);
        }
      }
    }
    return result;
  }
  for (const RealNum &currentRoot : relationship.getRoots(0, 1)) {
    result.push_back(currentRoot);
  }
  return result;
}

/*
 * Note that testParam, unlike curveParam, does NOT have to be a valid parameter
 * along a Bezier curve.
 * */
bool BezierCurveQ::sufficientlyCloseAlongCurve(const RealNum &curveParam,
                                               const RealNum &testParam) const {
  checkParam(curveParam);
  return (std::isinf(testParam) || std::isnan(testParam))
             ? false
             : sufficientlySmall(tangentSpeed(curveParam) *
                                 (curveParam - testParam));
}

/*
 * Returns -1 if the point is not along this curve.
 * */
RealNum BezierCurveQ::paramForPoint(const Point2D &input) const {
  const RealNum a = getParaFormX().getCoefficient<2>();
  const RealNum b = getParaFormX().getCoefficient<1>();
  const RealNum c = getParaFormX().getCoefficient<0>();
  const RealNum d = getParaFormY().getCoefficient<2>();
  const RealNum e = getParaFormY().getCoefficient<1>();
  const RealNum f = getParaFormY().getCoefficient<0>();
  const RealNum x = input.getX();
  const RealNum y = input.getY();
  const RealNum denominator = (a * e) - (b * d);
  RealNum straightLineSlope;
  const bool smallDenominator = sufficientlySmall(denominator);
  if (smallDenominator && straightLine(straightLineSlope)) {
    const RealNum straightLineParam =
        straightLineParamForPoint(straightLineSlope, input);
    return (std::isnan(straightLineParam) ||
            valueAt(straightLineParam) != input)
               ? -1
               : straightLineParam;
  } else { // The fact that the denominator may be very small is taken into
           // account here.
    const RealNum targetParam = ((a * (y - f)) + (d * (c - x))) / denominator;
    return targetParam < 0
               ? sufficientlyCloseAlongCurve(0, targetParam) ? 0 : -1
           : targetParam > 1
               ? sufficientlyCloseAlongCurve(1, targetParam) ? 1 : -1
           : std::isnan(targetParam) ? -1
                                     : targetParam;
  }
}

RealNum BezierCurveQ::getCWVerticalAngleForEndpoint(const BezierCurveQ &input,
                                                    bool start,
                                                    bool intoCurve) {
  const Point2D point(input.valueAt(start ? 0 : 1));
  const Point2D test(input.getTestPointForEndIntersection(start));
  const RealNum slope = Point2D::getSlopeBetween(point, test);
  return _getCWVerticalAngle(
      slope, slope,
      intoCurve ? test.getX() >= point.getX() : test.getX() < point.getX(),
      intoCurve ? test.getY() >= point.getY() : test.getY() < point.getY(),
      false, false);
}

std::ostream &operator<<(std::ostream &os, const BezierCurveQ &input) {
  os << input.toString();
  return os;
}
} // namespace bezier_geometry
