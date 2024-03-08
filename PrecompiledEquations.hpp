#ifndef PRECOMPILEDEQUATIONS_H
#define PRECOMPILEDEQUATIONS_H

#include "PolynomialFunction.hpp"

namespace bezier_geometry {
PolynomialFunction<5>
distanceFromPointSquared(const RealNum &a, const RealNum &b, const RealNum &c,
                         const RealNum &d, const RealNum &e, const RealNum &f,
                         const RealNum &x, const RealNum &y);
PolynomialFunction<5> shiftRelationForSlope(const RealNum &a, const RealNum &b,
                                            const RealNum &c, const RealNum &d,
                                            const RealNum &e, const RealNum &f,
                                            const RealNum &g, const RealNum &h,
                                            const RealNum &i, const RealNum &j,
                                            const RealNum &k, const RealNum &l,
                                            const RealNum &s);
PolynomialFunction<2> shiftSubValueNumerator(const RealNum &b, const RealNum &e,
                                             const RealNum &g, const RealNum &h,
                                             const RealNum &j,
                                             const RealNum &k);
PolynomialFunction<2>
shiftSubValueDenominator(const RealNum &a, const RealNum &d, const RealNum &g,
                         const RealNum &h, const RealNum &j, const RealNum &k);
PolynomialFunction<3> curveSlopeMatchesFulcrumSlope(
    const RealNum &a, const RealNum &b, const RealNum &c, const RealNum &d,
    const RealNum &e, const RealNum &f, const RealNum &x, const RealNum &y);
void pointRotateAgainstCircle(
    const RealNum &i, const RealNum &j, const RealNum &r, const RealNum &v,
    const RealNum &w, const RealNum &x, const RealNum &y,
    PolynomialFunction<3> &outputPointRotateAgainstCircleYCoordinate,
    PolynomialFunction<2> &outputPointRotateAgainstCircleXCoordinateNumerator,
    RealNum &outputPointRotateAgainstCircleXCoordinateDenominator);
PolynomialFunction<3>
pointShiftAgainstCircleYCoordinate(const RealNum &r, const RealNum &s,
                                   const RealNum &v, const RealNum &w,
                                   const RealNum &x, const RealNum &y);
void shiftAgainstCircle(
    const RealNum &a, const RealNum &b, const RealNum &c, const RealNum &d,
    const RealNum &e, const RealNum &f, const RealNum &r, const RealNum &s,
    const RealNum &x, const RealNum &y,
    PolynomialFunction<4> &outputShiftAgainstCircleXCoordinateNumerator,
    PolynomialFunction<4> &outputShiftAgainstCircleYCoordinateNumerator,
    PolynomialFunction<2> &outputShiftAgainstCircleCoordinateDenominator,
    PolynomialFunction<8> &outputShiftAgainstCircle);
void curveRotationRelations(
    const RealNum &a, const RealNum &b, const RealNum &c, const RealNum &d,
    const RealNum &e, const RealNum &f, const RealNum &g, const RealNum &h,
    const RealNum &i, const RealNum &j, const RealNum &k, const RealNum &l,
    const RealNum &x, const RealNum &y,
    std::array<std::array<RealNum, 4>, 4> &outputSlopeDifferenceRelation,
    std::array<std::array<RealNum, 4>, 3>
        &outputSlopeDifferenceRelationTDerivative,
    std::array<std::array<RealNum, 3>, 4>
        &outputSlopeDifferenceRelationUDerivative,
    std::array<std::array<RealNum, 5>, 5> &outputDistanceDifferenceRelation,
    PolynomialFunction<4> &outputDistanceDifferenceRelationTDerivative,
    PolynomialFunction<4> &outputDistanceDifferenceRelationUDerivative);
PolynomialFunction<15>
curveRotateTouchesCircle(const RealNum &a, const RealNum &b, const RealNum &c,
                         const RealNum &d, const RealNum &e, const RealNum &f,
                         const RealNum &g, const RealNum &h, const RealNum &i,
                         const RealNum &j, const RealNum &k);
void getTargetCircleFulcrumForCurveRotate(
    const RealNum &circleRadius,
    const RealNum &targetCircleFulcrumDistanceFromRotationFulcrum,
    const RealNum &curvePointX, const RealNum &curvePointY,
    const RealNum &rotationFulcrumX, const RealNum &rotationFulcrumY,
    const RealNum &curveSlopeAtPoint, RealNum &outputTargetFulcrumX,
    RealNum &outputTargetFulcrumY);

//  template<std::size_t DIM1_SIZE, std::size_t DIM2_SIZE> RealNum
//  eval2VarFunction(
//    const std::array<std::array<RealNum, DIM2_SIZE>, DIM1_SIZE> &input, const
//    RealNum &var1Value, const RealNum &var2Value)
//  {
//    RealNum result = 0.0;
//    if (DIM1_SIZE >= DIM2_SIZE)
//    {
//      for (int i = DIM1_SIZE-1; i >= 0; i--)
//      {
//        RealNum subResult = 0;
//        for (int j = DIM2_SIZE-1; j >= 0; j--)
//        {
//          subResult = (var2Value*subResult)+input[i][j];
//        }
//        result = (var1Value*result)+subResult;
//      }
//    } else
//    {
//      for (int i = DIM2_SIZE-1; i >= 0; i--)
//      {
//        RealNum subResult = 0;
//        for (int j = DIM1_SIZE-1; j >= 0; j--)
//        {
//          subResult = (var1Value*subResult)+input[j][i];
//        }
//        result = (var2Value*result)+subResult;
//      }
//    }
//    return result;
//  }

class TwoVarFunctionEvaluator {
public:
  template <std::size_t DIM1_SIZE, std::size_t DIM2_SIZE>
  static inline RealNum
  eval(const std::array<std::array<RealNum, DIM2_SIZE>, DIM1_SIZE> &input,
       const RealNum &var1Value, const RealNum &var2Value) {
    return _eval(input, var1Value, var2Value);
  }

private:
  template <std::size_t CURRENT_IDX, std::size_t OUTER_LOOP_IDX,
            std::size_t DIM1_SIZE, std::size_t DIM2_SIZE>
  static inline
      typename std::enable_if<(CURRENT_IDX == DIM1_SIZE), RealNum>::type
      innerLoopFirst(
          __attribute__((unused))
          const std::array<std::array<RealNum, DIM2_SIZE>, DIM1_SIZE> &input,
          __attribute__((unused)) const RealNum &var1Value,
          const RealNum &currentValue) {
    return currentValue;
  }

  template <std::size_t CURRENT_IDX, std::size_t OUTER_LOOP_IDX,
            std::size_t DIM1_SIZE, std::size_t DIM2_SIZE>
  static inline
      typename std::enable_if<(CURRENT_IDX < DIM1_SIZE), RealNum>::type
      innerLoopFirst(
          const std::array<std::array<RealNum, DIM2_SIZE>, DIM1_SIZE> &input,
          const RealNum &var1Value, const RealNum &currentValue) {
    return innerLoopFirst<CURRENT_IDX + 1, OUTER_LOOP_IDX>(
        input, var1Value,
        (var1Value * currentValue) +
            input[DIM1_SIZE - 1 - CURRENT_IDX][DIM2_SIZE - 1 - OUTER_LOOP_IDX]);
  }

  template <std::size_t CURRENT_IDX, std::size_t DIM1_SIZE,
            std::size_t DIM2_SIZE>
  static inline
      typename std::enable_if<(CURRENT_IDX == DIM2_SIZE), RealNum>::type
      outerLoopSecond(
          __attribute__((unused))
          const std::array<std::array<RealNum, DIM2_SIZE>, DIM1_SIZE> &input,
          __attribute__((unused)) const RealNum &var1Value,
          __attribute__((unused)) const RealNum &var2Value,
          const RealNum &currentValue) {
    return currentValue;
  }

  template <std::size_t CURRENT_IDX, std::size_t DIM1_SIZE,
            std::size_t DIM2_SIZE>
  static inline
      typename std::enable_if<(CURRENT_IDX < DIM2_SIZE), RealNum>::type
      outerLoopSecond(
          const std::array<std::array<RealNum, DIM2_SIZE>, DIM1_SIZE> &input,
          const RealNum &var1Value, const RealNum &var2Value,
          const RealNum &currentValue) {
    return outerLoopSecond<CURRENT_IDX + 1>(
        input, var1Value, var2Value,
        (var2Value * currentValue) +
            innerLoopFirst<0, CURRENT_IDX>(input, var1Value, 0));
  }

  template <std::size_t CURRENT_IDX, std::size_t OUTER_LOOP_IDX,
            std::size_t DIM1_SIZE, std::size_t DIM2_SIZE>
  static inline
      typename std::enable_if<(CURRENT_IDX == DIM2_SIZE), RealNum>::type
      innerLoopSecond(
          __attribute__((unused))
          const std::array<std::array<RealNum, DIM2_SIZE>, DIM1_SIZE> &input,
          __attribute__((unused)) const RealNum &var2Value,
          const RealNum &currentValue) {
    return currentValue;
  }

  template <std::size_t CURRENT_IDX, std::size_t OUTER_LOOP_IDX,
            std::size_t DIM1_SIZE, std::size_t DIM2_SIZE>
  static inline
      typename std::enable_if<(CURRENT_IDX < DIM2_SIZE), RealNum>::type
      innerLoopSecond(
          const std::array<std::array<RealNum, DIM2_SIZE>, DIM1_SIZE> &input,
          const RealNum &var2Value, const RealNum &currentValue) {
    return innerLoopSecond<CURRENT_IDX + 1, OUTER_LOOP_IDX>(
        input, var2Value,
        (var2Value * currentValue) +
            input[DIM1_SIZE - 1 - OUTER_LOOP_IDX][DIM2_SIZE - 1 - CURRENT_IDX]);
  }

  template <std::size_t CURRENT_IDX, std::size_t DIM1_SIZE,
            std::size_t DIM2_SIZE>
  static inline
      typename std::enable_if<(CURRENT_IDX == DIM1_SIZE), RealNum>::type
      outerLoopFirst(
          __attribute__((unused))
          const std::array<std::array<RealNum, DIM2_SIZE>, DIM1_SIZE> &input,
          __attribute__((unused)) const RealNum &var1Value,
          __attribute__((unused)) const RealNum &var2Value,
          const RealNum &currentValue) {
    return currentValue;
  }

  template <std::size_t CURRENT_IDX, std::size_t DIM1_SIZE,
            std::size_t DIM2_SIZE>
  static inline
      typename std::enable_if<(CURRENT_IDX < DIM1_SIZE), RealNum>::type
      outerLoopFirst(
          const std::array<std::array<RealNum, DIM2_SIZE>, DIM1_SIZE> &input,
          const RealNum &var1Value, const RealNum &var2Value,
          const RealNum &currentValue) {
    return outerLoopFirst<CURRENT_IDX + 1>(
        input, var1Value, var2Value,
        (var1Value * currentValue) +
            innerLoopSecond<0, CURRENT_IDX>(input, var2Value, 0));
  }

  template <std::size_t DIM1_SIZE, std::size_t DIM2_SIZE>
  static inline typename std::enable_if<(DIM1_SIZE >= DIM2_SIZE), RealNum>::type
  _eval(const std::array<std::array<RealNum, DIM2_SIZE>, DIM1_SIZE> &input,
        const RealNum &var1Value, const RealNum &var2Value) {
    return outerLoopFirst<0>(input, var1Value, var2Value, 0);
  }

  template <std::size_t DIM1_SIZE, std::size_t DIM2_SIZE>
  static inline typename std::enable_if<(DIM1_SIZE < DIM2_SIZE), RealNum>::type
  _eval(const std::array<std::array<RealNum, DIM2_SIZE>, DIM1_SIZE> &input,
        const RealNum &var1Value, const RealNum &var2Value) {
    return outerLoopSecond<0>(input, var1Value, var2Value, 0);
  }
};
} // namespace bezier_geometry

#endif // PRECOMPILEDEQUATIONS_H
