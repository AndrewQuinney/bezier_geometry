#include "PolynomialFunction.hpp"

namespace bezier_geometry {
namespace {
const std::vector<std::pair<RealNum, RealNum>>
    INFINITE_SOLUTIONS({{std::numeric_limits<RealNum>::quiet_NaN(),
                         std::numeric_limits<RealNum>::quiet_NaN()}});

void _reduceDeg1WithDeg1(
    const PolynomialFunction<3> &eqHigherCoeffLeft,
    const PolynomialFunction<3> &eqHigherCoeffRight,
    const PolynomialFunction<3> &eqLowerCoeffLeft,
    const PolynomialFunction<3> &eqLowerCoeffRight,
    PolynomialFunction<5> &outputReducedRight,
    PolynomialFunction<3> &outputLeftSolutionNumerator,
    RealNum &outputLeftSolutionDenominator) { // Reduces a system of 2 degree 1
  // polynomial equivalency relations by
  // substituting one into the other.
  /*
  ax + b = cy^2 + dy + e
  fx + g = hy^2 + iy + j
  */
  const RealNum a = eqLowerCoeffLeft.getCoefficient<1>();
  const RealNum b = eqLowerCoeffLeft.getCoefficient<0>();
  const RealNum c = eqLowerCoeffRight.getCoefficient<2>();
  const RealNum d = eqLowerCoeffRight.getCoefficient<1>();
  const RealNum e = eqLowerCoeffRight.getCoefficient<0>();
  const RealNum f = eqHigherCoeffLeft.getCoefficient<1>();
  const RealNum g = eqHigherCoeffLeft.getCoefficient<0>();
  const RealNum h = eqHigherCoeffRight.getCoefficient<2>();
  const RealNum i = eqHigherCoeffRight.getCoefficient<1>();
  const RealNum j = eqHigherCoeffRight.getCoefficient<0>();
  outputLeftSolutionNumerator = PolynomialFunction<3>({(j - g), i, h});
  outputLeftSolutionDenominator =
      f; // Guaranteed to be non-zero, even if it is tiny.
  if (!sufficientlySmall(f)) {
    outputReducedRight =
        PolynomialFunction<5>({((a * (j - g)) / f) + b - e, ((a * i) / f) - d,
                               ((a * h) / f) - c, 0, 0});
  } else {
    outputReducedRight =
        PolynomialFunction<5>({(a * (j - g)) + (f * (b - e)), (a * i) - (f * d),
                               (a * h) - (f * c), 0, 0});
  }
}

void _reduceDeg2WithDeg1(
    const PolynomialFunction<3> &eqDeg2Left,
    const PolynomialFunction<3> &eqDeg2Right,
    const PolynomialFunction<3> &eqDeg1Left,
    const PolynomialFunction<3> &eqDeg1Right,
    PolynomialFunction<5> &outputReducedRight,
    PolynomialFunction<3> &outputLeftSolutionNumerator,
    RealNum
        &outputLeftSolutionDenominator) { // Reduces a system of 1 degree 2 and
  // 1 degree 1 polynomial equivalency
  // relations by substituting the
  // second to eliminate the second
  // variable in the first.
  /*
  ax^2 + bx + c = dy^2 + ey + f
  gx + h = iy^2 + jy + k
  */
  const RealNum a = eqDeg2Left.getCoefficient<2>();
  const RealNum b = eqDeg2Left.getCoefficient<1>();
  const RealNum c = eqDeg2Left.getCoefficient<0>();
  const RealNum d = eqDeg2Right.getCoefficient<2>();
  const RealNum e = eqDeg2Right.getCoefficient<1>();
  const RealNum f = eqDeg2Right.getCoefficient<0>();
  const RealNum g = eqDeg1Left.getCoefficient<1>();
  const RealNum h = eqDeg1Left.getCoefficient<0>();
  const RealNum i = eqDeg1Right.getCoefficient<2>();
  const RealNum j = eqDeg1Right.getCoefficient<1>();
  const RealNum k = eqDeg1Right.getCoefficient<0>();
  outputLeftSolutionNumerator = PolynomialFunction<3>({k - h, j, i});
  outputLeftSolutionDenominator =
      g; // Guaranteed to be non-zero, even if it is tiny.
  const RealNum gSquared = pow(g, 2);
  if (gSquared > 1) {
    outputReducedRight = PolynomialFunction<5>(
        {static_cast<RealNum>(((a * pow(k - h, 2)) / gSquared) +
                              ((b * (k - h)) / g) + c - f),
         static_cast<RealNum>(((a * ((2 * k * j) - (2 * h * j))) / gSquared) +
                              ((b * j) / g) - e),
         static_cast<RealNum>(
             ((a * ((2 * i * k) - (2 * i * h) + pow(j, 2))) / gSquared) +
             ((b * i) / g) - d),
         static_cast<RealNum>((2 * a * i * j) / gSquared),
         static_cast<RealNum>(a * pow(i / g, 2))});
  } else {
    outputReducedRight = PolynomialFunction<5>(
        {static_cast<RealNum>((a * pow(k - h, 2)) +
                              (gSquared * (((b * (k - h)) / g) + c - f))),
         static_cast<RealNum>((a * ((2 * k * j) - (2 * h * j))) +
                              (gSquared * (((b * j) / g) - e))),
         static_cast<RealNum>((a * ((2 * i * k) - (2 * i * h) + pow(j, 2))) +
                              (gSquared * (((b * i) / g) - d))),
         static_cast<RealNum>(2 * a * i * j),
         static_cast<RealNum>(a * pow(i, 2))});
  }
}

void _reduceDeg2WithDeg2(
    const PolynomialFunction<3> &eq1Left, const PolynomialFunction<3> &eq1Right,
    const PolynomialFunction<3> &eq2Left, const PolynomialFunction<3> &eq2Right,
    PolynomialFunction<5> &outputReducedRight,
    PolynomialFunction<3> &outputLeftSolutionNumerator,
    RealNum &outputLeftSolutionDenominator) { // Reduces a system of 2 degree 2
  // polynomial equivalency relations
  // by multiplying and subtracting the
  // second to eliminate the degree 2
  // coefficient and substituting the
  // result.
  /*
  ax^2 + bx + c = dy^2 + ey + f
  gx^2 + hx + i = jy^2 + ky + l
  */
  struct LocalFunctions {
    static void
    reduceDeg2WithDeg2NoDeg1(const PolynomialFunction<3> &eq1Left,
                             const PolynomialFunction<3> &eq1Right,
                             const PolynomialFunction<3> &eq2Left,
                             const PolynomialFunction<3> &eq2Right,
                             PolynomialFunction<5> &outputReducedRight,
                             PolynomialFunction<3> &outputLeftSolutionNumerator,
                             RealNum &outputLeftSolutionDenominator) {
      const RealNum a = eq1Left.getCoefficient<2>();
      const RealNum b =
          eq1Left.getCoefficient<1>(); // Assumed to NOT be very small relative
                                       // to the others.
      const RealNum c = eq1Left.getCoefficient<0>();
      const RealNum d = eq1Right.getCoefficient<2>();
      const RealNum e = eq1Right.getCoefficient<1>();
      const RealNum f = eq1Right.getCoefficient<0>();
      const RealNum g = eq2Left.getCoefficient<2>();
      // const RealNum h = eq2Left.getCoefficient<1>(); // Assumed to be very
      // small relative to the others.
      const RealNum i = eq2Left.getCoefficient<0>();
      const RealNum j = eq2Right.getCoefficient<2>();
      const RealNum k = eq2Right.getCoefficient<1>();
      const RealNum l = eq2Right.getCoefficient<0>();
      const RealNum AdG = a / g;
      const RealNum m = (d - (j * AdG)) / b;           // ((d-aj/g)/b)
      const RealNum n = (e - (k * AdG)) / b;           // ((e-ak/g)/b)
      const RealNum o = (f - c - ((l - i) * AdG)) / b; // (f-c-(l-i)(a/g))/b
      outputLeftSolutionNumerator = PolynomialFunction<3>({o, n, m});
      outputLeftSolutionDenominator = 1;
      outputReducedRight = PolynomialFunction<5>(
          {static_cast<RealNum>((g * pow(o, 2)) + i - l),
           static_cast<RealNum>((2.0 * g * n * o) - k),
           static_cast<RealNum>((2.0 * g * m * o) + (g * pow(n, 2)) - j),
           static_cast<RealNum>(2.0 * g * m * n),
           static_cast<RealNum>(g * pow(m, 2))});
    }
  };
  /******************************************************************************************/
  const RealNum a = eq1Left.getCoefficient<2>();
  const RealNum b = eq1Left.getCoefficient<1>();
  const RealNum c = eq1Left.getCoefficient<0>();
  const RealNum d = eq1Right.getCoefficient<2>();
  const RealNum e = eq1Right.getCoefficient<1>();
  const RealNum f = eq1Right.getCoefficient<0>();
  const RealNum g = eq2Left.getCoefficient<2>();
  const RealNum h = eq2Left.getCoefficient<1>();
  const RealNum i = eq2Left.getCoefficient<0>();
  const RealNum j = eq2Right.getCoefficient<2>();
  const RealNum k = eq2Right.getCoefficient<1>();
  const RealNum l = eq2Right.getCoefficient<0>();
  static const RealNum significanceCoeff = 1000000000.0;
  if (fabs(a) > significanceCoeff * fabs(b) &&
      fabs(g) > significanceCoeff * fabs(h)) {
    throw std::string("Reducing two functions with no degree 1 coefficients is "
                      "currently not implemented.");
  } else if (fabs(a) > significanceCoeff * fabs(b)) {
    LocalFunctions::reduceDeg2WithDeg2NoDeg1(
        eq2Left, eq2Right, eq1Left, eq1Right, outputReducedRight,
        outputLeftSolutionNumerator, outputLeftSolutionDenominator);
  } else if (fabs(g) > significanceCoeff * fabs(h)) {
    LocalFunctions::reduceDeg2WithDeg2NoDeg1(
        eq1Left, eq1Right, eq2Left, eq2Right, outputReducedRight,
        outputLeftSolutionNumerator, outputLeftSolutionDenominator);
  } else if (fabs(b) > significanceCoeff * fabs(a) &&
             fabs(h) > significanceCoeff * fabs(g)) {
    if (fabs(b) > fabs(h)) {
      _reduceDeg1WithDeg1(eq1Left, eq1Right, eq2Left, eq2Right,
                          outputReducedRight, outputLeftSolutionNumerator,
                          outputLeftSolutionDenominator);
    } else {
      _reduceDeg1WithDeg1(eq2Left, eq2Right, eq1Left, eq1Right,
                          outputReducedRight, outputLeftSolutionNumerator,
                          outputLeftSolutionDenominator);
    }
  } else if (fabs(b) > significanceCoeff * fabs(a)) {
    _reduceDeg2WithDeg1(eq2Left, eq2Right, eq1Left, eq1Right,
                        outputReducedRight, outputLeftSolutionNumerator,
                        outputLeftSolutionDenominator);
  } else if (fabs(h) > significanceCoeff * fabs(g)) {
    _reduceDeg2WithDeg1(eq1Left, eq1Right, eq2Left, eq2Right,
                        outputReducedRight, outputLeftSolutionNumerator,
                        outputLeftSolutionDenominator);
  } else {
    const RealNum BGmAH = (b * g) - (a * h);
    const RealNum BGmAHSquared = pow(BGmAH, 2);
    const RealNum DGmAJ = (d * g) - (a * j);
    const RealNum EGmAK = (e * g) - (a * k);
    const RealNum GFmALmCGpAI = (g * f) - (a * l) - (c * g) + (a * i);
    if (!sufficientlySmall(BGmAH)) {
      outputLeftSolutionNumerator =
          PolynomialFunction<3>({GFmALmCGpAI, EGmAK, DGmAJ});
      outputLeftSolutionDenominator = BGmAH;
      outputReducedRight = PolynomialFunction<5>(
          {static_cast<RealNum>((a * pow(GFmALmCGpAI / BGmAH, 2)) +
                                ((b * GFmALmCGpAI) / BGmAH) + c - f),
           static_cast<RealNum>((a * (EGmAK * GFmALmCGpAI * 2) / BGmAHSquared) +
                                ((b * EGmAK) / BGmAH) - e),
           static_cast<RealNum>(
               (a * ((DGmAJ * GFmALmCGpAI * 2) + pow(EGmAK, 2)) /
                BGmAHSquared) +
               ((b * DGmAJ) / BGmAH) - d),
           static_cast<RealNum>((EGmAK * DGmAJ * 2 * a) / BGmAHSquared),
           static_cast<RealNum>((a * pow(DGmAJ, 2)) / BGmAHSquared)});
    } else {
      outputLeftSolutionNumerator = PolynomialFunction<3>({0, 0, 0});
      outputLeftSolutionDenominator = 0;
      outputReducedRight =
          PolynomialFunction<5>({GFmALmCGpAI, EGmAK, DGmAJ, 0, 0});
    }
  }
}

void swapSides(std::vector<std::pair<RealNum, RealNum>> &input) {
  for (std::pair<RealNum, RealNum> &current : input) {
    RealNum temp = current.first;
    current.first = current.second;
    current.second = temp;
  }
}

/*
Uses a sum of squares technique to find parameters that result in both the first
equation's value matching 'firstValue', and the second equation's value matching
'secondValue'.  Both equations must have a degree of 2 or less.

Returns NaN if no such value exists.
*/
std::vector<RealNum>
deg2PairParams(const PolynomialFunction<3> &first,
               const PolynomialFunction<3> &second, const RealNum &firstValue,
               const RealNum &secondValue, const RealNum &lowerBound,
               const RealNum &upperBound, bool returnNextClosest) {
  /*
  If:

  m=firstValue
  n=secondValue

  Then:

  (ax^2 + bx + c - m)^2 + (dx^2 + ex + f - n)^2 = 0
  */
  const RealNum a = first.getCoefficient<2>();
  const RealNum b = first.getCoefficient<1>();
  const RealNum c = first.getCoefficient<0>();
  const RealNum d = second.getCoefficient<2>();
  const RealNum e = second.getCoefficient<1>();
  const RealNum f = second.getCoefficient<0>();
  const RealNum m = firstValue;
  const RealNum n = secondValue;
  std::vector<RealNum> result;
  const PolynomialFunction<5> sumOfSquares(
      {static_cast<RealNum>(pow(c, 2) + pow(f, 2) + pow(m, 2) + pow(n, 2) +
                            ((-2.0) * ((c * m) + (f * n)))),
       static_cast<RealNum>((2.0) * ((b * (c - m)) + (e * (f - n)))),
       static_cast<RealNum>(pow(b, 2) + pow(e, 2) +
                            ((2.0) * ((a * (c - m)) + (d * (f - n))))),
       static_cast<RealNum>((2.0) * ((a * b) + (d * e))),
       static_cast<RealNum>(pow(a, 2) + pow(d, 2))});
  for (const RealNum &current : sumOfSquares.getRoots(lowerBound, upperBound)) {
    result.push_back(current);
  }
  if (result.size() == 0 && returnNextClosest) {
    RealNum placeholder1;
    RealNum placeholder2;
    RealNum placeholder3;
    result.push_back(0);
    sumOfSquares.minMaxValues(lowerBound, upperBound, placeholder1,
                              result.back(), placeholder2, placeholder3);
  }
  return result;
}

std::vector<RealNum> getOtherSideParameters(
    const RealNum &mySideParam, const PolynomialFunction<3> &mySideEq1,
    const PolynomialFunction<3> &mySideEq2,
    const PolynomialFunction<3> &otherSideEq1,
    const PolynomialFunction<3> &otherSideEq2,
    const RealNum &otherSideLowerBound, const RealNum &otherSideUpperBound,
    const PolynomialFunction<3> &otherSideSolutionNumerator,
    const RealNum &otherSideSolutionDenominator,
    bool returnNextClosest = false) {
  const RealNum otherSolution =
      otherSideSolutionNumerator.valueAt(mySideParam) /
      otherSideSolutionDenominator;
  if (std::isinf(otherSolution) || isnan(otherSolution)) {
    /*
    A bad result came from the simple calculation of the other parameter, use a
    sum of squares. This should only be for rare cases.
    */
    if (!sufficientlySmall(otherSideSolutionDenominator)) {
      throw std::string(
          "Invalid division when computing other equation solution.");
    }
    return deg2PairParams(otherSideEq1, otherSideEq2,
                          mySideEq1.valueAt(mySideParam),
                          mySideEq2.valueAt(mySideParam), otherSideLowerBound,
                          otherSideUpperBound, returnNextClosest);
  }
  if (otherSolution > otherSideUpperBound &&
      sufficientlyClose(otherSolution, otherSideUpperBound)) {
    return {otherSideUpperBound};
  }
  if (otherSolution < otherSideLowerBound &&
      sufficientlyClose(otherSolution, otherSideLowerBound)) {
    return {otherSideLowerBound};
  }
  if (otherSolution >= otherSideLowerBound &&
      otherSolution <= otherSideUpperBound) {
    return {otherSolution};
  }
  return {};
}
} // namespace

std::string nTs(const RealNum &input) {
  char output[100];
  return std::string(
      output, snprintf(output, std::size_t(sizeof(output) / sizeof(*output)),
                       "%.20g", input));
}

std::string iTs(const int input) {
  char output[100];
  return std::string(
      output, snprintf(output, std::size_t(sizeof(output) / sizeof(*output)),
                       "%d", input));
}

std::vector<std::pair<RealNum, RealNum>> solveSystem(
    const PolynomialFunction<3> &eq1Left, const PolynomialFunction<3> &eq1Right,
    const PolynomialFunction<3> &eq2Left, const PolynomialFunction<3> &eq2Right,
    const RealNum &leftLowerBound, const RealNum &leftUpperBound,
    const RealNum &rightLowerBound, const RealNum &rightUpperBound) {
  struct LocalFunctions {
    static RealNum
    differencesSquaredSum(const RealNum &leftParam, const RealNum &rightParam,
                          const PolynomialFunction<3> &eq1Left,
                          const PolynomialFunction<3> &eq1Right,
                          const PolynomialFunction<3> &eq2Left,
                          const PolynomialFunction<3> &eq2Right) {
      return pow(eq1Left.valueAt(leftParam) - eq1Right.valueAt(rightParam), 2) +
             pow(eq2Left.valueAt(leftParam) - eq2Right.valueAt(rightParam), 2);
    }

    static RealNum differenceSquaredDerivativeWRTLeftParam(
        const RealNum &leftParam, const RealNum &rightParam,
        const PolynomialFunction<3> &eqLeft,
        const PolynomialFunction<3> &eqRight) {
      const RealNum a = eqLeft.getCoefficient<2>();
      const RealNum b = eqLeft.getCoefficient<1>();
      const RealNum c = eqLeft.getCoefficient<0>();
      const RealNum g = eqRight.getCoefficient<2>();
      const RealNum h = eqRight.getCoefficient<1>();
      const RealNum i = eqRight.getCoefficient<0>();
      const RealNum t = leftParam;
      const RealNum u = rightParam;
      const RealNum UtHpGU = u * (h + (g * u));
      const RealNum CmI = c - i;
      // 2(b(c+t(b+3at)-u(h+gu)-i)+2at(c+at^2-u(h+gu)-i))
      return 2.0 * ((b * (CmI + (t * (b + (3.0 * a * t))) - UtHpGU)) +
                    (2.0 * a * t * (CmI + (a * pow(t, 2)) - UtHpGU)));
    }

    static RealNum differenceSquaredDerivativeWRTRightParam(
        const RealNum &leftParam, const RealNum &rightParam,
        const PolynomialFunction<3> &eqLeft,
        const PolynomialFunction<3> &eqRight) {
      const RealNum a = eqLeft.getCoefficient<2>();
      const RealNum b = eqLeft.getCoefficient<1>();
      const RealNum c = eqLeft.getCoefficient<0>();
      const RealNum g = eqRight.getCoefficient<2>();
      const RealNum h = eqRight.getCoefficient<1>();
      const RealNum i = eqRight.getCoefficient<0>();
      const RealNum t = leftParam;
      const RealNum u = rightParam;
      const RealNum TtBpAT = t * (b + (a * t));
      const RealNum ImC = i - c;
      // 2(h(i-c-t(b+at)+u(h+3gu))+2gu(i-c-t(b+at)+gu^2))
      return 2.0 * ((h * (ImC - TtBpAT + (u * (h + (3.0 * g * u))))) +
                    (2.0 * g * u * (ImC - TtBpAT + (g * pow(u, 2)))));
    }

    static RealNum getLeftSideParam(const PolynomialFunction<3> &eq1Left,
                                    const PolynomialFunction<3> &eq2Left,
                                    const RealNum &leftLowerBound,
                                    const RealNum &leftUpperBound,
                                    const RealNum &eq1LeftVal,
                                    const RealNum eq2LeftVal) {
      const PolynomialFunction<3> eq1LeftEquivalence(
          eq1Left.subtract(PolynomialFunction<3>({eq1LeftVal, 0, 0})));
      const PolynomialFunction<3> eq2LeftEquivalence(
          eq2Left.subtract(PolynomialFunction<3>({eq2LeftVal, 0, 0})));
      const bool eqiv1Constant =
          eq1LeftEquivalence.isConstant(leftLowerBound, leftUpperBound);
      const bool eqiv2Constant =
          eq2LeftEquivalence.isConstant(leftLowerBound, leftUpperBound);
      /*
       * If one of the equivalence relations is constant, the other must not be
       * constant.  If both were constant, the equations would represent a curve
       * that is just one point, which is an invalid case.  Because a curve is
       * constant in one dimension, there is at most one solution to the other
       * dimension's equivalence, otherwise this method would not have been
       * called.
       * */
      if (eqiv1Constant && eqiv2Constant) {
        throw std::string(
            "Unhandled case - both left side equations are constant.");
      } else if (eqiv1Constant) {
        if (!sufficientlySmall(eq1LeftEquivalence.valueAt(0))) {
          return std::numeric_limits<RealNum>::quiet_NaN();
        }
        const std::vector<RealNum> roots(
            eq2LeftEquivalence.getRoots(leftLowerBound, leftUpperBound));
        return roots.empty() ? std::numeric_limits<RealNum>::quiet_NaN()
                             : roots[0];
      } else if (eqiv2Constant) {
        if (!sufficientlySmall(eq2LeftEquivalence.valueAt(0))) {
          return std::numeric_limits<RealNum>::quiet_NaN();
        }
        const std::vector<RealNum> roots(
            eq1LeftEquivalence.getRoots(leftLowerBound, leftUpperBound));
        return roots.empty() ? std::numeric_limits<RealNum>::quiet_NaN()
                             : roots[0];
      } else { // Infinitely intersecting curves will only have one valid
               // solution here.
        if ((eq1Left.getCoefficient<2>() > eq2Left.getCoefficient<2>() &&
             eq1Left.getCoefficient<2>() > eq2Left.getCoefficient<1>()) ||
            (eq1Left.getCoefficient<1>() > eq2Left.getCoefficient<2>() &&
             eq1Left.getCoefficient<1>() > eq2Left.getCoefficient<1>())) {
          for (const RealNum &currentRoot :
               eq1LeftEquivalence.getRoots(leftLowerBound, leftUpperBound)) {
            if (sufficientlyClose(eq2Left.valueAt(currentRoot), eq2LeftVal)) {
              return currentRoot;
            }
          }
          return std::numeric_limits<RealNum>::quiet_NaN();
        } else {
          for (const RealNum &currentRoot :
               eq2LeftEquivalence.getRoots(leftLowerBound, leftUpperBound)) {
            if (sufficientlyClose(eq1Left.valueAt(currentRoot), eq1LeftVal)) {
              return currentRoot;
            }
          }
          return std::numeric_limits<RealNum>::quiet_NaN();
        }
      }
    }

    static bool sufficientlyCloseAlongCurve(const PolynomialFunction<3> &curve,
                                            const RealNum &curveParam,
                                            const RealNum &testParam) {
      return sufficientlySmall(
          sqrt(pow(curveParam - testParam, 2) +
               pow(curve.getDerivative().valueAt(curveParam) *
                       (curveParam - testParam),
                   2)));
    }
  };
  /************************************************************************************************************/
  const std::size_t eq1LeftDegree =
      eq1Left.effectiveDegree(leftLowerBound, leftUpperBound);
  const std::size_t eq2LeftDegree =
      eq2Left.effectiveDegree(leftLowerBound, leftUpperBound);
  const std::size_t eq1RightDegree =
      eq1Right.effectiveDegree(rightLowerBound, rightUpperBound);
  const std::size_t eq2RightDegree =
      eq2Right.effectiveDegree(rightLowerBound, rightUpperBound);
  if (eq1LeftDegree == 0 || eq2LeftDegree == 0) {
    if (eq1RightDegree > 0 && eq2RightDegree > 0) {
      std::vector<std::pair<RealNum, RealNum>> recursiveResult(
          solveSystem(eq1Right, eq1Left, eq2Right, eq2Left, rightLowerBound,
                      rightUpperBound, leftLowerBound, leftUpperBound));
      swapSides(recursiveResult);
      return recursiveResult;
    } else {
      throw std::string(
          "Invalid solveSystem input - degree 0 on left and right.");
    }
  } else if (eq1LeftDegree + eq2LeftDegree > eq1RightDegree + eq2RightDegree &&
             eq1RightDegree > 0 && eq2RightDegree > 0) {
    std::vector<std::pair<RealNum, RealNum>> recursiveResult(
        solveSystem(eq1Right, eq1Left, eq2Right, eq2Left, rightLowerBound,
                    rightUpperBound, leftLowerBound, leftUpperBound));
    swapSides(recursiveResult);
    return recursiveResult;
  } else if (eq1LeftDegree < eq2LeftDegree ||
             (eq1LeftDegree == 1 && eq2LeftDegree == 1 &&
              fabs(eq1Left.getDerivative().valueAt(leftLowerBound)) >
                  fabs(eq2Left.getDerivative().valueAt(leftLowerBound)))) {
    return solveSystem(eq2Left, eq2Right, eq1Left, eq1Right, leftLowerBound,
                       leftUpperBound, rightLowerBound, rightUpperBound);
  }
  /*
  At this point it is guaranteed that:

  -all input equations are of degree equal to or less than 2
  -any degree 0 equations are on the right side
  -the left side of the second equation has a degree equal to or less than the
  left side of the first equation -except for degree 0 equations (right side
  only), no other input function has a degree less than the left side of the
  second equation -if both left sides have a degree of 1, the left side of the
  second equation's max degree coefficient is greater than or equal to that of
  the left side of the first equation
  */
  // The reduced system with the left side's variable eliminated.
  PolynomialFunction<5> outputReduced({});
  // Combined system to express the left side's parameter in terms of the
  // right's (numerator).
  PolynomialFunction<3> outputOtherSolutionNumerator({});
  // Combined system to express the left side's parameter in terms of the
  // right's (denominator).
  RealNum outputOtherSolutionDenominator;
  if (eq2LeftDegree == 1) {
    if (eq1LeftDegree == 2) {
      _reduceDeg2WithDeg1(eq1Left, eq1Right, eq2Left, eq2Right, outputReduced,
                          outputOtherSolutionNumerator,
                          outputOtherSolutionDenominator);
    } else {
      _reduceDeg1WithDeg1(eq2Left, eq2Right, eq1Left, eq1Right, outputReduced,
                          outputOtherSolutionNumerator,
                          outputOtherSolutionDenominator);
    }
  } else {
    /*
    Either all input functions are degree 2 or the inputs consist of only degree
    2 and degree 0. No degree 0 functions are on the left.
    */
    _reduceDeg2WithDeg2(eq1Left, eq1Right, eq2Left, eq2Right, outputReduced,
                        outputOtherSolutionNumerator,
                        outputOtherSolutionDenominator);
    if (sufficientlySmall(outputOtherSolutionDenominator) &&
        eq1RightDegree == 2 && eq2RightDegree == 2) {
      PolynomialFunction<5> testOutputReduced({});
      PolynomialFunction<3> testOutputOtherSolutionNumerator({});
      RealNum testOutputOtherSolutionDenominator;
      _reduceDeg2WithDeg2(eq1Right, eq1Left, eq2Right, eq2Left,
                          testOutputReduced, testOutputOtherSolutionNumerator,
                          testOutputOtherSolutionDenominator);
      if (fabs(testOutputOtherSolutionDenominator) >
          fabs(outputOtherSolutionDenominator)) {
        std::vector<std::pair<RealNum, RealNum>> recursiveResult(
            solveSystem(eq1Right, eq1Left, eq2Right, eq2Left, rightLowerBound,
                        rightUpperBound, leftLowerBound, leftUpperBound));
        swapSides(recursiveResult);
        return recursiveResult;
      }
    }
  }
  std::vector<std::pair<RealNum, RealNum>> result;
  if (outputReduced.isConstant(rightLowerBound, rightUpperBound)) {
    if (sufficientlySmall(outputReduced.valueAt(
            0))) { // There are infinite solutions due to equal curve shapes.
      const RealNum leftParamForRightLower = LocalFunctions::getLeftSideParam(
          eq1Left, eq2Left, leftLowerBound, leftUpperBound,
          eq1Right.valueAt(rightLowerBound), eq2Right.valueAt(rightLowerBound));
      const RealNum leftParamForRightUpper = LocalFunctions::getLeftSideParam(
          eq1Left, eq2Left, leftLowerBound, leftUpperBound,
          eq1Right.valueAt(rightUpperBound), eq2Right.valueAt(rightUpperBound));
      const RealNum rightParamForLeftLower = LocalFunctions::getLeftSideParam(
          eq1Right, eq2Right, rightLowerBound, rightUpperBound,
          eq1Left.valueAt(leftLowerBound), eq2Left.valueAt(leftLowerBound));
      const RealNum rightParamForLeftUpper = LocalFunctions::getLeftSideParam(
          eq1Right, eq2Right, rightLowerBound, rightUpperBound,
          eq1Left.valueAt(leftUpperBound), eq2Left.valueAt(leftUpperBound));
      typedef std::vector<std::pair<RealNum, RealNum>> ResultType;
      if (!isnan(leftParamForRightLower) && !isnan(leftParamForRightUpper)) {
        if (leftParamForRightLower < leftParamForRightUpper) {
          result = ResultType(INFINITE_SOLUTIONS);
          result.push_back({leftParamForRightLower, rightLowerBound});
          result.push_back({leftParamForRightUpper, rightUpperBound});
        } else {
          result = ResultType(INFINITE_SOLUTIONS);
          result.push_back({leftParamForRightUpper, rightUpperBound});
          result.push_back({leftParamForRightLower, rightLowerBound});
        }
      } else if (!isnan(leftParamForRightLower)) {
        if (!isnan(rightParamForLeftLower) &&
            (!LocalFunctions::sufficientlyCloseAlongCurve(
                 eq1Left, leftParamForRightLower, leftLowerBound) ||
             !LocalFunctions::sufficientlyCloseAlongCurve(
                 eq2Left, leftParamForRightLower, leftLowerBound))) {
          result = ResultType(INFINITE_SOLUTIONS);
          result.push_back({leftLowerBound, rightParamForLeftLower});
          result.push_back({leftParamForRightLower, rightLowerBound});
        } else if (!isnan(rightParamForLeftUpper) &&
                   (!LocalFunctions::sufficientlyCloseAlongCurve(
                        eq1Left, leftParamForRightLower, leftUpperBound) ||
                    !LocalFunctions::sufficientlyCloseAlongCurve(
                        eq2Left, leftParamForRightLower, leftUpperBound))) {
          result = ResultType(INFINITE_SOLUTIONS);
          result.push_back({leftParamForRightLower, rightLowerBound});
          result.push_back({leftUpperBound, rightParamForLeftUpper});
        } else {
          result = ResultType({{leftParamForRightLower, rightLowerBound}});
        }
      } else if (!isnan(leftParamForRightUpper)) {
        if (!isnan(rightParamForLeftLower) &&
            (!LocalFunctions::sufficientlyCloseAlongCurve(
                 eq1Left, leftParamForRightUpper, leftLowerBound) ||
             !LocalFunctions::sufficientlyCloseAlongCurve(
                 eq2Left, leftParamForRightUpper, leftLowerBound))) {
          result = ResultType(INFINITE_SOLUTIONS);
          result.push_back({leftLowerBound, rightParamForLeftLower});
          result.push_back({leftParamForRightUpper, rightUpperBound});
        } else if (!isnan(rightParamForLeftUpper) &&
                   (!LocalFunctions::sufficientlyCloseAlongCurve(
                        eq1Left, leftParamForRightUpper, leftUpperBound) ||
                    !LocalFunctions::sufficientlyCloseAlongCurve(
                        eq2Left, leftParamForRightUpper, leftUpperBound))) {
          result = ResultType(INFINITE_SOLUTIONS);
          result.push_back({leftParamForRightUpper, rightUpperBound});
          result.push_back({leftUpperBound, rightParamForLeftUpper});
        } else {
          result = ResultType({{leftParamForRightUpper, rightUpperBound}});
        }
      } else if (!isnan(rightParamForLeftLower) &&
                 !isnan(rightParamForLeftUpper)) {
        result = ResultType(INFINITE_SOLUTIONS);
        result.push_back({leftLowerBound, rightParamForLeftLower});
        result.push_back({leftUpperBound, rightParamForLeftUpper});
      }
    }
  } else {
    for (const RealNum &currentRoot :
         outputReduced.getRoots(rightLowerBound, rightUpperBound)) {
      for (const RealNum &currentLeftSolution : getOtherSideParameters(
               currentRoot, eq1Right, eq2Right, eq1Left, eq2Left,
               leftLowerBound, leftUpperBound, outputOtherSolutionNumerator,
               outputOtherSolutionDenominator)) {
        if (sufficientlyClose(eq1Left.valueAt(currentLeftSolution),
                              eq1Right.valueAt(currentRoot)) &&
            sufficientlyClose(eq2Left.valueAt(currentLeftSolution),
                              eq2Right.valueAt(currentRoot))) {
          result.push_back({currentLeftSolution, currentRoot});
        } else {
          RealNum bestLeftParam, bestRightParam;
          RealNum bestDifferencesSquared =
              std::numeric_limits<RealNum>::infinity();
          {
            RealNum leftParam = currentLeftSolution;
            RealNum rightParam = currentRoot;
            RealNum differencesSquared = LocalFunctions::differencesSquaredSum(
                currentLeftSolution, currentRoot, eq1Left, eq1Right, eq2Left,
                eq2Right);
            for (int i = 0; i < 15; i++) {
              if (differencesSquared < bestDifferencesSquared) {
                if ((leftParam >= leftLowerBound ||
                     sufficientlyClose(leftParam, leftLowerBound)) &&
                    (leftParam <= leftUpperBound ||
                     sufficientlyClose(leftParam, leftUpperBound)) &&
                    (rightParam >= rightLowerBound ||
                     sufficientlyClose(rightParam, rightLowerBound)) &&
                    (rightParam <= rightUpperBound ||
                     sufficientlyClose(rightParam, rightUpperBound))) {
                  bestLeftParam = std::min(std::max(leftParam, leftLowerBound),
                                           leftUpperBound);
                  bestRightParam = std::min(
                      std::max(rightParam, rightLowerBound), rightUpperBound);
                }
                bestDifferencesSquared = differencesSquared;
              }
              const RealNum derivativeWrtLeftParam =
                  LocalFunctions::differenceSquaredDerivativeWRTLeftParam(
                      leftParam, rightParam, eq1Left, eq1Right) +
                  LocalFunctions::differenceSquaredDerivativeWRTLeftParam(
                      leftParam, rightParam, eq2Left, eq2Right);
              const RealNum derivativeWrtRightParam =
                  LocalFunctions::differenceSquaredDerivativeWRTRightParam(
                      leftParam, rightParam, eq1Left, eq1Right) +
                  LocalFunctions::differenceSquaredDerivativeWRTRightParam(
                      leftParam, rightParam, eq2Left, eq2Right);
              const RealNum nextLeftParam =
                  leftParam - (differencesSquared / derivativeWrtLeftParam);
              const RealNum nextRightParam =
                  rightParam - (differencesSquared / derivativeWrtRightParam);
              const RealNum nextLeftParamDifferencesSquared =
                  LocalFunctions::differencesSquaredSum(
                      nextLeftParam, rightParam, eq1Left, eq1Right, eq2Left,
                      eq2Right);
              const RealNum nextRightParamDifferencesSquared =
                  LocalFunctions::differencesSquaredSum(
                      leftParam, nextRightParam, eq1Left, eq1Right, eq2Left,
                      eq2Right);
              if (((nextLeftParam >= leftLowerBound &&
                    nextLeftParam <= leftUpperBound) ||
                   nextRightParam < rightLowerBound ||
                   nextRightParam > rightUpperBound) &&
                  nextLeftParamDifferencesSquared < differencesSquared) {
                leftParam = nextLeftParam;
                differencesSquared = nextLeftParamDifferencesSquared;
              } else if (((nextRightParam >= rightLowerBound &&
                           nextRightParam <= rightUpperBound) ||
                          nextLeftParam < leftLowerBound ||
                          nextLeftParam > leftUpperBound) &&
                         nextRightParamDifferencesSquared <
                             differencesSquared) {
                rightParam = nextRightParam;
                differencesSquared = nextRightParamDifferencesSquared;
              } else {
                break;
              }
            }
          }
          if (sufficientlyClose(eq1Left.valueAt(bestLeftParam),
                                eq1Right.valueAt(bestRightParam)) &&
              sufficientlyClose(eq2Left.valueAt(bestLeftParam),
                                eq2Right.valueAt(bestRightParam))) {
            result.push_back({bestLeftParam, bestRightParam});
          }
        }
      }
    }
  }
  return result;
}

bool isSolutionInfinite(const std::vector<std::pair<RealNum, RealNum>> &input) {
  return input.size() == 3 && isnan(input.front().first) &&
         isnan(input.front().second);
}

std::pair<RealNum, RealNum> solve2VarLinearSystem(
    const RealNum &tValue, const RealNum &uValue,
    const RealNum &firstDerivativeAtParamsWRT_t,
    const RealNum &firstDerivativeAtParamsWRT_u,
    const RealNum &firstFunctionValueAtParams,
    const RealNum &secondDerivativeAtParamsWRT_t,
    const RealNum &secondDerivativeAtParamsWRT_u,
    const RealNum
        &secondFunctionValueAtParams) { // Variable names treat the 2 variables
  // in the system as t and u.
  if (firstFunctionValueAtParams == 0 && secondFunctionValueAtParams == 0) {
    return {tValue, uValue};
  }
  const RealNum firstConstantAddition =
      firstFunctionValueAtParams - (firstDerivativeAtParamsWRT_t * tValue) -
      (firstDerivativeAtParamsWRT_u * uValue);
  const RealNum secondConstantAddition =
      secondFunctionValueAtParams - (secondDerivativeAtParamsWRT_t * tValue) -
      (secondDerivativeAtParamsWRT_u * uValue);
  const RealNum newTValue =
      ((firstConstantAddition * secondDerivativeAtParamsWRT_u) -
       (firstDerivativeAtParamsWRT_u * secondConstantAddition)) /
      ((firstDerivativeAtParamsWRT_u * secondDerivativeAtParamsWRT_t) -
       (firstDerivativeAtParamsWRT_t * secondDerivativeAtParamsWRT_u));
  if (std::isnan(newTValue) || std::isinf(newTValue)) {
    return {tValue, uValue};
  }
  const RealNum newUValue =
      (-1.0) * (std::fabs(firstDerivativeAtParamsWRT_u) >
                        std::fabs(secondDerivativeAtParamsWRT_u)
                    ? (((firstDerivativeAtParamsWRT_t * newTValue) +
                        firstConstantAddition) /
                       firstDerivativeAtParamsWRT_u)
                    : (((secondDerivativeAtParamsWRT_t * newTValue) +
                        secondConstantAddition) /
                       secondDerivativeAtParamsWRT_u));
  if (std::isnan(newUValue) || std::isinf(newUValue)) {
    return {tValue, uValue};
  }
  return {newTValue, newUValue};
}
} // namespace bezier_geometry
