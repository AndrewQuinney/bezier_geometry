#ifndef PolynomialFunction_HPP_
#define PolynomialFunction_HPP_

#include <array>
#include <cmath>
#include <vector>

#include "BezierGeometryGlobal.hpp"
#include "GeometryUtil.hpp"
#include "StaticVector.hpp"

namespace bezier_geometry {
static constexpr RealNum MAX_ROOT_MAGNITUDE = 1000000000; // 1 billion.

std::string nTs(const RealNum &input);

std::string iTs(const int input);

/**
 * \brief A single variable polynomial function.
 *
 * For a given number of constant coefficients (a, b, c, d, e), a polynomial
 * function has the form:
 *
 * at^4+bt^3+ct^2+dt+e
 *
 * For a single variable 't'.
 */
template <std::size_t COEFF_COUNT> class PolynomialFunction {
public:
  typedef PolynomialFunction<(COEFF_COUNT > 1) ? (COEFF_COUNT - 1) : 1>
      DerivativeType; /**< The type that the derivative of this function has. It
                         is another function with a degree one lower, unless
                         this is degree 0. */
  using RootsType = StaticVector<
      RealNum, COEFF_COUNT - 1>; /**< The type necessary to contain the roots of
                                    this function. The function may not have as
                                    many roots indicated in the structure - this
                                    is a sufficiently large set to contain all
                                    possible roots for a given degree. */
  static constexpr std::size_t coefficientCount =
      COEFF_COUNT; /**< A constant expression for the number of coefficeints in
                      this type. */

  /**
   * \brief Creates a new function.
   *
   * A polynomial function consists of a set of coefficients, their position in
   * the set indicating their parameter degree.
   */
  PolynomialFunction(
      const std::array<RealNum, COEFF_COUNT>
          &coeffs /**< The coefficients. The first entry is the degree 0
                     coefficient, the second is degree 1, etc. */
      )
      : coeffs(coeffs) {
    static_assert(COEFF_COUNT > 0,
                  "Coefficient counts must be greater than 0.");
  }

  /**
   * \brief Evaluates this function for a parameter value.
   *
   * @return The value of this function at the input parameter.
   */
  inline RealNum valueAt(const RealNum &input /**< The parameter value to
                                                 evaluate this function at. */
  ) const {
    return _valueAt(input);
  }

  /**
   * \brief Calculates and returns the derivative of this function.
   *
   * @return This function's derivative.
   */
  inline DerivativeType getDerivative() const { return _getDerivative(); }

  /**
   * \brief Gets the coefficient value for a specified degree.
   *
   * @return The coefficient value for the degree. Possibly 0.
   */
  template <std::size_t COEFF_IDX>
  inline typename std::enable_if<(COEFF_IDX >= 0) && (COEFF_IDX < COEFF_COUNT),
                                 const RealNum &>::type
  getCoefficient() const {
    return coeffs[COEFF_IDX];
  }

  /**
   * \brief The the roots of this function.
   *
   * The roots of a function are where it equals 0. This calculates all roots
   * within the specified range.
   *
   * @return A set of parameter values representing the roots of this function
   * within the input interval.
   */
  RootsType
  getRoots(const RealNum &lowerBound, /**< The minimum parameter value that
                                         roots are to be calculated for. */
           const RealNum &upperBound /**< The maximum parameter value that roots
                                        are to be calculated for. */
  ) const {
    if (lowerBound > upperBound) {
      throw std::string(
          "Attempted to get roots over an interval with negative size.");
    }
    if (preciseIsConstant()) {
      throw std::string("Attempted to get roots on a constant function: ") +
          toString();
    }
    RootsType result(getRootsInternal(lowerBound, upperBound));
    if (result.empty()) {
      return result;
    }
    std::sort(result.begin(), result.end());
    {
      typename RootsType::iterator i = result.begin();
      while (i != result.end()) {
        if (*i > lowerBound || sufficientlyClose(*i, lowerBound)) {
          break;
        }
        i++;
      }
      if (i != result.begin()) {
        i = result.erase(result.begin(), i);
      }
      while (i != result.end() && *i < lowerBound) {
        *i = lowerBound;
        i++;
      }
      while (i != result.end() && *i <= upperBound) {
        i++;
      }
      if (i != result.end()) {
        if (sufficientlyClose(*i, upperBound)) {
          *i = upperBound;
          i++;
        }
        result.erase(i, result.end());
      }
    }
    {
      typename RootsType::iterator i = result.begin();
      while (i != result.end()) {
        typename RootsType::iterator j = i + 1;
        for (; j != result.end() && *i == *j; j++) {
        }
        if (j == i + 1) {
          i++;
        } else {
          i = result.erase(i + 1, j);
        }
      }
    }
    if (!result.empty()) {
      for (typename RootsType::iterator i = result.begin() + 1;
           i != result.end();) {
        if (sufficientlyClose(*i, *(i - 1))) {
          const RealNum currentValue = fabs(valueAt(*i));
          const RealNum previousValue = fabs(valueAt(*(i - 1)));
          const RealNum halfWayTestValue =
              fabs(valueAt(((*i) + (*(i - 1))) / 2.0));
          if (halfWayTestValue <= currentValue ||
              halfWayTestValue <= previousValue) {
            if (currentValue < previousValue) {
              i = (result.erase(i - 1) + 1);
            } else {
              i = result.erase(i);
            }
            continue;
          }
        }
        i++;
      }
    }
    if (result.empty()) {
      if (sufficientlySmall(valueAt(lowerBound))) {
        result.push_back(lowerBound);
      } else if (sufficientlySmall(valueAt(upperBound))) {
        result.push_back(upperBound);
      }
    }
    return result;
  }

  /**
   * \brief Get a string representation of this function.
   *
   * @return A string representation of this function in the form
   * ax^2[+-]bx[+-]c.
   */
  std::string toString() const { return _toString(); }

  /**
   * \brief Get the minimum and maximum values of this function over an
   * interval.
   */
  void minMaxValues(
      const RealNum &lowerBound, /**< The start of the interval to calculate
                                    min/max values for. */
      const RealNum &upperBound, /**< The end of the interval to calculate
                                    min/max values for. */
      RealNum &resultMin, /**< Destination for the minimum value over the input
                             interval. */
      RealNum &resultMinPara, /**< Destination for the parameter for the minimum
                                 value over the input interval. */
      RealNum &resultMax,     /**< Destination for the maximum value over the
                                 interval. */
      RealNum &resultMaxPara  /**< Destination for the parameter for the maximum
                                 value over the input interval. */
  ) const {
    if (lowerBound > upperBound) {
      throw std::string("Input interval has a negative size.");
    }
    _minMaxValues(lowerBound, upperBound, resultMin, resultMinPara, resultMax,
                  resultMaxPara);
  }

  /**
   * \brief Determine if this is a constant function on the specified interval.
   *
   * Any degree 0 (only one coefficient) function is constant, but others may
   * also be considered constant over an interval if its derivatives are also
   * considered to be both sufficiently small and constant as well. \see
   * SufficientlyClose
   *
   * @return True if this function is sufficiently constant over the input
   * interval.
   */
  inline bool isConstant(
      const RealNum
          &lowerBound, /**< The start of the parameter interval to check. */
      const RealNum
          &upperBound /**< The end of the parameter interval to check. */
  ) const {
    return _isConstant(lowerBound, upperBound);
  }

  /**
   * \brief Get the result of subtracting a function from this function.
   *
   * Polynomial subtraction is done by getting the difference between all
   * coefficients for corresponding degrees and using those for a new function.
   *
   * @return The function that results from subtracting the input function from
   * this.
   */
  template <std::size_t INPUT_COEFF_COUNT>
  PolynomialFunction<(COEFF_COUNT > INPUT_COEFF_COUNT) ? COEFF_COUNT
                                                       : INPUT_COEFF_COUNT>
  subtract(const PolynomialFunction<INPUT_COEFF_COUNT>
               &input /**< The fucntion to subtract from this. */
  ) const {
    std::array<RealNum, (COEFF_COUNT > INPUT_COEFF_COUNT) ? COEFF_COUNT
                                                          : INPUT_COEFF_COUNT>
        subtractedCoeffs;
    populateSubtractResultCoeffs<0>(input, subtractedCoeffs);
    return subtractedCoeffs;
  }

  /**
   * \brief Get the degree of this function, excluding sufficiently
   * inconsequential coefficients.
   *
   * While a function may have higher degree coefficients, they may be so small
   * relative to the values of this function in a given range that it would be
   * inappropriate to consider it to be of that degree. This makes use of the
   * 'constant' logic in this class to determine if a degree is 'effective'.
   * \see PolynomialFunction:isConstant
   *
   * @return The effective degree of this function; i.e. the highest degree
   * without an inconsequentially small coefficient.
   */
  inline std::size_t effectiveDegree(const RealNum &lowerBound,
                                     const RealNum &upperBound) const {
    return _effectiveDegree(lowerBound, upperBound);
  }

  /**
   * \brief Tests if a function is equal to another.
   *
   * A function is equal to another if they have the same number of coefficients
   * and the coefficients are equal. This is a higher requirement than the
   * 'sufficiently close' conditions. \see GeometryUtil
   *
   * @return True if this function and the input function are equal.
   */
  template <std::size_t INPUT_COEFF_COUNT>
  inline bool
  operator==(const PolynomialFunction<INPUT_COEFF_COUNT> &input) const {
    return coeffsEqual<0>(input);
  }

private:
  friend class PolynomialFunction<COEFF_COUNT + 1>;

  std::array<RealNum, COEFF_COUNT> coeffs;

  bool testRoot(const DerivativeType &derivative,
                const RealNum &testParameter) const {
    if (fabs(testParameter) > MAX_ROOT_MAGNITUDE) {
      return false;
    }
    const RealNum testValue = valueAt(testParameter);
    return sufficientlySmall(testValue) ||
           sufficientlySmall(testValue / derivative.valueAt(testParameter));
  }

  void minMaxValuesForBounds(const RealNum &lowerBound,
                             const RealNum &upperBound, RealNum &resultMin,
                             RealNum &resultMinPara, RealNum &resultMax,
                             RealNum &resultMaxPara) const {
    const RealNum lowerBoundVal = valueAt(lowerBound);
    const RealNum upperBoundVal = valueAt(upperBound);
    if (lowerBoundVal < upperBoundVal) {
      resultMin = lowerBoundVal;
      resultMinPara = lowerBound;
      resultMax = upperBoundVal;
      resultMaxPara = upperBound;
    } else if (upperBoundVal < lowerBoundVal) {
      resultMin = upperBoundVal;
      resultMinPara = upperBound;
      resultMax = lowerBoundVal;
      resultMaxPara = lowerBound;
    } else { // To maintain consistency with old logic.
      resultMin = lowerBoundVal;
      resultMinPara = lowerBound;
      resultMax = lowerBoundVal;
      resultMaxPara = lowerBound;
    }
  }

  template <std::size_t C = COEFF_COUNT>
  inline typename std::enable_if<(C == 1), std::size_t>::type
  _effectiveDegree(__attribute__((unused)) const RealNum &lowerBound,
                   __attribute__((unused)) const RealNum &upperBound) const {
    return 0;
  }

  template <std::size_t C = COEFF_COUNT>
  inline typename std::enable_if<(C > 1), std::size_t>::type
  _effectiveDegree(const RealNum &lowerBound, const RealNum &upperBound) const {
    return isConstant(lowerBound, upperBound)
               ? 0
               : (1 + getDerivative().effectiveDegree(lowerBound, upperBound));
  }

  inline RealNum
  _valueAt(__attribute__((unused)) const RealNum &input,
           std::integral_constant<std::size_t, COEFF_COUNT>) const {
    return 0;
  }

  template <std::size_t CURRENT_COEFF_IDX = 0>
  inline RealNum
  _valueAt(const RealNum &input,
           std::integral_constant<std::size_t, CURRENT_COEFF_IDX> =
               std::integral_constant<std::size_t, 0>()) const {
    return coeffs[CURRENT_COEFF_IDX] +
           (input *
            _valueAt(
                input,
                std::integral_constant<std::size_t, CURRENT_COEFF_IDX + 1>()));
  }

  inline void populateDerivativeCoeffs(
      __attribute__((unused))
      std::array<RealNum, DerivativeType::coefficientCount> &derivativeCoeffs,
      std::integral_constant<std::size_t, DerivativeType::coefficientCount>)
      const {}

  template <std::size_t CURRENT_COEFF_IDX = 0>
  inline void populateDerivativeCoeffs(
      std::array<RealNum, DerivativeType::coefficientCount> &derivativeCoeffs,
      std::integral_constant<std::size_t, CURRENT_COEFF_IDX> =
          std::integral_constant<std::size_t, 0>()) const {
    derivativeCoeffs[CURRENT_COEFF_IDX] =
        coeffs[CURRENT_COEFF_IDX + 1] * (CURRENT_COEFF_IDX + 1);
    populateDerivativeCoeffs(
        derivativeCoeffs,
        std::integral_constant<std::size_t, CURRENT_COEFF_IDX + 1>());
  }

  template <unsigned int ITERATIONS_REMAINING>
  inline typename std::enable_if<ITERATIONS_REMAINING == 0, RealNum>::type
  newtonsMethod(const RealNum &initialGuess,
                __attribute__((unused))
                const DerivativeType &derivative) const {
    return initialGuess;
  }

  template <unsigned int ITERATIONS_REMAINING>
  inline typename std::enable_if<(ITERATIONS_REMAINING > 0), RealNum>::type
  newtonsMethod(const RealNum &initialGuess,
                const DerivativeType &derivative) const {
    const RealNum nextGuess = initialGuess - (valueAt(initialGuess) /
                                              derivative.valueAt(initialGuess));
    if (std::isinf(nextGuess) || isnan(nextGuess) ||
        nextGuess == initialGuess) {
      return initialGuess;
    } else {
      return newtonsMethod<ITERATIONS_REMAINING - 1>(nextGuess, derivative);
    }
  }

  template <std::size_t C = COEFF_COUNT>
  typename std::enable_if<C == 2, RootsType>::type
  getRootsInternal(__attribute__((unused)) const RealNum &lowerBound,
                   __attribute__((unused)) const RealNum &upperBound) const {
    const RealNum root = (-1.0) * valueAt(0) / getDerivative().valueAt(0);
    if (isnan(root) || std::isinf(root)) {
      throw std::string("Attempted to get roots on a constant function: ") +
          toString();
    }
    RootsType result;
    result.push_back((-1.0) * valueAt(0) / getDerivative().valueAt(0));
    return result;
  }

  template <std::size_t C = COEFF_COUNT>
  typename std::enable_if<C == 3, RootsType>::type
  getRootsInternal(const RealNum &lowerBound, const RealNum &upperBound) const {
    RootsType result;
    const RealNum discriminant =
        pow(getCoefficient<1>(), 2) -
        (4 * getCoefficient<2>() * getCoefficient<0>());
    const RealNum negativeB = (-1.0) * getCoefficient<1>();
    const RealNum _2a = 2.0 * getCoefficient<2>();
    const DerivativeType derivative(getDerivative());
    // Results of the quadratic equations are refined with Newton's method to
    // improve their accuracy.
    if (discriminant <= 0) {
      const RealNum calculatedRoot = negativeB / _2a;
      if (isnan(calculatedRoot)) {
        throw std::string("Attempted to get roots on a constant function: ") +
            toString();
      } else if (std::isinf(calculatedRoot)) {
        typename PolynomialFunction<2>::RootsType deg2Roots(
            PolynomialFunction<2>({getCoefficient<0>(), getCoefficient<1>()})
                .getRootsInternal(lowerBound, upperBound));
        for (const RealNum &current :
             PolynomialFunction<2>({getCoefficient<0>(), getCoefficient<1>()})
                 .getRootsInternal(lowerBound, upperBound)) {
          result.push_back(current);
        }
      } else {
        RealNum refinedRoot = newtonsMethod<3>(calculatedRoot, derivative);
        refinedRoot = fabs(valueAt(calculatedRoot)) < fabs(valueAt(refinedRoot))
                          ? calculatedRoot
                          : refinedRoot;
        if (testRoot(derivative, refinedRoot)) {
          result.push_back(refinedRoot);
        }
      }
    } else {
      const RealNum discriminantSqrt = sqrt(discriminant);
      const RealNum root1 = (negativeB + discriminantSqrt) / _2a;
      const RealNum root2 = (negativeB - discriminantSqrt) / _2a;
      if (std::isinf(root1) || std::isinf(root2)) {
        for (const RealNum &current :
             PolynomialFunction<2>({getCoefficient<0>(), getCoefficient<1>()})
                 .getRootsInternal(lowerBound, upperBound)) {
          result.push_back(current);
        }
      } else if (isnan(root1) || isnan(root2)) {
        throw std::string("Attempted to get roots on a constant function: ") +
            toString();
      } else {
        result.push_back(newtonsMethod<3>(root1, derivative));
        result.push_back(newtonsMethod<3>(root2, derivative));
      }
    }
    return result;
  }

  template <std::size_t C = COEFF_COUNT>
  typename std::enable_if<(C > 3), RootsType>::type
  getRootsInternal(const RealNum &lowerBound, const RealNum &upperBound) const {
    RootsType result;
    const DerivativeType derivative(getDerivative());
    RealNum root = std::numeric_limits<RealNum>::quiet_NaN();
    for (const RealNum &currentCoefficient : {0.0, 0.33, 0.66, 1.0}) {
      RealNum currentGuess = newtonsMethod<20>(
          lowerBound + ((upperBound - lowerBound) * currentCoefficient),
          derivative);
      {
        RealNum smallestValue = fabs(valueAt(currentGuess));
        for (;;) {
          const RealNum nextGuess = newtonsMethod<3>(currentGuess, derivative);
          const RealNum nextGuessVal = fabs(valueAt(nextGuess));
          if (nextGuessVal < smallestValue) {
            currentGuess = nextGuess;
            smallestValue = nextGuessVal;
          } else {
            break;
          }
        }
      }
      if (fuzzyEquals(
              derivative.valueAt(currentGuess),
              0)) { // Can get accepted roots at critical points, even though
                    // better approximations exist, so take action here.
        if (testRoot(derivative, currentGuess)) {
          const RealNum secondDerivativeVal(
              derivative.getDerivative().valueAt(currentGuess));
          const RealNum functionVal = valueAt(currentGuess);
          if ((functionVal > 0) ==
              (secondDerivativeVal <
               0)) { // A parabola, centered on the y-axis, whose value,
                     // derivative, and second derivative matches mine for the
                     // 0 parameter.
            const RealNum deg2Coeff = secondDerivativeVal / 2.0;
            const RealNum deg0Coeff = functionVal;
            const RealNum parabolaRoot =
                sqrt(-4.0 * deg2Coeff * deg0Coeff) / (2.0 * deg2Coeff);
            RealNum testRoot = (fabs(valueAt(currentGuess + parabolaRoot)) <
                                fabs(valueAt(currentGuess - parabolaRoot)))
                                   ? (currentGuess + parabolaRoot)
                                   : (currentGuess - parabolaRoot);
            RealNum testRootValue = fabs(valueAt(testRoot));
            for (;;) {
              const RealNum tempRoot = newtonsMethod<3>(testRoot, derivative);
              const RealNum tempVal = fabs(valueAt(tempRoot));
              if (tempVal < testRootValue) {
                testRoot = tempRoot;
                testRootValue = tempVal;
              } else {
                break;
              }
            }
            if (testRootValue < fabs(valueAt(currentGuess))) {
              currentGuess = testRoot;
            }
          }
        }
      } else { // Newton's method converges slowly on roots at critical points
               // - find the closest critical point and see if it's a better
               // root.
        const typename DerivativeType::DerivativeType secondDerivative(
            derivative.getDerivative());
        RealNum testParam = currentGuess;
        for (int i = 0; i < 5; i++) {
          const RealNum secondDerivativeVal =
              secondDerivative.valueAt(testParam);
          const RealNum derivativeVal = derivative.valueAt(testParam);
          const RealNum nextParam =
              testParam - (derivativeVal / secondDerivativeVal);
          if (isnan(nextParam) || std::isinf(nextParam) ||
              nextParam == testParam) {
            break;
          }
          testParam = nextParam;
        }
        if (fabs(valueAt(testParam)) < fabs(valueAt(currentGuess))) {
          currentGuess = testParam;
        }
      }
      if (testRoot(derivative, currentGuess)) {
        if (currentGuess != lowerBound &&
            sufficientlyClose(currentGuess, lowerBound) &&
            fabs(valueAt(lowerBound)) < fabs(valueAt(currentGuess))) {
          currentGuess = lowerBound;
        } else if (currentGuess != upperBound &&
                   sufficientlyClose(currentGuess, upperBound) &&
                   fabs(valueAt(upperBound)) < fabs(valueAt(currentGuess))) {
          currentGuess = upperBound;
        }
        root = currentGuess;
        result.push_back(root);
        break;
      }
    }
    if (!isnan(root)) {
      std::array<RealNum, COEFF_COUNT - 1> divisionResultCoeffs;
      if (fuzzyEquals(root, 0)) {
        populateZeroRootDivisionResult(divisionResultCoeffs);
      } else {
        // fill(divisionResultCoeffs, 0);
        divisionResultCoeffs[COEFF_COUNT - 2] = coeffs[COEFF_COUNT - 1];
        populateRootDivisionResult(divisionResultCoeffs, root);
      }
      const std::size_t lastIdx = result.size();
      {
        const PolynomialFunction<COEFF_COUNT - 1> divisionResult(
            divisionResultCoeffs);
        if (!divisionResult.preciseIsConstant()) {
          for (const RealNum &current :
               divisionResult.getRootsInternal(lowerBound, upperBound)) {
            result.push_back(current);
          }
        }
      }
      typename RootsType::iterator currentRoot = result.begin();
      std::advance(currentRoot, lastIdx);
      for (; currentRoot != result.end();
           currentRoot++) { // If dividing the function caused an error
                            // resulting in invalid or inaccurate roots, refine
                            // them here.
        const RealNum refinedRoot = newtonsMethod<3>(*currentRoot, derivative);
        const RealNum refinedRootVal = fabs(valueAt(refinedRoot));
        const RealNum recursiveRootVal = fabs(valueAt(*currentRoot));
        if (refinedRootVal < recursiveRootVal &&
            fabs(valueAt(((*currentRoot) + refinedRoot) / 2.0)) <
                recursiveRootVal) {
          (*currentRoot) = refinedRoot;
        }
      }
    }
    return result;
  }

  template <std::size_t CURRENT_COEFF_IDX>
  inline typename std::enable_if<(CURRENT_COEFF_IDX == 0)>::type
  populateRootDivisionResult(
      __attribute__((unused))
      std::array<RealNum, COEFF_COUNT - 1> &divisionResultCoeffs,
      __attribute__((unused)) const RealNum &root) const {}

  template <std::size_t CURRENT_COEFF_IDX = COEFF_COUNT - 2>
  inline typename std::enable_if<(CURRENT_COEFF_IDX > 0)>::type
  populateRootDivisionResult(
      std::array<RealNum, COEFF_COUNT - 1> &divisionResultCoeffs,
      const RealNum &root) const { // Polynomial 'synthetic division'.
    divisionResultCoeffs[CURRENT_COEFF_IDX - 1] =
        coeffs[CURRENT_COEFF_IDX] +
        (divisionResultCoeffs[CURRENT_COEFF_IDX] * root);
    populateRootDivisionResult<CURRENT_COEFF_IDX - 1>(divisionResultCoeffs,
                                                      root);
  }

  template <std::size_t CURRENT_COEFF_IDX>
  inline typename std::enable_if<(CURRENT_COEFF_IDX == COEFF_COUNT - 1)>::type
  populateZeroRootDivisionResult(
      __attribute__((unused))
      std::array<RealNum, COEFF_COUNT - 1> &divisionResultCoeffs) const {}

  template <std::size_t CURRENT_COEFF_IDX = 0>
  inline typename std::enable_if<(CURRENT_COEFF_IDX < COEFF_COUNT - 1)>::type
  populateZeroRootDivisionResult(
      std::array<RealNum, COEFF_COUNT - 1> &divisionResultCoeffs) const {
    divisionResultCoeffs[CURRENT_COEFF_IDX] = coeffs[CURRENT_COEFF_IDX + 1];
    populateZeroRootDivisionResult<CURRENT_COEFF_IDX + 1>(divisionResultCoeffs);
  }

  template <std::size_t C = COEFF_COUNT>
  inline typename std::enable_if<(C < 2), DerivativeType>::type
  _getDerivative() const {
    return DerivativeType({0});
  }

  template <std::size_t C = COEFF_COUNT>
  inline typename std::enable_if<(C > 1), DerivativeType>::type
  _getDerivative() const {
    std::array<RealNum, DerivativeType::coefficientCount> derivativeCoeffs;
    populateDerivativeCoeffs(derivativeCoeffs);
    return DerivativeType(derivativeCoeffs);
  }

  template <std::size_t C = COEFF_COUNT>
  inline typename std::enable_if<(C == 1)>::type _minMaxValues(
      const RealNum &lowerBound,
      __attribute__((unused)) const RealNum &upperBound, RealNum &resultMin,
      RealNum &resultMinPara, RealNum &resultMax,
      RealNum &resultMaxPara) const { // This probably never gets generated.
    resultMin = getCoefficient<0>();
    resultMinPara = lowerBound;
    resultMax = getCoefficient<0>();
    resultMaxPara = lowerBound;
  }

  template <std::size_t C = COEFF_COUNT>
  inline typename std::enable_if<(C == 2)>::type
  _minMaxValues(const RealNum &lowerBound, const RealNum &upperBound,
                RealNum &resultMin, RealNum &resultMinPara, RealNum &resultMax,
                RealNum &resultMaxPara) const {
    minMaxValuesForBounds(lowerBound, upperBound, resultMin, resultMinPara,
                          resultMax, resultMaxPara);
  }

  template <std::size_t C = COEFF_COUNT>
  inline typename std::enable_if<(C > 2)>::type
  _minMaxValues(const RealNum &lowerBound, const RealNum &upperBound,
                RealNum &resultMin, RealNum &resultMinPara, RealNum &resultMax,
                RealNum &resultMaxPara) const {
    minMaxValuesForBounds(lowerBound, upperBound, resultMin, resultMinPara,
                          resultMax, resultMaxPara);
    const DerivativeType derivative(getDerivative());
    if (!derivative.preciseIsConstant()) {
      for (const RealNum &currentCrit :
           derivative.getRoots(lowerBound, upperBound)) {
        const RealNum critValue = valueAt(currentCrit);
        if (critValue < resultMin) {
          resultMin = critValue;
          resultMinPara = currentCrit;
        } else if (critValue > resultMax) {
          resultMax = critValue;
          resultMaxPara = currentCrit;
        }
      }
    }
  }

  template <std::size_t C = COEFF_COUNT>
  inline typename std::enable_if<(C == 1), bool>::type
  _isConstant(__attribute__((unused)) const RealNum &lowerBound,
              __attribute__((unused)) const RealNum &upperBound) const {
    return true;
  }

  template <std::size_t C = COEFF_COUNT>
  inline typename std::enable_if<(C > 1), bool>::type
  _isConstant(const RealNum &lowerBound, const RealNum &upperBound) const {
    if (sufficientlyClose(valueAt(lowerBound), valueAt(upperBound))) {
      const DerivativeType derivative(getDerivative());
      return sufficientlySmall(derivative.valueAt(upperBound)) &&
             derivative.isConstant(lowerBound, upperBound);
    }
    return false;
  }

  template <std::size_t CURRENT_IDX, std::size_t RESULT_SIZE,
            std::size_t INPUT_SIZE>
  inline typename std::enable_if<(CURRENT_IDX < COEFF_COUNT &&
                                  CURRENT_IDX < INPUT_SIZE)>::type
  populateSubtractResultCoeffs(
      const PolynomialFunction<INPUT_SIZE> &input,
      std::array<RealNum, RESULT_SIZE> &outputCoeffs) const {
    outputCoeffs[CURRENT_IDX] = getCoefficient<CURRENT_IDX>() -
                                input.template getCoefficient<CURRENT_IDX>();
    populateSubtractResultCoeffs<CURRENT_IDX + 1>(input, outputCoeffs);
  }

  template <std::size_t CURRENT_IDX, std::size_t RESULT_SIZE,
            std::size_t INPUT_SIZE>
  inline typename std::enable_if<(CURRENT_IDX < COEFF_COUNT &&
                                  CURRENT_IDX >= INPUT_SIZE)>::type
  populateSubtractResultCoeffs(
      const PolynomialFunction<INPUT_SIZE> &input,
      std::array<RealNum, RESULT_SIZE> &outputCoeffs) const {
    outputCoeffs[CURRENT_IDX] = getCoefficient<CURRENT_IDX>();
    populateSubtractResultCoeffs<CURRENT_IDX + 1>(input, outputCoeffs);
  }

  template <std::size_t CURRENT_IDX, std::size_t RESULT_SIZE,
            std::size_t INPUT_SIZE>
  inline typename std::enable_if<(CURRENT_IDX >= COEFF_COUNT &&
                                  CURRENT_IDX < INPUT_SIZE)>::type
  populateSubtractResultCoeffs(
      const PolynomialFunction<INPUT_SIZE> &input,
      std::array<RealNum, RESULT_SIZE> &outputCoeffs) const {
    outputCoeffs[CURRENT_IDX] =
        (-1.0) * (input.template getCoefficient<CURRENT_IDX>());
    populateSubtractResultCoeffs<CURRENT_IDX + 1>(input, outputCoeffs);
  }

  template <std::size_t CURRENT_IDX, std::size_t RESULT_SIZE,
            std::size_t INPUT_SIZE>
  inline typename std::enable_if<(CURRENT_IDX >= COEFF_COUNT &&
                                  CURRENT_IDX >= INPUT_SIZE)>::type
  populateSubtractResultCoeffs(
      __attribute__((unused)) const PolynomialFunction<INPUT_SIZE> &input,
      __attribute__((unused))
      std::array<RealNum, RESULT_SIZE> &outputCoeffs) const {}

  template <std::size_t CURRENT_IDX, std::size_t INPUT_SIZE>
  inline typename std::enable_if<
      (CURRENT_IDX < COEFF_COUNT && CURRENT_IDX < INPUT_SIZE), bool>::type
  coeffsEqual(const PolynomialFunction<INPUT_SIZE> &input) const {
    return fuzzyEquals(getCoefficient<CURRENT_IDX>(),
                       input.template getCoefficient<CURRENT_IDX>()) &&
           coeffsEqual<CURRENT_IDX + 1>(input);
  }

  template <std::size_t CURRENT_IDX, std::size_t INPUT_SIZE>
  inline typename std::enable_if<
      (CURRENT_IDX < COEFF_COUNT && CURRENT_IDX >= INPUT_SIZE), bool>::type
  coeffsEqual(const PolynomialFunction<INPUT_SIZE> &input) const {
    return fuzzyEquals(getCoefficient<CURRENT_IDX>(), 0) &&
           coeffsEqual<CURRENT_IDX + 1>(input);
  }

  template <std::size_t CURRENT_IDX, std::size_t INPUT_SIZE>
  inline typename std::enable_if<
      (CURRENT_IDX >= COEFF_COUNT && CURRENT_IDX < INPUT_SIZE), bool>::type
  coeffsEqual(const PolynomialFunction<INPUT_SIZE> &input) const {
    return fuzzyEquals(input.template getCoefficient<CURRENT_IDX>(), 0) &&
           coeffsEqual<CURRENT_IDX + 1>(input);
  }

  template <std::size_t CURRENT_IDX, std::size_t INPUT_SIZE>
  inline typename std::enable_if<
      (CURRENT_IDX >= COEFF_COUNT && CURRENT_IDX >= INPUT_SIZE), bool>::type
  coeffsEqual(const PolynomialFunction<INPUT_SIZE> &input) const {
    (void)input;
    return true;
  }

  template <std::size_t CURRENT_IDX = COEFF_COUNT - 1>
  inline typename std::enable_if<(CURRENT_IDX == 0), bool>::type
  preciseIsConstant() const {
    return true;
  }

  template <std::size_t CURRENT_IDX = COEFF_COUNT - 1>
  inline typename std::enable_if<(CURRENT_IDX > 0), bool>::type
  preciseIsConstant() const {
    return fuzzyIsNull(getCoefficient<CURRENT_IDX>()) &&
           preciseIsConstant<CURRENT_IDX - 1>();
  }

  template <std::size_t CURRENT_IDX = 0>
  inline
      typename std::enable_if<(CURRENT_IDX == COEFF_COUNT), std::string>::type
      _toString() const {
    return "";
  }

  template <std::size_t CURRENT_IDX = 0>
  typename std::enable_if<(CURRENT_IDX == 0), std::string>::type
  _toString() const {
    const std::string recursiveResult(_toString<CURRENT_IDX + 1>());
    if (recursiveResult.empty()) {
      return nTs(getCoefficient<CURRENT_IDX>());
    } else if (getCoefficient<CURRENT_IDX>() != 0) {
      return recursiveResult +
             (getCoefficient<CURRENT_IDX>() > 0 ? " + " : " - ") +
             nTs(fabs(getCoefficient<CURRENT_IDX>()));
    } else {
      return recursiveResult;
    }
  }

  template <std::size_t CURRENT_IDX = 0>
  typename std::enable_if<(CURRENT_IDX == 1 && CURRENT_IDX < COEFF_COUNT),
                          std::string>::type
  _toString() const {
    if (getCoefficient<CURRENT_IDX>() != 0) {
      const std::string recursiveResult(_toString<CURRENT_IDX + 1>());
      if (recursiveResult.empty()) {
        return (fabs(getCoefficient<CURRENT_IDX>()) != 1
                    ? nTs(getCoefficient<CURRENT_IDX>())
                    : std::string(getCoefficient<CURRENT_IDX>() > 0 ? ""
                                                                    : "-")) +
               "x";
      } else {
        return recursiveResult +
               std::string(getCoefficient<CURRENT_IDX>() > 0 ? " + " : " - ") +
               (fabs(getCoefficient<CURRENT_IDX>()) != 1
                    ? nTs(fabs(getCoefficient<CURRENT_IDX>()))
                    : std::string("")) +
               "x";
      }
    } else {
      return _toString<CURRENT_IDX + 1>();
    }
  }

  template <std::size_t CURRENT_IDX = 0>
  typename std::enable_if<(CURRENT_IDX > 1 && CURRENT_IDX < COEFF_COUNT),
                          std::string>::type
  _toString() const {
    if (getCoefficient<CURRENT_IDX>() != 0) {
      const std::string recursiveResult(_toString<CURRENT_IDX + 1>());
      if (recursiveResult.empty()) {
        return (fabs(getCoefficient<CURRENT_IDX>()) != 1
                    ? nTs(getCoefficient<CURRENT_IDX>())
                    : std::string(getCoefficient<CURRENT_IDX>() > 0 ? ""
                                                                    : "-")) +
               "x^" + iTs(CURRENT_IDX);
      } else {
        return recursiveResult +
               std::string(getCoefficient<CURRENT_IDX>() > 0 ? " + " : " - ") +
               (fabs(getCoefficient<CURRENT_IDX>()) != 1
                    ? nTs(fabs(getCoefficient<CURRENT_IDX>()))
                    : std::string("")) +
               "x^" + iTs(CURRENT_IDX);
      }
    } else {
      return _toString<CURRENT_IDX + 1>();
    }
  }
};

StaticVector<std::pair<RealNum, RealNum>, 16> solveSystem(
    const PolynomialFunction<3> &eq1Left, const PolynomialFunction<3> &eq1Right,
    const PolynomialFunction<3> &eq2Left, const PolynomialFunction<3> &eq2Right,
    const RealNum &leftLowerBound, const RealNum &leftUpperBound,
    const RealNum &rightLowerBound, const RealNum &rightUpperBound);

bool isSolutionInfinite(
    const StaticVector<std::pair<RealNum, RealNum>, 4> &input);

std::pair<RealNum, RealNum>
solve2VarLinearSystem(const RealNum &tValue, const RealNum &uValue,
                      const RealNum &firstDerivativeAtParamsWRT_t,
                      const RealNum &firstDerivativeAtParamsWRT_u,
                      const RealNum &firstFunctionValueAtParams,
                      const RealNum &secondDerivativeAtParamsWRT_t,
                      const RealNum &secondDerivativeAtParamsWRT_u,
                      const RealNum &secondFunctionValueAtParams);

template <std::size_t COEFF_COUNT>
std::ostream &operator<<(std::ostream &os,
                         const PolynomialFunction<COEFF_COUNT> &input) {
  os << input.toString();
  return os;
}
} // namespace bezier_geometry

#endif
