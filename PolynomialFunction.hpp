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

template <std::size_t COEFF_COUNT> class PolynomialFunction {
public:
  typedef PolynomialFunction<(COEFF_COUNT > 1) ? (COEFF_COUNT - 1) : 1>
      DerivativeType;
  using RootsType = StaticVector<RealNum, COEFF_COUNT - 1>;
  static constexpr std::size_t coefficientCount = COEFF_COUNT;

  PolynomialFunction(const std::array<RealNum, COEFF_COUNT> &coeffs)
      : coeffs(coeffs) {
    static_assert(COEFF_COUNT > 0,
                  "Coefficient counts must be greater than 0.");
  }

  inline RealNum valueAt(const RealNum &input) const { return _valueAt(input); }

  inline DerivativeType getDerivative() const { return _getDerivative(); }

  template <std::size_t COEFF_IDX>
  inline typename std::enable_if<(COEFF_IDX >= 0) && (COEFF_IDX < COEFF_COUNT),
                                 const RealNum &>::type
  getCoefficient() const {
    return coeffs[COEFF_IDX];
  }

  RootsType getRoots(const RealNum &lowerBound,
                     const RealNum &upperBound) const {
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

  std::string toString() const { return _toString(); }

  void minMaxValues(const RealNum &lowerBound, const RealNum &upperBound,
                    RealNum &resultMin, RealNum &resultMinPara,
                    RealNum &resultMax, RealNum &resultMaxPara) const {
    if (lowerBound > upperBound) {
      throw std::string("Input interval has a negative size.");
    }
    _minMaxValues(lowerBound, upperBound, resultMin, resultMinPara, resultMax,
                  resultMaxPara);
  }

  inline bool isConstant(const RealNum &lowerBound,
                         const RealNum &upperBound) const {
    return _isConstant(lowerBound, upperBound);
  }

  template <std::size_t INPUT_COEFF_COUNT>
  PolynomialFunction<(COEFF_COUNT > INPUT_COEFF_COUNT) ? COEFF_COUNT
                                                       : INPUT_COEFF_COUNT>
  subtract(const PolynomialFunction<INPUT_COEFF_COUNT> &input) const {
    std::array<RealNum, (COEFF_COUNT > INPUT_COEFF_COUNT) ? COEFF_COUNT
                                                          : INPUT_COEFF_COUNT>
        subtractedCoeffs;
    populateSubtractResultCoeffs<0>(input, subtractedCoeffs);
    return subtractedCoeffs;
  }

  inline std::size_t effectiveDegree(const RealNum &lowerBound,
                                     const RealNum &upperBound) const {
    return _effectiveDegree(lowerBound, upperBound);
  }

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
