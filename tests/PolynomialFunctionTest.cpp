#include "PolynomialFunction.hpp"
#include "TestUtils.hpp"

namespace {
template <typename... N>
auto numAry(N &&...args)
    -> std::array<bezier_geometry::RealNum, sizeof...(args)> {
  return {static_cast<bezier_geometry::RealNum>(std::forward<N>(args))...};
}

template <std::size_t COUNT>
std::string
printArray(const std::array<bezier_geometry::RealNum, COUNT> &input) {
  std::string result;
  for (const bezier_geometry::RealNum &current : input) {
    result += result.length() > 0 ? ", " : "";
    result += bezier_geometry::nTs(current);
  }
  return result;
}

template <std::size_t COEFF_COUNT, std::size_t ROOT_COUNT>
void testRoots(
    const bezier_geometry::PolynomialFunction<COEFF_COUNT> &testFunction,
    const bezier_geometry::RealNum &lowerBound,
    const bezier_geometry::RealNum &upperBound,
    const std::array<bezier_geometry::RealNum, ROOT_COUNT> &expectedResult) {
  try {
    debug() << "*** BEFORE getRoots - function:" << testFunction
            << "lowerBound:" << lowerBound << "upperBound:" << upperBound
            << std::endl;
    const std::vector<bezier_geometry::RealNum> roots(
        testFunction.getRoots(lowerBound, upperBound));
    {
      struct LocalFunctions {
        static std::string
        printVector(const std::vector<bezier_geometry::RealNum> &input) {
          std::string result;
          for (const bezier_geometry::RealNum &current : input) {
            result += result.length() > 0 ? ", " : "";
            result += bezier_geometry::toString(current);
          }
          return result;
        }
      };
      debug()
          << "*** AFTER getRoots -"
          << "expected:"
          << printArray(
                 expectedResult) // LocalFunctions::printVector(expectedResult)
          << "actual" << LocalFunctions::printVector(roots) << std::endl;
    }
    CHECK(roots.size(), expectedResult.size());
    {
      auto currentExpectedResult = expectedResult.begin();
      for (auto i = roots.begin(); i != roots.end(); i++) {
        CHECK(bezier_geometry::sufficientlyClose(*i, *currentExpectedResult),
              true);
        currentExpectedResult++;
      }
    }
    for (std::vector<bezier_geometry::RealNum>::const_iterator i =
             roots.begin();
         i != roots.end(); i++) {
      if (bezier_geometry::sufficientlyClose(testFunction.valueAt(*i), 0)) {
        continue;
      }
      bezier_geometry::RealNum newtonRoot =
          (*i) -
          (testFunction.valueAt(*i) / testFunction.getDerivative().valueAt(*i));
      CHECK(bezier_geometry::sufficientlyClose(*i, newtonRoot), true);
      CHECK(bezier_geometry::sufficientlySmall(
                testFunction.valueAt(newtonRoot)) ||
                bezier_geometry::sufficientlySmall(
                    testFunction.valueAt(newtonRoot) /
                    testFunction.getDerivative().valueAt(newtonRoot)),
            true);
    }
  } catch (const std::string &msg) {
    debug() << "testRoots error:" << msg << std::endl;
    throw msg;
  }
}

template <std::size_t OUTPUT_SIZE, std::size_t CURRENT_IDX = 0>
typename std::enable_if<(CURRENT_IDX == OUTPUT_SIZE)>::type populateLargeCoeffs(
    const std::array<bezier_geometry::RealNum, OUTPUT_SIZE> &source,
    std::array<bezier_geometry::RealNum, OUTPUT_SIZE> &target) {
  (void)source;
  (void)target;
}

template <std::size_t OUTPUT_SIZE, std::size_t CURRENT_IDX = 0>
typename std::enable_if<(CURRENT_IDX < OUTPUT_SIZE)>::type populateLargeCoeffs(
    const std::array<bezier_geometry::RealNum, OUTPUT_SIZE> &source,
    std::array<bezier_geometry::RealNum, OUTPUT_SIZE> &target) {
  target[CURRENT_IDX] = source[CURRENT_IDX] * 10000.0;
  populateLargeCoeffs<OUTPUT_SIZE, CURRENT_IDX + 1>(source, target);
}

template <std::size_t COEFF_COUNT, std::size_t ROOT_COUNT>
void rootsTest(
    const std::array<bezier_geometry::RealNum, COEFF_COUNT> &coeffs,
    const bezier_geometry::RealNum &lowerBound,
    const bezier_geometry::RealNum &upperBound,
    const std::array<bezier_geometry::RealNum, ROOT_COUNT> &expectedResult) {
  testRoots(bezier_geometry::PolynomialFunction<COEFF_COUNT>(coeffs),
            lowerBound, upperBound, expectedResult);
  std::array<bezier_geometry::RealNum, COEFF_COUNT> largeCoeffs{};
  populateLargeCoeffs(coeffs, largeCoeffs);
  testRoots(bezier_geometry::PolynomialFunction<COEFF_COUNT>(largeCoeffs),
            lowerBound, upperBound, expectedResult);
}

template <std::size_t COEFF_COUNT>
void testMinMax(const std::array<bezier_geometry::RealNum, COEFF_COUNT> &coeffs,
                const bezier_geometry::RealNum &lowerBound,
                const bezier_geometry::RealNum &upperBound,
                const bezier_geometry::RealNum &expectedMinVal,
                const bezier_geometry::RealNum &expectedMinValPara,
                const bezier_geometry::RealNum &expectedMaxVal,
                const bezier_geometry::RealNum &expectedMaxValPara) {
  bezier_geometry::PolynomialFunction<COEFF_COUNT> test(coeffs);
  bezier_geometry::RealNum minVal;
  bezier_geometry::RealNum minPara;
  bezier_geometry::RealNum maxVal;
  bezier_geometry::RealNum maxPara;
  test.minMaxValues(lowerBound, upperBound, minVal, minPara, maxVal, maxPara);
  debug() << "testMinMax -"
          << "function:" << test
          << "lowerBound:" << bezier_geometry::toString(lowerBound)
          << "upperBound:" << bezier_geometry::toString(upperBound)
          << "minVal:" << bezier_geometry::toString(minVal)
          << "minPara:" << bezier_geometry::toString(minPara)
          << "maxVal:" << bezier_geometry::toString(maxVal)
          << "maxPara:" << bezier_geometry::toString(maxPara)
          << "expectedMinVal:" << bezier_geometry::toString(expectedMinVal)
          << "expectedMinValPara:"
          << bezier_geometry::toString(expectedMinValPara)
          << "expectedMaxVal:" << bezier_geometry::toString(expectedMaxVal)
          << "expectedMaxValPara:"
          << bezier_geometry::toString(expectedMaxValPara) << std::endl;
  CHECK(bezier_geometry::sufficientlyClose(minVal, expectedMinVal), true);
  CHECK(bezier_geometry::sufficientlyClose(minPara, expectedMinValPara), true);
  CHECK(bezier_geometry::sufficientlyClose(maxVal, expectedMaxVal), true);
  CHECK(bezier_geometry::sufficientlyClose(maxPara, expectedMaxValPara), true);
}
} // namespace

void degreeTest() {
  TEST_START {
    bezier_geometry::PolynomialFunction<1> test({});
    CHECK(test.effectiveDegree(-1000, 1000), 0);
  }
  {
    bezier_geometry::PolynomialFunction<2> test({5, -2});
    CHECK(test.effectiveDegree(-1000, 1000), 1);
  }
  TEST_END
}

void coefficientTest() {
  TEST_START {
    bezier_geometry::PolynomialFunction<2> test({});
    CHECK(test.getCoefficient<0>(), (bezier_geometry::RealNum)0);
    CHECK(test.getCoefficient<1>(), (bezier_geometry::RealNum)0);
  }
  {
    bezier_geometry::PolynomialFunction<3> test({5, -2});
    CHECK(test.getCoefficient<0>(), (bezier_geometry::RealNum)5);
    CHECK(test.getCoefficient<1>(), (bezier_geometry::RealNum)-2);
    CHECK(test.getCoefficient<2>(), (bezier_geometry::RealNum)0);
  }
  TEST_END
}

void equalsTest() {
  TEST_START
  typedef std::pair<std::pair<bezier_geometry::PolynomialFunction<3>,
                              bezier_geometry::PolynomialFunction<3>>,
                    bool>
      paramEntry;
  const std::vector<paramEntry> paramEntries(
      {{{bezier_geometry::PolynomialFunction<3>({5, -2}),
         bezier_geometry::PolynomialFunction<3>({5, -2})},
        true},
       {{bezier_geometry::PolynomialFunction<3>({5, -2, 0}),
         bezier_geometry::PolynomialFunction<3>({5, -2})},
        true},
       {{bezier_geometry::PolynomialFunction<3>({0, 5, -2}),
         bezier_geometry::PolynomialFunction<3>({5, -2})},
        false}});
  for (const paramEntry &current : paramEntries) {
    CHECK(current.first.first == current.first.second, current.second);
    CHECK(current.first.first == current.first.second,
          current.first.second == current.first.first);
  }
  TEST_END
}

void toStringTest() {
  TEST_START
  CHECK(bezier_geometry::PolynomialFunction<1>({}).toString(),
        std::string("0"));
  CHECK(bezier_geometry::PolynomialFunction<2>({5, -2}).toString(),
        std::string("-2x + 5"));
  CHECK(bezier_geometry::PolynomialFunction<2>({5, -1}).toString(),
        std::string("-x + 5"));
  CHECK(bezier_geometry::PolynomialFunction<2>({5, 1}).toString(),
        std::string("x + 5"));
  CHECK(bezier_geometry::PolynomialFunction<2>({5, 0}).toString(),
        std::string("5"));
  CHECK(bezier_geometry::PolynomialFunction<2>({0, 5}).toString(),
        std::string("5x"));
  CHECK(bezier_geometry::PolynomialFunction<3>({1, 1, 5}).toString(),
        std::string("5x^2 + x + 1"));
  TEST_END
}

void derivativeTest() {
  TEST_START
  typedef bezier_geometry::PolynomialFunction<3> FunctionType;
  typedef std::pair<FunctionType, FunctionType> paramEntry;
  const std::vector<paramEntry> testInputs(
      {{FunctionType({}), FunctionType({0})},
       {FunctionType({5}), FunctionType({0})},
       {FunctionType({0}), FunctionType({0})},
       {FunctionType({5, -2}), FunctionType({-2})},
       {FunctionType({0, -2}), FunctionType({-2})},
       {FunctionType({5, -2, 3}), FunctionType({-2, 6})},
       {FunctionType({0, 0, 3}), FunctionType({0, 6})}});
  for (const paramEntry &current : testInputs) {
    CHECK(current.first.getDerivative(), current.second);
  }
  TEST_END
}

void valueAtTest() {
  TEST_START
  typedef bezier_geometry::PolynomialFunction<3> FunctionType;
  typedef std::pair<FunctionType, std::pair<bezier_geometry::RealNum,
                                            bezier_geometry::RealNum>>
      paramEntry;
  for (const paramEntry &current : std::initializer_list<paramEntry>{
           {FunctionType({5, -2}), {1, 3}},
           {FunctionType({5, -2}), {-1, 7}},
           {FunctionType({}), {1, 0}},
           {FunctionType({}), {0, 0}},
           {FunctionType({}), {-1, 0}},
           {FunctionType({5}), {1, 5}},
           {FunctionType({5}), {0, 5}},
           {FunctionType({5}), {-1, 5}},
           {FunctionType({5, -2, 1}), {1, 4}},
           {FunctionType({5, -2, 1}), {2, 5}},
           {FunctionType({5, -2, 1}), {3, 8}},
           {FunctionType({5, -2, 1}), {-2, 13}},
           {FunctionType({5, -2, 1}), {0.5, 4.25}},
       }) {
    CHECK(current.first.valueAt(current.second.first), current.second.second);
  }
  TEST_END
}

void getRootsTest() {
  TEST_START {
    bezier_geometry::PolynomialFunction<2> test({});
    bool failure = true;
    try {
      test.getRoots(0, 1);
    } catch (const std::string &) {
      failure = false;
    }
    CHECK(failure, false);
  }
  {
    bezier_geometry::PolynomialFunction<2> test({0});
    bool failure = true;
    try {
      test.getRoots(0, 1);
    } catch (const std::string &) {
      failure = false;
    }
    CHECK(failure, false);
  }
  {
    bezier_geometry::PolynomialFunction<2> test({5});
    bool failure = true;
    try {
      test.getRoots(0, 1);
    } catch (const std::string &) {
      failure = false;
    }
    CHECK(failure, false);
  }
  {
    bezier_geometry::PolynomialFunction<2> test({5, -2});
    bool failure = true;
    try {
      test.getRoots(1, 0);
    } catch (const std::string &) {
      failure = false;
    }
    CHECK(failure, false);
  }
  rootsTest(numAry(5, -2), 0, 1,
            numAry()); // Straight line, angling down to the right, intersects
                       // x-axis at 2.5.
  rootsTest(numAry(5, -2), 0, 2.4,
            numAry()); // Straight line, angling down to the right, intersects
                       // x-axis at 2.5.
  rootsTest(numAry(5, -2), 0, 2.5,
            numAry(2.5)); // Straight line, angling down to the right,
                          // intersects x-axis at 2.5.
  rootsTest(numAry(5, -2), -5000, 5000,
            numAry(2.5)); // Straight line, angling down to the right,
                          // intersects x-axis at 2.5.
  rootsTest(
      numAry(0, 0, 1), -1, 1,
      numAry(0)); // Parabola, centered on the y-axis, base right on x-axis.
  rootsTest(
      numAry(1, 0, 1), -1, 1,
      numAry()); // Parabola, centered on the y-axis, base 1 unit above x-axis.
  rootsTest(
      numAry(-1, 0, 1), -1, 1,
      numAry(-1,
             1)); // Parabola, centered on the y-axis, base 1 unit below x-axis.
  rootsTest(numAry(0, -2, 1), -10, 10,
            numAry(0, 2)); // Parabola, centered on 1 unit right of the y-axis,
                           // base 1 unit below x-axis.
  rootsTest(numAry(-99, -2, 1), -8, 10,
            numAry()); // Parabola, centered on 1 unit right of the y-axis,
                       // base 100 units below x-axis.
  rootsTest(numAry(-99, -2, 1), -100, 100,
            numAry(-9, 11)); // Parabola, centered on 1 unit right of the
                             // y-axis, base 100 units below x-axis.
  rootsTest(
      numAry(bezier_geometry::ACCEPTABLE_ERROR_MARGIN / 10001.0, 0,
             1), // Division by 10k offsets the multiplication by 10k below.
      -1, 1,
      numAry(0)); // Parabola, centered on the y-axis, base above x-axis, but
                  // within the acceptable error margin.
  rootsTest(numAry(bezier_geometry::ACCEPTABLE_ERROR_MARGIN, 0, 1), -1, 1,
            numAry()); // Parabola, centered on the y-axis, base above x-axis,
                       // just outside the acceptable error margin.
  rootsTest(
      numAry(0, 0, 1), -1, 0,
      numAry(0)); // Parabola, centered on the y-axis, base right on x-axis.
  rootsTest(
      numAry(0, 0, 1), 0, 1,
      numAry(0)); // Parabola, centered on the y-axis, base right on x-axis.
  rootsTest(
      numAry(0, 0, 1), 0, 0,
      numAry(0)); // Parabola, centered on the y-axis, base right on x-axis.
  {
    const bezier_geometry::RealNum upperBound =
        1 - bezier_geometry::ACCEPTABLE_ERROR_MARGIN +
        (bezier_geometry::ACCEPTABLE_ERROR_MARGIN / 1000);
    rootsTest(numAry(-1, 0, 1), -1, upperBound,
              numAry(-1, upperBound)); // Parabola, centered on the y-axis,
                                       // base 1 unit below x-axis.
  }
  rootsTest(numAry(1000, 0, -100, 1), -2000, 2000,
            numAry(-3.1141594617467580619, 3.2143601647611310668,
                   99.89979929698561989)); // Degree 3 polynomial with 3 roots.
  rootsTest(numAry(-362880, 1026576, -1172700, 723680, -269325, 63273, -9450,
                   870, -45, 1),
            0, 10,
            numAry(1, 2, 3, 4, 5, 6, 7, 8,
                   9)); // Degree 9 polynomial with 9 roots, none of which are
                        // at a critical point.
  rootsTest(numAry(-1655565.7015076563694, 4.6156855760206708214e-10,
                   4120900.0000000018626, -1.8462742445990447366e-9,
                   2.0679515313825691872e-25),
            0, 1, numAry(0.63383639427411597733));
  TEST_END
}

void minMaxValTest() {
  TEST_START {
    bezier_geometry::PolynomialFunction<2> test({5, -2});
    bool failure = true;
    bezier_geometry::RealNum minVal;
    bezier_geometry::RealNum minValPara;
    bezier_geometry::RealNum maxVal;
    bezier_geometry::RealNum maxValPara;
    try {
      test.minMaxValues(1, 0, minVal, minValPara, maxVal, maxValPara);
    } catch (const std::string &) {
      failure = false;
    }
    CHECK(failure, false);
    failure = true;
    try {
      test.minMaxValues(1, 0, minVal, minValPara, maxVal, maxValPara);
    } catch (const std::string &) {
      failure = false;
    }
    CHECK(failure, false);
  }
  testMinMax(numAry(5, -2), 0, 1, 3, 1, 5,
             0); // Straight line, angling down to the right, intersects x-axis
                 // at 2.5.
  testMinMax(numAry(5, -2), 1, 2.5, 0, 2.5, 3,
             1); // Straight line, angling down to the right, intersects x-axis
                 // at 2.5.
  testMinMax(numAry(0, 0, 1), 1, 2, 1, 1, 4,
             2); // Parabola, centered on the y-axis, base right on x-axis.
  testMinMax(numAry(0, 0, 1), -1, 1, 0, 0, 1,
             -1); // Parabola, centered on the y-axis, base right on x-axis.
  testMinMax(numAry(0, 0, -1), -1, 1, -1, -1, 0,
             0); // Parabola, opening downward, centered on the y-axis, base
                 // right on x-axis.
  testMinMax(numAry(0, -2, 1), 0, 1, -1, 1, 0,
             0); // Parabola, centered on 1 unit right of the y-axis, base 1
                 // unit below x-axis.
  testMinMax(numAry(0, -2, 1), 0, 5, -1, 1, 15,
             5); // Parabola, centered on 1 unit right of the y-axis, base 1
                 // unit below x-axis.
  testMinMax(numAry(1000, 0, -100, 1), -2, 90, -147148.148148, 66.666667, 1000,
             0); // Degree 3 polynomial with 3 roots.
  TEST_END
}

void reduceTest() {
  TEST_START
  typedef struct {
    bezier_geometry::PolynomialFunction<3> eq1Left;
    bezier_geometry::PolynomialFunction<3> eq1Right;
    bezier_geometry::PolynomialFunction<3> eq2Left;
    bezier_geometry::PolynomialFunction<3> eq2Right;
    bezier_geometry::RealNum leftLowerBound;
    bezier_geometry::RealNum leftUpperBound;
    bezier_geometry::RealNum rightLowerBound;
    bezier_geometry::RealNum rightUpperBound;
    std::vector<std::pair<bezier_geometry::RealNum, bezier_geometry::RealNum>>
        expectedSolutions;
  } ReduceTestEntry;
  const std::vector<ReduceTestEntry> testEntries(
      {{// Standard case, everything is degree 2.
        bezier_geometry::PolynomialFunction<3>({4, 3, 2}),
        bezier_geometry::PolynomialFunction<3>({-23, 6, 5}),
        bezier_geometry::PolynomialFunction<3>({7, 6, 5}),
        bezier_geometry::PolynomialFunction<3>({4, 3, 2}), -100, 100, -100, 100,
        std::vector<
            std::pair<bezier_geometry::RealNum, bezier_geometry::RealNum>>(
            {{0.746032, -3.13579},
             {-1.89293, -3.0612},
             {-2.13882, 1.91128},
             {1, 2}})},
       {// Standard case, everything is degree 2, restrict result range.
        bezier_geometry::PolynomialFunction<3>({4, 3, 2}),
        bezier_geometry::PolynomialFunction<3>({-23, 6, 5}),
        bezier_geometry::PolynomialFunction<3>({7, 6, 5}),
        bezier_geometry::PolynomialFunction<3>({4, 3, 2}), 1, 1, 2, 2,
        std::vector<
            std::pair<bezier_geometry::RealNum, bezier_geometry::RealNum>>(
            {{1, 2}})},
       {// Standard case, one equation is degree 1.
        bezier_geometry::PolynomialFunction<3>({4, 3, 2}),
        bezier_geometry::PolynomialFunction<3>({-9, 6}),
        bezier_geometry::PolynomialFunction<3>({7, 6, 5}),
        bezier_geometry::PolynomialFunction<3>({-27, 6, 3}), -100, 100, -100,
        100,
        std::vector<
            std::pair<bezier_geometry::RealNum, bezier_geometry::RealNum>>(
            {{-2.92228, 3.55211}, {1, 3}})},
       {// Standard case, one side is all degree 1.
        bezier_geometry::PolynomialFunction<3>({4, 3, 2}),
        bezier_geometry::PolynomialFunction<3>({-9, 6}),
        bezier_geometry::PolynomialFunction<3>({7, 6, 5}),
        bezier_geometry::PolynomialFunction<3>({3, 5}), -100, 100, -100, 100,
        std::vector<
            std::pair<bezier_geometry::RealNum, bezier_geometry::RealNum>>(
            {{-2.05, 2.5425}, {1, 3}})},
       {// All equations are degree 2, left side does not combine/reduce
        // properly.
        bezier_geometry::PolynomialFunction<3>({1, 3, 5}),
        bezier_geometry::PolynomialFunction<3>({3, 2, 1}),
        bezier_geometry::PolynomialFunction<3>({7, 6, 10}),
        bezier_geometry::PolynomialFunction<3>({-25, 5, 4}), -100, 100, -100,
        100,
        std::vector<
            std::pair<bezier_geometry::RealNum, bezier_geometry::RealNum>>(
            {{-2.6, 4}, {-1.95529, -4.5}, {1.35529, -4.5}, {2, 4}})},
       {// All equations are degree 2, both sides do not combine/reduce
        // properly.
        bezier_geometry::PolynomialFunction<3>({1, 3, 5}),
        bezier_geometry::PolynomialFunction<3>({3, 2, 1}),
        bezier_geometry::PolynomialFunction<3>({7, 6, 10}),
        bezier_geometry::PolynomialFunction<3>({-13, 6, 3}), -100, 100, -100,
        100,
        std::vector<
            std::pair<bezier_geometry::RealNum, bezier_geometry::RealNum>>(
            {{2, -6}, {-2.6, -6}, {-2.6, 4}, {2, 4}})},
       {// The system reduces to 0, meaning infinite solutions, but the ranges
        // only overlap at a point.
        bezier_geometry::PolynomialFunction<3>({1, 1}),
        bezier_geometry::PolynomialFunction<3>({2, 1}),
        bezier_geometry::PolynomialFunction<3>({3, 1}),
        bezier_geometry::PolynomialFunction<3>({4, 1}), 0, 1, 0, 1,
        std::vector<
            std::pair<bezier_geometry::RealNum, bezier_geometry::RealNum>>(
            {{1, 0}})}});
  for (const ReduceTestEntry &current : testEntries) {
    std::vector<std::pair<bezier_geometry::RealNum, bezier_geometry::RealNum>>
        result = bezier_geometry::solveSystem(
            current.eq1Left, current.eq1Right, current.eq2Left,
            current.eq2Right, current.leftLowerBound, current.leftUpperBound,
            current.rightLowerBound, current.rightUpperBound);
    for (const std::pair<bezier_geometry::RealNum, bezier_geometry::RealNum>
             &currentExpected : current.expectedSolutions) {
      bool resultFound = false;
      for (const std::pair<bezier_geometry::RealNum, bezier_geometry::RealNum>
               &currentActualSolution : result) {
        debug() << "^"
                << "currentActualSolution.first:"
                << bezier_geometry::toString(currentActualSolution.first)
                << "currentExpected.first:"
                << bezier_geometry::toString(currentExpected.first)
                << "currentActualSolution.second:"
                << bezier_geometry::toString(currentActualSolution.second)
                << "currentExpected.second:"
                << bezier_geometry::toString(currentExpected.second)
                << std::endl;
        if (bezier_geometry::sufficientlyClose(currentActualSolution.first,
                                               currentExpected.first) &&
            bezier_geometry::sufficientlyClose(currentActualSolution.second,
                                               currentExpected.second)) {
          resultFound = true;
          break;
        }
      }
      CHECK(resultFound, true);
    }
    for (const std::pair<bezier_geometry::RealNum, bezier_geometry::RealNum>
             &currentResult : result) {
      CHECK(bezier_geometry::sufficientlyClose(
                current.eq1Left.valueAt(currentResult.first),
                current.eq1Right.valueAt(currentResult.second)),
            true);
      CHECK(bezier_geometry::sufficientlyClose(
                current.eq2Left.valueAt(currentResult.first),
                current.eq2Right.valueAt(currentResult.second)),
            true);
      {
        bool resultFound = false;
        for (const std::pair<bezier_geometry::RealNum, bezier_geometry::RealNum>
                 &currentExpectedSolution : current.expectedSolutions) {
          if (bezier_geometry::sufficientlyClose(currentExpectedSolution.first,
                                                 currentResult.first) &&
              bezier_geometry::sufficientlyClose(currentExpectedSolution.second,
                                                 currentResult.second)) {
            resultFound = true;
            break;
          }
        }
        CHECK(resultFound, true);
      }
    }
  }
  TEST_END
}

int main(int argc, char **argv) {
  std::cout << "%SUITE_STARTING% PolynomialFunctionTest" << std::endl;
  std::cout << "%SUITE_STARTED%" << std::endl;
  degreeTest();
  coefficientTest();
  equalsTest();
  toStringTest();
  derivativeTest();
  valueAtTest();
  getRootsTest();
  minMaxValTest();
  reduceTest();
  std::cout << "%SUITE_FINISHED% time=0" << std::endl;
  return (EXIT_SUCCESS);
}
