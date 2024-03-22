#include "BezierCurveQ.hpp"
#include "TestUtils.hpp"

template <typename A, typename B>
std::ostream &operator<<(std::ostream &os, const std::pair<A, B> &input) {
  os << "(" << input.first << "," << input.second << ")";
  return os;
}

// template <typename T>
// std::ostream &operator<<(std::ostream &os, const std::vector<T> &input) {
//   os << "[";
//   for (const T &current : input) {
//     if (&current != &input.front()) {
//       os << ",";
//     }
//     os << current;
//   }
//   os << "]";
//   return os;
// }

typedef struct {
  bezier_geometry::Point2D inputStart;
  bezier_geometry::Point2D inputEnd;
  bezier_geometry::Point2D inputControl;
  bezier_geometry::RealNum expectedMaxX;
  bezier_geometry::RealNum expectedMinX;
  bezier_geometry::RealNum expectedMaxY;
  bezier_geometry::RealNum expectedMinY;
} BasicFunctionalityTestEntry;

void basicFunctionalityTest() {
  TEST_START {
    bool failure = true;
    try {
      bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                    bezier_geometry::Point2D(2, 2),
                                    bezier_geometry::Point2D(3, 3))
          .valueAt(1.1);
    } catch (std::string) {
      failure = false;
    }
    CHECK(failure, false);
  }
  {
    bool failure = true;
    try {
      bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                    bezier_geometry::Point2D(2, 2),
                                    bezier_geometry::Point2D(3, 3))
          .valueAt(-0.1);
    } catch (std::string) {
      failure = false;
    }
    CHECK(failure, false);
  }
  {
    bool failure = true;
    try {
      bezier_geometry::BezierCurveQ test(bezier_geometry::Point2D(0, 0),
                                         bezier_geometry::Point2D(0, 0),
                                         bezier_geometry::Point2D(0, 0));
    } catch (std::string) {
      failure = false;
    }
    CHECK(failure, false);
  }
  {
    bool failure = true;
    try {
      bezier_geometry::BezierCurveQ test(bezier_geometry::Point2D(0, 0),
                                         bezier_geometry::Point2D(0, 0),
                                         bezier_geometry::Point2D(1, 1));
    } catch (std::string) {
      failure = false;
    }
    CHECK(failure, false);
  }
  std::vector<BasicFunctionalityTestEntry> testInputs;
  {
    BasicFunctionalityTestEntry entry = {bezier_geometry::Point2D(0, 1),
                                         bezier_geometry::Point2D(1, 0),
                                         bezier_geometry::Point2D(0.5, 0.5),
                                         1,
                                         0,
                                         1,
                                         0};
    testInputs.push_back(entry);
  }
  {
    BasicFunctionalityTestEntry entry = {bezier_geometry::Point2D(0, 1),
                                         bezier_geometry::Point2D(1, 1),
                                         bezier_geometry::Point2D(0.5, 1),
                                         1,
                                         0,
                                         1,
                                         1};
    testInputs.push_back(entry);
  }
  {
    BasicFunctionalityTestEntry entry = {bezier_geometry::Point2D(1, 0),
                                         bezier_geometry::Point2D(1, 1),
                                         bezier_geometry::Point2D(1, 0.5),
                                         1,
                                         1,
                                         1,
                                         0};
    testInputs.push_back(entry);
  }
  {
    BasicFunctionalityTestEntry entry = {bezier_geometry::Point2D(0, 1),
                                         bezier_geometry::Point2D(1, 0),
                                         bezier_geometry::Point2D(1, 1),
                                         1,
                                         0,
                                         1,
                                         0};
    testInputs.push_back(entry);
  }
  {
    BasicFunctionalityTestEntry entry = {bezier_geometry::Point2D(1, 7),
                                         bezier_geometry::Point2D(3, -2),
                                         bezier_geometry::Point2D(7, 6.5),
                                         4.6,
                                         1,
                                         7,
                                         -2};
    testInputs.push_back(entry);
  }
  {
    BasicFunctionalityTestEntry entry = {bezier_geometry::Point2D(1, 7),
                                         bezier_geometry::Point2D(3, -2),
                                         bezier_geometry::Point2D(7, 10),
                                         4.6,
                                         1,
                                         7.6,
                                         -2};
    testInputs.push_back(entry);
  }
  {
    BasicFunctionalityTestEntry entry = {bezier_geometry::Point2D(1, 7),
                                         bezier_geometry::Point2D(3, -2),
                                         bezier_geometry::Point2D(7, -10),
                                         4.6,
                                         1,
                                         7,
                                         -4.56};
    testInputs.push_back(entry);
  }
  {
    BasicFunctionalityTestEntry entry = {bezier_geometry::Point2D(1, 7),
                                         bezier_geometry::Point2D(5, 10),
                                         bezier_geometry::Point2D(-2, -10),
                                         5,
                                         0.1,
                                         10,
                                         -0.8108108108108};
    testInputs.push_back(entry);
  }
  bool colinearTest = false;
  for (std::vector<BasicFunctionalityTestEntry>::iterator i =
           testInputs.begin();
       i != testInputs.end(); i++) {
    for (int k = 0; k < 2; k++) {
      BasicFunctionalityTestEntry testEntry(*i);
      if (k == 1) {
        testEntry.inputStart =
            bezier_geometry::Point2D(testEntry.inputStart.getX() * 10000,
                                     testEntry.inputStart.getY() * 10000);
        testEntry.inputEnd =
            bezier_geometry::Point2D(testEntry.inputEnd.getX() * 10000,
                                     testEntry.inputEnd.getY() * 10000);
        testEntry.inputControl =
            bezier_geometry::Point2D(testEntry.inputControl.getX() * 10000,
                                     testEntry.inputControl.getY() * 10000);
        testEntry.expectedMaxX = testEntry.expectedMaxX * 10000;
        testEntry.expectedMinX = testEntry.expectedMinX * 10000;
        testEntry.expectedMaxY = testEntry.expectedMaxY * 10000;
        testEntry.expectedMinY = testEntry.expectedMinY * 10000;
      }
      bezier_geometry::BezierCurveQ test(
          testEntry.inputStart, testEntry.inputEnd, testEntry.inputControl);
      CHECK(test.valueAt(0), testEntry.inputStart);
      CHECK(test.valueAt(1), testEntry.inputEnd);
      CHECK(test.getControl(), testEntry.inputControl);
      CHECK(test.toString().find(testEntry.inputStart.toString()), 0);
      CHECK(test.toString().rfind(testEntry.inputEnd.toString()),
            test.toString().size() - testEntry.inputEnd.toString().size());
      {
        bezier_geometry::Point2D testPoint(
            testEntry.expectedMaxX + 1,
            test.valueAt(test.getMaxXPara()).getY());
        CHECK(bezier_geometry::sufficientlyClose(
                  test.valueAt(test.minDistance(testPoint))
                      .distanceFrom(testPoint),
                  1),
              true);
      }
      {
        bezier_geometry::Point2D testPoint1(
            testEntry.expectedMaxX, test.valueAt(test.getMinXPara()).getY());
        bezier_geometry::Point2D testPoint2(
            test.valueAt(test.getMinYPara()).getX(), testEntry.expectedMaxY);
        CHECK(bezier_geometry::sufficientlyClose(test.maxDistance(testPoint1),
                                                 test.getMinXPara()) ||
                  bezier_geometry::sufficientlyClose(
                      test.maxDistance(testPoint2), test.getMinYPara()),
              true);
      }
      CHECK(bezier_geometry::sufficientlyClose(test.getMaxXExtent(),
                                               testEntry.expectedMaxX),
            true);
      CHECK(
          bezier_geometry::sufficientlyClose(
              test.valueAt(test.getMaxXPara()).getX(), testEntry.expectedMaxX),
          true);
      CHECK(test.valueAt(test.minDistance(test.valueAt(test.getMaxXPara()))),
            test.valueAt(test.getMaxXPara()));
      {
        bezier_geometry::BezierCurveQ testLine(
            bezier_geometry::BezierCurveQ::longStraightLine(
                std::numeric_limits<bezier_geometry::RealNum>::infinity(),
                bezier_geometry::Point2D(
                    test.getMaxXExtent(),
                    test.valueAt(test.getMaxXPara()).getY())));
        bezier_geometry::StaticVector<
            std::pair<bezier_geometry::RealNum, bezier_geometry::RealNum>, 4>
            testLineIntersection(testLine.pointsOfIntersection(test));
        CHECK(bezier_geometry::BezierCurveQ::isIntersectionInfinite(
                  testLineIntersection) ||
                  testLineIntersection.size() > 0,
              true);
        CHECK(testLine.shift(0.1, 0, true, true)
                  .pointsOfIntersection(test)
                  .size(),
              static_cast<std::size_t>(0));
      }
      CHECK(bezier_geometry::sufficientlyClose(test.getMaxYExtent(),
                                               testEntry.expectedMaxY),
            true);
      CHECK(
          bezier_geometry::sufficientlyClose(
              test.valueAt(test.getMaxYPara()).getY(), testEntry.expectedMaxY),
          true);
      {
        bezier_geometry::BezierCurveQ testLine(
            bezier_geometry::BezierCurveQ::longStraightLine(
                0, bezier_geometry::Point2D(
                       test.valueAt(test.getMaxYPara()).getX(),
                       test.getMaxYExtent())));
        bezier_geometry::StaticVector<
            std::pair<bezier_geometry::RealNum, bezier_geometry::RealNum>, 4>
            testLineIntersection(testLine.pointsOfIntersection(test));
        CHECK(bezier_geometry::BezierCurveQ::isIntersectionInfinite(
                  testLineIntersection) ||
                  testLineIntersection.size() > 0,
              true);
        CHECK(testLine
                  .shift(
                      0.1,
                      std::numeric_limits<bezier_geometry::RealNum>::infinity(),
                      true, true)
                  .pointsOfIntersection(test)
                  .size(),
              static_cast<std::size_t>(0));
      }
      CHECK(bezier_geometry::sufficientlyClose(test.getMinXExtent(),
                                               testEntry.expectedMinX),
            true);
      CHECK(
          bezier_geometry::sufficientlyClose(
              test.valueAt(test.getMinXPara()).getX(), testEntry.expectedMinX),
          true);
      {
        bezier_geometry::BezierCurveQ testLine(
            bezier_geometry::BezierCurveQ::longStraightLine(
                std::numeric_limits<bezier_geometry::RealNum>::infinity(),
                bezier_geometry::Point2D(
                    test.getMinXExtent(),
                    test.valueAt(test.getMinXPara()).getY())));
        bezier_geometry::StaticVector<
            std::pair<bezier_geometry::RealNum, bezier_geometry::RealNum>, 4>
            testLineIntersection(testLine.pointsOfIntersection(test));
        CHECK(bezier_geometry::BezierCurveQ::isIntersectionInfinite(
                  testLineIntersection) ||
                  testLineIntersection.size() > 0,
              true);
        CHECK(testLine.shift(0.1, 0, false, true)
                  .pointsOfIntersection(test)
                  .size(),
              static_cast<std::size_t>(0));
      }
      CHECK(bezier_geometry::sufficientlyClose(test.getMinYExtent(),
                                               testEntry.expectedMinY),
            true);
      CHECK(
          bezier_geometry::sufficientlyClose(
              test.valueAt(test.getMinYPara()).getY(), testEntry.expectedMinY),
          true);
      {
        bezier_geometry::BezierCurveQ testLine(
            bezier_geometry::BezierCurveQ::longStraightLine(
                0, bezier_geometry::Point2D(
                       test.valueAt(test.getMinYPara()).getX(),
                       test.getMinYExtent())));
        bezier_geometry::StaticVector<
            std::pair<bezier_geometry::RealNum, bezier_geometry::RealNum>, 4>
            testLineIntersection(testLine.pointsOfIntersection(test));
        CHECK(bezier_geometry::BezierCurveQ::isIntersectionInfinite(
                  testLineIntersection) ||
                  testLineIntersection.size() > 0,
              true);
        CHECK(testLine
                  .shift(
                      0.1,
                      std::numeric_limits<bezier_geometry::RealNum>::infinity(),
                      true, false)
                  .pointsOfIntersection(test)
                  .size(),
              static_cast<std::size_t>(0));
      }
      for (int j = 0; j < 100; j++) {
        bezier_geometry::RealNum testParam = std::fabs(randomReal(0, 1));
        {
          bezier_geometry::RealNum testAngle = randomReal(0, 1000);
          bezier_geometry::Point2D testFulcrum(randomReal(0, 1000),
                                               randomReal(0, 1000));
          CHECK(test.valueAt(testParam).rotate(testFulcrum, testAngle),
                test.rotate(testFulcrum, testAngle).valueAt(testParam));
          CHECK(test.valueAt(testParam),
                test.rotate(testFulcrum, testAngle)
                    .rotate(testFulcrum, testAngle * -1.0)
                    .valueAt(testParam));
          CHECK(test, test.rotate(testFulcrum, testAngle)
                          .rotate(testFulcrum, testAngle * -1.0));
        }
        {
          bezier_geometry::RealNum shiftDistance =
              std::fabs(randomReal(0, 1000));
          bezier_geometry::RealNum shiftSlope = randomReal(0, 1000);
          bool shiftRight = randomReal(0, 1000) > 0;
          bool shiftUp = randomReal(0, 1000) > 0;
          CHECK(test.valueAt(testParam).shift(shiftDistance, shiftSlope,
                                              shiftRight, shiftUp),
                test.shift(shiftDistance, shiftSlope, shiftRight, shiftUp)
                    .valueAt(testParam));
          CHECK(test.valueAt(testParam),
                test.shift(shiftDistance, shiftSlope, shiftRight, shiftUp)
                    .shift(shiftDistance, shiftSlope, !shiftRight, !shiftUp)
                    .valueAt(testParam));
          CHECK(test,
                test.shift(shiftDistance, shiftSlope, shiftRight, shiftUp)
                    .shift(shiftDistance, shiftSlope, !shiftRight, !shiftUp));
        }
      }
      if (bezier_geometry::Point2D::colinear(
              testEntry.inputStart, testEntry.inputEnd,
              testEntry.inputControl)) { // Straight line.
        colinearTest = true;
        CHECK(bezier_geometry::sufficientlyClose(test.rateOfChangeAtParam(0),
                                                 test.rateOfChangeAtParam(1)),
              true);
        CHECK(bezier_geometry::sufficientlyClose(test.rateOfChangeAtParam(0.5),
                                                 test.rateOfChangeAtParam(1)),
              true);
        CHECK(bezier_geometry::sufficientlyClose(test.rateOfChangeAtParam(0.25),
                                                 test.rateOfChangeAtParam(.75)),
              true);
        bezier_geometry::RealNum slope =
            (testEntry.inputEnd.getY() - testEntry.inputStart.getY()) /
            (testEntry.inputEnd.getX() - testEntry.inputStart.getX());
        CHECK(bezier_geometry::sufficientlyClose(slope,
                                                 test.rateOfChangeAtParam(.75)),
              true);
        CHECK(bezier_geometry::BezierCurveQ::isIntersectionInfinite(
                  bezier_geometry::BezierCurveQ::longStraightLine(
                      slope, testEntry.inputControl)
                      .pointsOfIntersection(test)),
              true);
        CHECK(bezier_geometry::BezierCurveQ::isIntersectionInfinite(
                  bezier_geometry::BezierCurveQ::longStraightLine(
                      testEntry.inputControl, testEntry.inputStart)
                      .pointsOfIntersection(test)),
              true);
        CHECK(bezier_geometry::BezierCurveQ::isIntersectionInfinite(
                  bezier_geometry::BezierCurveQ::straightLine(
                      testEntry.inputStart, testEntry.inputEnd)
                      .pointsOfIntersection(test)),
              true);
        CHECK(bezier_geometry::BezierCurveQ::straightLine(testEntry.inputStart,
                                                          testEntry.inputEnd)
                  .shift(1, 1, true, true)
                  .pointsOfIntersection(test)
                  .size(),
              static_cast<std::size_t>(0));
      } else {
        bezier_geometry::RealNum startSlope = test.rateOfChangeAtParam(0);
        bezier_geometry::RealNum startControlSlope =
            (test.getControl().getY() - test.valueAt(0).getY()) /
            (test.getControl().getX() - test.valueAt(0).getX());
        CHECK(bezier_geometry::sufficientlyClose(startSlope, startControlSlope),
              true);
        bezier_geometry::RealNum endSlope = test.rateOfChangeAtParam(1);
        bezier_geometry::RealNum endControlSlope =
            (test.getControl().getY() - test.valueAt(1).getY()) /
            (test.getControl().getX() - test.valueAt(1).getX());
        CHECK(bezier_geometry::sufficientlyClose(endSlope, endControlSlope),
              true);
      }
    }
  }
  CHECK(colinearTest, true);
  TEST_END
}

typedef struct {
  bezier_geometry::BezierCurveQ first;
  bezier_geometry::BezierCurveQ second;
  bezier_geometry::StaticVector<
      std::pair<bezier_geometry::RealNum, bezier_geometry::RealNum>, 4>
      firstIntersections;
} PointsOfIntersectionTestEntry;

void pointsOfIntersectionTest() {
  TEST_START
  const bezier_geometry::RealNum multiplier = 10000;
  const bezier_geometry::RealNum unacceptableDiff =
      bezier_geometry::ACCEPTABLE_ERROR_MARGIN +
      (bezier_geometry::ACCEPTABLE_ERROR_MARGIN / 10.0);
  const bezier_geometry::RealNum acceptableDiff =
      bezier_geometry::ACCEPTABLE_ERROR_MARGIN / (multiplier + 1);
  std::vector<PointsOfIntersectionTestEntry> testInputs;
  /*
  ------------------------------------------------------------------------------
  2 straight lines, same slope.
  */
  {
    PointsOfIntersectionTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                      bezier_geometry::Point2D(1, 3),
                                      bezier_geometry::Point2D(1, 2)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 3),
                                      bezier_geometry::Point2D(1, 5),
                                      bezier_geometry::Point2D(1, 4)),
        {{1, 0}}};
    testInputs.push_back(entry);
  }
  {
    PointsOfIntersectionTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                      bezier_geometry::Point2D(1, 3),
                                      bezier_geometry::Point2D(1, 2)),
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(1 + acceptableDiff, 3),
            bezier_geometry::Point2D(1 + acceptableDiff, 5),
            bezier_geometry::Point2D(1 + acceptableDiff, 4)),
        {{1, 0}}};
    testInputs.push_back(entry);
  }
  {
    PointsOfIntersectionTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                      bezier_geometry::Point2D(1, 3),
                                      bezier_geometry::Point2D(1, 2)),
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(1 + unacceptableDiff, 3),
            bezier_geometry::Point2D(1 + unacceptableDiff, 5),
            bezier_geometry::Point2D(1 + unacceptableDiff, 4)),
        {}};
    testInputs.push_back(entry);
  }
  {
    PointsOfIntersectionTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                      bezier_geometry::Point2D(1, 3),
                                      bezier_geometry::Point2D(1, 2)),
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(1 + acceptableDiff, 3),
            bezier_geometry::Point2D(1 + acceptableDiff, 5),
            bezier_geometry::Point2D(1 + unacceptableDiff, 4)),
        {{1, 0}}};
    testInputs.push_back(entry);
  }
  {
    PointsOfIntersectionTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                      bezier_geometry::Point2D(1, 3),
                                      bezier_geometry::Point2D(1, 2)),
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(1, 3 + unacceptableDiff),
            bezier_geometry::Point2D(1, 5), bezier_geometry::Point2D(1, 4)),
        {}};
    testInputs.push_back(entry);
  }
  {
    PointsOfIntersectionTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                      bezier_geometry::Point2D(1, 3),
                                      bezier_geometry::Point2D(1, 2)),
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(1, 3 - unacceptableDiff),
            bezier_geometry::Point2D(1, 5), bezier_geometry::Point2D(1, 4)),
        {{std::numeric_limits<bezier_geometry::RealNum>::quiet_NaN(),
          std::numeric_limits<bezier_geometry::RealNum>::quiet_NaN()},
         {1, 5.4993950831663160588e-5},
         {0.99994500000000008377, 0}}};
    testInputs.push_back(entry);
  }
  {
    PointsOfIntersectionTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                      bezier_geometry::Point2D(1, 3),
                                      bezier_geometry::Point2D(1, 2)),
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(1, 3 + acceptableDiff),
            bezier_geometry::Point2D(1, 5), bezier_geometry::Point2D(1, 4)),
        {{1, 0}}};
    testInputs.push_back(entry);
  }
  /*
  ------------------------------------------------------------------------------
  2 straight lines, different slopes.
  */
  {
    PointsOfIntersectionTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(0, 0),
                                      bezier_geometry::Point2D(4, 4),
                                      bezier_geometry::Point2D(2, 2)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(4, 3),
                                      bezier_geometry::Point2D(2, 5),
                                      bezier_geometry::Point2D(3, 4)),
        {{0.875, 0.25}}};
    testInputs.push_back(entry);
  }
  {
    PointsOfIntersectionTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(0, 0),
                                      bezier_geometry::Point2D(0, 4),
                                      bezier_geometry::Point2D(0, 2)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(2, 3),
                                      bezier_geometry::Point2D(-4, 0),
                                      bezier_geometry::Point2D(-1, 1.5)),
        {{0.5, 0.33333333333333331483}}};
    testInputs.push_back(entry);
  }
  {
    PointsOfIntersectionTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(0, 0),
                                      bezier_geometry::Point2D(0, 4),
                                      bezier_geometry::Point2D(0, 2)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(2, 3),
                                      bezier_geometry::Point2D(0, 2),
                                      bezier_geometry::Point2D(1, 2.5)),
        {{0.5, 1}}};
    testInputs.push_back(entry);
  }
  {
    PointsOfIntersectionTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(0, 0),
                                      bezier_geometry::Point2D(0, 4),
                                      bezier_geometry::Point2D(0, 2)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(2, 2),
                                      bezier_geometry::Point2D(0, 2),
                                      bezier_geometry::Point2D(1, 2)),
        {{0.5, 1}}};
    testInputs.push_back(entry);
  }
  {
    PointsOfIntersectionTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(0, 0),
                                      bezier_geometry::Point2D(0, 4),
                                      bezier_geometry::Point2D(0, 2)),
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(2, 2),
            bezier_geometry::Point2D(0 + acceptableDiff, 2),
            bezier_geometry::Point2D(1, 2)),
        {{0.5, 1}}};
    testInputs.push_back(entry);
  }
  {
    PointsOfIntersectionTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(0, 0),
                                      bezier_geometry::Point2D(0, 4),
                                      bezier_geometry::Point2D(0, 2)),
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(2, 2),
            bezier_geometry::Point2D(0 + unacceptableDiff, 2),
            bezier_geometry::Point2D(1, 2)),
        {}};
    testInputs.push_back(entry);
  }
  {
    PointsOfIntersectionTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(0, 0),
                                      bezier_geometry::Point2D(0, 4),
                                      bezier_geometry::Point2D(0, 2)),
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(2, 2),
            bezier_geometry::Point2D(0.0 - unacceptableDiff, 2),
            bezier_geometry::Point2D(1, 2)),
        {{0.5, 1}}};
    testInputs.push_back(entry);
  }
  /*
  ------------------------------------------------------------------------------
  Various curves.
  */
  {
    PointsOfIntersectionTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 8),
                                      bezier_geometry::Point2D(7, 9),
                                      bezier_geometry::Point2D(4, 0)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                      bezier_geometry::Point2D(7, 0),
                                      bezier_geometry::Point2D(4, 10)),
        {{0.30325659638060503687, 0.30325659638060503687},
         {0.64118784806383943842, 0.64118784806383943842}}};
    testInputs.push_back(entry);
  }
  {
    PointsOfIntersectionTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(50, 40),
                                      bezier_geometry::Point2D(30, 60),
                                      bezier_geometry::Point2D(-20, -10)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(25, 2),
                                      bezier_geometry::Point2D(30, 15),
                                      bezier_geometry::Point2D(-5, 100)),
        {{0.20777391596408648078, 0.94093239991331878436},
         {0.29292903206591147658, 0.10781150260033661725},
         {0.80000345236063141741, 0.22468948757673531946},
         {0.85789575014699648303, 0.79108273894186775799}}};
    testInputs.push_back(entry);
  }
  {
    PointsOfIntersectionTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(2, 2),
                                      bezier_geometry::Point2D(8, 3),
                                      bezier_geometry::Point2D(4, 10)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(0, -10),
                                      bezier_geometry::Point2D(6, 0),
                                      bezier_geometry::Point2D(1, 9)),
        {{0, 0.5}}};
    testInputs.push_back(entry);
  }
  {
    PointsOfIntersectionTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(2, 2),
                                      bezier_geometry::Point2D(8, 3),
                                      bezier_geometry::Point2D(4, 10)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(0, -10),
                                      bezier_geometry::Point2D(6, 0),
                                      bezier_geometry::Point2D(1, 12)),
        {{0.16064789601388285156, 0.60793459497838553407}}};
    testInputs.push_back(entry);
  }
  {
    PointsOfIntersectionTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(2, 2),
                                      bezier_geometry::Point2D(8, 3),
                                      bezier_geometry::Point2D(4, 10)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 3),
                                      bezier_geometry::Point2D(1, 5),
                                      bezier_geometry::Point2D(20, 2)),
        {{0.062133320405741765402, 0.034231093573338561287},
         {0.20917176788164915102, 0.94650123096181848492},
         {0.93180770484741681781, 0.78266764857729753668}}};
    testInputs.push_back(entry);
  }
  {
    PointsOfIntersectionTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(2, 2),
                                      bezier_geometry::Point2D(8, 3),
                                      bezier_geometry::Point2D(4, 10)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(2, 2),
                                      bezier_geometry::Point2D(8, 3),
                                      bezier_geometry::Point2D(4, 8)),
        {{0, 0}, {1, 1}}};
    testInputs.push_back(entry);
  }
  {
    PointsOfIntersectionTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(2, 10),
                                      bezier_geometry::Point2D(10, 10),
                                      bezier_geometry::Point2D(6, 8)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(2, 8),
                                      bezier_geometry::Point2D(10, 8),
                                      bezier_geometry::Point2D(6, 10)),
        {{0.5, 0.5}}};
    testInputs.push_back(entry);
  }
  {
    PointsOfIntersectionTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(0, 0),
                                      bezier_geometry::Point2D(4, 4),
                                      bezier_geometry::Point2D(2, 2)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(-25000, 3),
                                      bezier_geometry::Point2D(25000, 3),
                                      bezier_geometry::Point2D(0, 3)),
        {{0.75, 0.5}}};
    testInputs.push_back(entry);
  }
  {
    PointsOfIntersectionTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                      bezier_geometry::Point2D(4, 4),
                                      bezier_geometry::Point2D(2, 2)),
        bezier_geometry::BezierCurveQ::longStraightLine(
            0, bezier_geometry::Point2D(3, 3)),
        {{0.7320508075688773042, 0.5}}};
    testInputs.push_back(entry);
  }
  { // Not touching, no straight lines, bounding boxes overlap.
    PointsOfIntersectionTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(2.15, 1),
                                      bezier_geometry::Point2D(3.15, 2),
                                      bezier_geometry::Point2D(4.15, 4)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(3, 2),
                                      bezier_geometry::Point2D(4, 3),
                                      bezier_geometry::Point2D(2, 0)),
        {}};
    testInputs.push_back(entry);
  }
  /*
  End inputs.
  ------------------------------------------------------------------------------
  */
  for (std::vector<PointsOfIntersectionTestEntry>::iterator i =
           testInputs.begin();
       i != testInputs.end(); i++) {
    for (int j = 0; j < 3; j++) {
      bool multiplierSet = false;
      PointsOfIntersectionTestEntry currentEntry(*i);
      if (j == 1) {
        multiplierSet = true;
        currentEntry.first = bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(
                currentEntry.first.valueAt(0).getX() * multiplier,
                currentEntry.first.valueAt(0).getY() * multiplier),
            bezier_geometry::Point2D(
                currentEntry.first.valueAt(1).getX() * multiplier,
                currentEntry.first.valueAt(1).getY() * multiplier),
            bezier_geometry::Point2D(
                currentEntry.first.getControl().getX() * multiplier,
                currentEntry.first.getControl().getY() * multiplier));
        currentEntry.second = bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(
                currentEntry.second.valueAt(0).getX() * multiplier,
                currentEntry.second.valueAt(0).getY() * multiplier),
            bezier_geometry::Point2D(
                currentEntry.second.valueAt(1).getX() * multiplier,
                currentEntry.second.valueAt(1).getY() * multiplier),
            bezier_geometry::Point2D(
                currentEntry.second.getControl().getX() * multiplier,
                currentEntry.second.getControl().getY() * multiplier));
      } else if (j == 2) {
        currentEntry.first =
            currentEntry.first.rotate(bezier_geometry::Point2D(0, 0), -90);
        currentEntry.second =
            currentEntry.second.rotate(bezier_geometry::Point2D(0, 0), -90);
      }
      debug() << "************** BEFORE pointsOfIntersection -"
              << "first:" << currentEntry.first
              << "second:" << currentEntry.second << std::endl;
      bezier_geometry::StaticVector<
          std::pair<bezier_geometry::RealNum, bezier_geometry::RealNum>, 4>
          firstIntersections(
              currentEntry.first.pointsOfIntersection(currentEntry.second));
      debug() << "************** AFTER pointsOfIntersection -"
              << "result:" << firstIntersections
              << "expected:" << currentEntry.firstIntersections << std::endl;
      CHECK(bezier_geometry::BezierCurveQ::isIntersectionInfinite(
                currentEntry.firstIntersections),
            bezier_geometry::BezierCurveQ::isIntersectionInfinite(
                firstIntersections));
      if (bezier_geometry::BezierCurveQ::isIntersectionInfinite(
              firstIntersections)) {
        CHECK(currentEntry.first.valueAt(firstIntersections[1].first),
              currentEntry.second.valueAt(firstIntersections[1].second));
        CHECK(currentEntry.first.valueAt(firstIntersections[2].first),
              currentEntry.second.valueAt(firstIntersections[2].second));
      } else {
        if (currentEntry.firstIntersections.size() == 0) {
          CHECK(firstIntersections.size(),
                currentEntry.firstIntersections.size());
        } else {
          bezier_geometry::StaticVector<
              std::pair<bezier_geometry::RealNum, bezier_geometry::RealNum>, 16>
              matchedParameters;
          for (typename bezier_geometry::StaticVector<
                   std::pair<bezier_geometry::RealNum,
                             bezier_geometry::RealNum>,
                   4>::iterator k = currentEntry.firstIntersections.begin();
               k != currentEntry.firstIntersections.end(); k++) {
            if (std::find(matchedParameters.begin(), matchedParameters.end(),
                          *k) != matchedParameters.end()) {
              continue;
            }
            for (typename bezier_geometry::StaticVector<
                     std::pair<bezier_geometry::RealNum,
                               bezier_geometry::RealNum>,
                     4>::iterator l = firstIntersections.begin();
                 l != firstIntersections.end(); l++) {
              if (!multiplierSet) {
                CHECK(currentEntry.first.valueAt(l->first),
                      currentEntry.second.valueAt(l->second));
              }
              if ((currentEntry.first.valueAt(l->first) ==
                       currentEntry.first.valueAt(k->first) ||
                   bezier_geometry::sufficientlyClose(l->first, k->first)) &&
                  (currentEntry.second.valueAt(l->second) ==
                       currentEntry.second.valueAt(k->second) ||
                   bezier_geometry::sufficientlyClose(l->second, k->second))) {
                matchedParameters.push_back(*k);
                break;
              }
            }
          }
          if (matchedParameters.size() > 0) {
            std::sort(matchedParameters.begin(), matchedParameters.end());
          }
          if (matchedParameters != currentEntry.firstIntersections) {
            std::string matchedOutput;
            for (typename bezier_geometry::StaticVector<
                     std::pair<bezier_geometry::RealNum,
                               bezier_geometry::RealNum>,
                     16>::iterator k = matchedParameters.begin();
                 k != matchedParameters.end(); k++) {
              matchedOutput += k == matchedParameters.begin() ? "" : ", ";
              matchedOutput += "(" + bezier_geometry::toString(k->first) +
                               ", " + bezier_geometry::toString(k->second) +
                               ")";
            }
            std::string expectedOutput;
            for (typename bezier_geometry::StaticVector<
                     std::pair<bezier_geometry::RealNum,
                               bezier_geometry::RealNum>,
                     4>::iterator k = currentEntry.firstIntersections.begin();
                 k != currentEntry.firstIntersections.end(); k++) {
              expectedOutput +=
                  k == currentEntry.firstIntersections.begin() ? "" : ", ";
              expectedOutput += "(" + bezier_geometry::toString(k->first) +
                                ", " + bezier_geometry::toString(k->second) +
                                ")";
            }
            debug() << std::string("CURVE: ") + currentEntry.first.toString()
                    << std::endl;
            debug() << std::string("INPUT: ") + currentEntry.second.toString()
                    << std::endl;
            debug() << std::string("EXPECTED: ") + expectedOutput << std::endl;
            debug() << std::string("FOUND: ") + matchedOutput << std::endl;
          }
          CHECK(matchedParameters, currentEntry.firstIntersections);
        }
        for (typename bezier_geometry::StaticVector<
                 std::pair<bezier_geometry::RealNum, bezier_geometry::RealNum>,
                 4>::iterator k = firstIntersections.begin();
             k != firstIntersections.end(); k++) {
          bezier_geometry::Point2D testPoint(
              currentEntry.first.valueAt(k->first));
          bezier_geometry::RealNum minDistanceParam =
              currentEntry.second.minDistance(testPoint);
          bezier_geometry::Point2D minDistancePoint(
              currentEntry.second.valueAt(minDistanceParam));
          if (minDistancePoint != testPoint) {
            bezier_geometry::Point2D otherPoint1(currentEntry.second.valueAt(
                minDistanceParam >= 0.0001 ? minDistanceParam - 0.0001
                                           : minDistanceParam));
            bezier_geometry::Point2D otherPoint2(currentEntry.second.valueAt(
                minDistanceParam <= 0.9999 ? minDistanceParam + 0.0001
                                           : minDistanceParam));
            CHECK(otherPoint1.distanceFrom(testPoint) >=
                      minDistancePoint.distanceFrom(testPoint),
                  true);
            CHECK(otherPoint2.distanceFrom(testPoint) >=
                      minDistancePoint.distanceFrom(testPoint),
                  true);
          }
        }
      }
    }
  }
  TEST_END
}

typedef struct {
  bezier_geometry::BezierCurveQ moving;
  bezier_geometry::BezierCurveQ stationary;
  bezier_geometry::RealNum slope;
  bool right;
  bool up;
  bezier_geometry::RealNum expectedDistance;
} ShiftAgainstTestEntry;

void shiftAgainstTest() {
  TEST_START
  std::vector<ShiftAgainstTestEntry> testInputs;
  {
    ShiftAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                      bezier_geometry::Point2D(3, 3),
                                      bezier_geometry::Point2D(2, 2)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(3, 1),
                                      bezier_geometry::Point2D(3, 3),
                                      bezier_geometry::Point2D(3, 2)),
        0,
        false,
        true,
        -1};
    testInputs.push_back(entry);
  }
  {
    ShiftAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                      bezier_geometry::Point2D(4, 4),
                                      bezier_geometry::Point2D(2, 2)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(3, 1),
                                      bezier_geometry::Point2D(3, 3),
                                      bezier_geometry::Point2D(3, 2)),
        0,
        false,
        true,
        -1};
    testInputs.push_back(entry);
  }
  {
    ShiftAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                      bezier_geometry::Point2D(4, 4),
                                      bezier_geometry::Point2D(2, 2)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(3, 1),
                                      bezier_geometry::Point2D(3, 3),
                                      bezier_geometry::Point2D(3, 2)),
        0,
        true,
        true,
        0};
    testInputs.push_back(entry);
  }
  {
    ShiftAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                      bezier_geometry::Point2D(3, 3),
                                      bezier_geometry::Point2D(2, 2)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(3, 1),
                                      bezier_geometry::Point2D(3, 3),
                                      bezier_geometry::Point2D(3, 2)),
        0,
        true,
        true,
        0};
    testInputs.push_back(entry);
  }
  {
    ShiftAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                      bezier_geometry::Point2D(3, 3),
                                      bezier_geometry::Point2D(2, 2)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(3, 1),
                                      bezier_geometry::Point2D(3, 3),
                                      bezier_geometry::Point2D(3, 2)),
        0,
        false,
        true,
        -1};
    testInputs.push_back(entry);
  }
  {
    ShiftAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                      bezier_geometry::Point2D(3, 3),
                                      bezier_geometry::Point2D(2, 2)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(3, 1),
                                      bezier_geometry::Point2D(3, 3),
                                      bezier_geometry::Point2D(3, 2)),
        0.99,
        true,
        true,
        0};
    testInputs.push_back(entry);
  }
  {
    ShiftAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                      bezier_geometry::Point2D(3, 3),
                                      bezier_geometry::Point2D(2, 2)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(3, 1),
                                      bezier_geometry::Point2D(3, 3),
                                      bezier_geometry::Point2D(3, 2)),
        1,
        true,
        true,
        -1};
    testInputs.push_back(entry);
  }
  {
    ShiftAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                      bezier_geometry::Point2D(3, 3),
                                      bezier_geometry::Point2D(2, 2)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(3, 1),
                                      bezier_geometry::Point2D(3, 3),
                                      bezier_geometry::Point2D(3, 2)),
        2,
        true,
        true,
        -1};
    testInputs.push_back(entry);
  }
  {
    ShiftAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                      bezier_geometry::Point2D(3, 3),
                                      bezier_geometry::Point2D(2, 2)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(3, 1),
                                      bezier_geometry::Point2D(3, 3),
                                      bezier_geometry::Point2D(3, 2)),
        -2,
        true,
        true,
        0};
    testInputs.push_back(entry);
  }
  {
    ShiftAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                      bezier_geometry::Point2D(3, 3),
                                      bezier_geometry::Point2D(2, 2)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(3, 1),
                                      bezier_geometry::Point2D(3, 3),
                                      bezier_geometry::Point2D(3, 2)),
        //-5729578,
        -2864785, true, true, -1};
    testInputs.push_back(entry);
  }
  {
    ShiftAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                      bezier_geometry::Point2D(3, 3),
                                      bezier_geometry::Point2D(2, 2)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(4, 1),
                                      bezier_geometry::Point2D(4, 3),
                                      bezier_geometry::Point2D(4, 2)),
        0,
        true,
        true,
        1};
    testInputs.push_back(entry);
  }
  { // Touching, same slope at intersection, slope is 0.
    ShiftAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                      bezier_geometry::Point2D(2, 2),
                                      bezier_geometry::Point2D(1, 2)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(2, 2),
                                      bezier_geometry::Point2D(3, 1),
                                      bezier_geometry::Point2D(3, 2)),
        0,
        true,
        true,
        0};
    testInputs.push_back(entry);
  }
  { // Touching, same slope at intersection, slope is not 0, intersection is an
    // endpoint of both curves and they extend in non-overlapping directions.
    ShiftAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                      bezier_geometry::Point2D(2, 2),
                                      bezier_geometry::Point2D(3, 4)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(2, 2),
                                      bezier_geometry::Point2D(3, 3),
                                      bezier_geometry::Point2D(1, 0)),
        0,
        true,
        true,
        0.32};
    testInputs.push_back(entry);
  }
  { // Not touching, same slope at closest point, slope is not 0, closest point
    // is an endpoint of both curves and they extend in non-overlapping
    // directions.
    ShiftAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                      bezier_geometry::Point2D(2, 2),
                                      bezier_geometry::Point2D(3, 4)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(3, 2),
                                      bezier_geometry::Point2D(4, 3),
                                      bezier_geometry::Point2D(2, 0)),
        0,
        true,
        true,
        1.32};
    testInputs.push_back(entry);
  }
  { // Not touching, same slope at closest point, slope is 0.
    ShiftAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                      bezier_geometry::Point2D(2, 2),
                                      bezier_geometry::Point2D(1, 2)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(3, 2),
                                      bezier_geometry::Point2D(4, 1),
                                      bezier_geometry::Point2D(4, 2)),
        0,
        true,
        true,
        1};
    testInputs.push_back(entry);
  }
  { // Touching, same slope at intersection, slope is 0, curves extend in
    // non-overlapping directions
    ShiftAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                      bezier_geometry::Point2D(2, 2),
                                      bezier_geometry::Point2D(1, 2)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(2, 2),
                                      bezier_geometry::Point2D(3, 3),
                                      bezier_geometry::Point2D(3, 2)),
        0,
        true,
        true,
        -1};
    testInputs.push_back(entry);
  }
  { // Not touching, same slope at closest point, slope is 0, curves extend in
    // non-overlapping directions
    ShiftAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                      bezier_geometry::Point2D(2, 2),
                                      bezier_geometry::Point2D(1, 2)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(3, 2),
                                      bezier_geometry::Point2D(4, 3),
                                      bezier_geometry::Point2D(4, 2)),
        0,
        true,
        true,
        -1};
    testInputs.push_back(entry);
  }
  { // Touching, same non-zero slope at intersection, stationary object
    // obstructs at intersection.
    ShiftAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                      bezier_geometry::Point2D(2, 1),
                                      bezier_geometry::Point2D(3, 2)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 0),
                                      bezier_geometry::Point2D(4, 3),
                                      bezier_geometry::Point2D(2, 1)),
        0,
        true,
        true,
        0};
    testInputs.push_back(entry);
  }
  { // Touching, same non-zero slope at intersection, stationary object does
    // not
    // obstruct at intersection.
    ShiftAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                      bezier_geometry::Point2D(2, 1),
                                      bezier_geometry::Point2D(3, 2)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(-1, 0),
                                      bezier_geometry::Point2D(3, 2),
                                      bezier_geometry::Point2D(1, 1)),
        0,
        true,
        true,
        -1};
    testInputs.push_back(entry);
  }
  { // Not touching, same non-zero slope at point of obstruction.
    ShiftAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                      bezier_geometry::Point2D(2, 1),
                                      bezier_geometry::Point2D(3, 2)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(2, 0),
                                      bezier_geometry::Point2D(5, 3),
                                      bezier_geometry::Point2D(3, 1)),
        0,
        true,
        true,
        1};
    testInputs.push_back(entry);
  }
  { // Touching at neither curves' endoint, stationary obstructs.
    ShiftAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                      bezier_geometry::Point2D(1, 3),
                                      bezier_geometry::Point2D(2, 2)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(0, 0),
                                      bezier_geometry::Point2D(0, 4),
                                      bezier_geometry::Point2D(3, 2)),
        0,
        true,
        true,
        0};
    testInputs.push_back(entry);
  }
  { // Touching at neither curves' endoint, stationary does not obstruct.
    ShiftAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(0, 0),
                                      bezier_geometry::Point2D(0, 4),
                                      bezier_geometry::Point2D(3, 2)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                      bezier_geometry::Point2D(1, 3),
                                      bezier_geometry::Point2D(2, 2)),
        0,
        true,
        true,
        -1};
    testInputs.push_back(entry);
  }
  { // Touching at neither curves' endoint, stationary does not obstruct,
    // except
    // there is a 'cross' at the intersection.
    ShiftAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(0, 0),
                                      bezier_geometry::Point2D(0, 4),
                                      bezier_geometry::Point2D(3, 2)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                      bezier_geometry::Point2D(1, 3),
                                      bezier_geometry::Point2D(2.01, 2)),
        0,
        true,
        true,
        0};
    testInputs.push_back(entry);
  }
  { // Not touching, stationary obstructs at both curves' endpoint, unequal
    // slopes.
    ShiftAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                      bezier_geometry::Point2D(3, 3),
                                      bezier_geometry::Point2D(2, 2)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(4, 1),
                                      bezier_geometry::Point2D(4, 3),
                                      bezier_geometry::Point2D(4, 2)),
        0,
        true,
        true,
        1};
    testInputs.push_back(entry);
  }
  { // Touching at one curve's endpoint, input obstructs.
    ShiftAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                      bezier_geometry::Point2D(1, 3),
                                      bezier_geometry::Point2D(3, 2)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(2, 1),
                                      bezier_geometry::Point2D(2, 2),
                                      bezier_geometry::Point2D(2, 1.5)),
        0,
        true,
        true,
        0};
    testInputs.push_back(entry);
  }
  { // Touching at one curve's endpoint, input does not obstruct.
    ShiftAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(2, 1),
                                      bezier_geometry::Point2D(2, 2),
                                      bezier_geometry::Point2D(2, 1.5)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                      bezier_geometry::Point2D(1, 3),
                                      bezier_geometry::Point2D(3, 2)),
        0,
        true,
        true,
        -1};
    testInputs.push_back(entry);
  }
  { // Not touching, input is not in path of intersection and therefore does
    // not
    // obstruct.
    ShiftAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                      bezier_geometry::Point2D(1, 3),
                                      bezier_geometry::Point2D(2, 2)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(10, 4),
                                      bezier_geometry::Point2D(9, 6),
                                      bezier_geometry::Point2D(9, 5)),
        0,
        true,
        true,
        -1};
    testInputs.push_back(entry);
  }
  { // Not touching, input is not in path of intersection and therefore does
    // not
    // obstruct.
    ShiftAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                      bezier_geometry::Point2D(1, 3),
                                      bezier_geometry::Point2D(2, 2)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(10, 4),
                                      bezier_geometry::Point2D(9, 6),
                                      bezier_geometry::Point2D(9, 5)),
        -0.2,
        true,
        true,
        -1};
    testInputs.push_back(entry);
  }
  { // Not touching, input is not in path of intersection and therefore does
    // not
    // obstruct.
    ShiftAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                      bezier_geometry::Point2D(1, 3),
                                      bezier_geometry::Point2D(2, 2)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(10, 4),
                                      bezier_geometry::Point2D(9, 6),
                                      bezier_geometry::Point2D(9, 5)),
        0.2,
        false,
        true,
        -1};
    testInputs.push_back(entry);
  }
  { // Not touching, non-zero slope of shift, stationary is in path.
    ShiftAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                      bezier_geometry::Point2D(1, 3),
                                      bezier_geometry::Point2D(2, 2)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(10, 4),
                                      bezier_geometry::Point2D(9, 6),
                                      bezier_geometry::Point2D(9, 5)),
        0.2,
        true,
        true,
        8.563762095};
    testInputs.push_back(entry);
  }
  { // Not touching, stationary is in path, horizontal shift, multiple
    // potential
    // intersection points in shift path.
    ShiftAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(5, 5),
                                      bezier_geometry::Point2D(10, 5),
                                      bezier_geometry::Point2D(8, 10)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(6, 6),
                                      bezier_geometry::Point2D(7, 6),
                                      bezier_geometry::Point2D(9, 8)),
        0,
        true,
        true,
        0.3364916731};
    testInputs.push_back(entry);
  }
  { // Produces that issue with input intersections not matching the first
    // curve's intersections.
    ShiftAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(6.21478, 1),
                                      bezier_geometry::Point2D(4.21478, -1),
                                      bezier_geometry::Point2D(4.21478, 1)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(7, 1),
                                      bezier_geometry::Point2D(6, 1),
                                      bezier_geometry::Point2D(6.48, 1)),
        15.1143,
        true,
        true,
        0};
    testInputs.push_back(entry);
  }
  { // Touching, stationary is in path, horizontal shift, stationary is
    // horizontal line.
    ShiftAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                      bezier_geometry::Point2D(1, 3),
                                      bezier_geometry::Point2D(2, 2)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1.5, 2),
                                      bezier_geometry::Point2D(5, 2),
                                      bezier_geometry::Point2D(4, 2)),
        0,
        true,
        true,
        0};
    testInputs.push_back(entry);
  }
  { // Not touching, stationary is not in path, horizontal shift, stationary is
    // horizontal line.
    ShiftAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                      bezier_geometry::Point2D(1, 3),
                                      bezier_geometry::Point2D(2, 2)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(0, 2),
                                      bezier_geometry::Point2D(1.4, 2),
                                      bezier_geometry::Point2D(0.5, 2)),
        0,
        true,
        true,
        -1};
    testInputs.push_back(entry);
  }
  { // Touching, stationary is not in path, horizontal shift, stationary is
    // horizontal line.
    ShiftAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                      bezier_geometry::Point2D(1, 3),
                                      bezier_geometry::Point2D(2, 2)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(0, 2),
                                      bezier_geometry::Point2D(1.5, 2),
                                      bezier_geometry::Point2D(0.5, 2)),
        0,
        true,
        true,
        -1};
    testInputs.push_back(entry);
  }
  { // Not touching, stationary is in path, horizontal shift, stationary is
    // horizontal line.
    ShiftAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                      bezier_geometry::Point2D(1, 3),
                                      bezier_geometry::Point2D(2, 2)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(3, 2),
                                      bezier_geometry::Point2D(5, 2),
                                      bezier_geometry::Point2D(4, 2)),
        0,
        true,
        true,
        1.5};
    testInputs.push_back(entry);
  }
  { // Not touching, stationary is in path, horizontal shift, both straight
    // horizontal lines.
    ShiftAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                      bezier_geometry::Point2D(3, 1),
                                      bezier_geometry::Point2D(2, 1)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(4, 1),
                                      bezier_geometry::Point2D(6, 1),
                                      bezier_geometry::Point2D(5, 1)),
        0,
        true,
        true,
        -1};
    testInputs.push_back(entry);
  }
  { // Touching but not overlapping, stationary is in path, horizontal shift,
    // both straight horizontal lines.
    ShiftAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                      bezier_geometry::Point2D(3, 1),
                                      bezier_geometry::Point2D(2, 1)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(3, 1),
                                      bezier_geometry::Point2D(5, 1),
                                      bezier_geometry::Point2D(4, 1)),
        0,
        true,
        true,
        -1};
    testInputs.push_back(entry);
  }
  { // Touching and overlapping, stationary is in path, horizontal shift, both
    // straight horizontal lines.
    ShiftAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                      bezier_geometry::Point2D(3, 1),
                                      bezier_geometry::Point2D(2, 1)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(2, 1),
                                      bezier_geometry::Point2D(4, 1),
                                      bezier_geometry::Point2D(3, 1)),
        0,
        true,
        true,
        -1};
    testInputs.push_back(entry);
  }
  { // Touching, stationary is in path, large scale coordinates - produced the
    // issue with endpoint intersecions being missed.
    ShiftAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(678.59012987012999929, 100),
            bezier_geometry::Point2D(478.59012987012994245, -100),
            bezier_geometry::Point2D(478.59012987012994245, 100)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(700, 100),
                                      bezier_geometry::Point2D(600, 100),
                                      bezier_geometry::Point2D(648, 100)),
        -1.1298701298701296913,
        false,
        true,
        0};
    testInputs.push_back(entry);
  }
  { // Barely not touching, stationary is in path, stationary is a straight
    // line
    // with large scale coordinates, stop point is moving's end.
    ShiftAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(10.000090229826355426,
                                     414.1451884327384505),
            bezier_geometry::Point2D(210.00009022982891338,
                                     67.735026918961921183),
            bezier_geometry::Point2D(10.000090229827051758,
                                     183.20508075688755412)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(10, 0),
                                      bezier_geometry::Point2D(10, 3015),
                                      bezier_geometry::Point2D(10, 1507.5)),
        -0.46038787456134389053,
        false,
        true,
        9.9333039798921163310296833515167236328125e-5};
    testInputs.push_back(entry);
  }
  { // Special case where the precompiled equation for a horizontal shift
    // equation numerator evaluates to 0 despite there being legitimate
    // solutions.
    ShiftAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(131, 300),
                                      bezier_geometry::Point2D(96, 91),
                                      bezier_geometry::Point2D(9, 213)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(10, 0),
                                      bezier_geometry::Point2D(10, 3015),
                                      bezier_geometry::Point2D(10, 1507.5)),
        0,
        false,
        true,
        49.784688995215304885};
    testInputs.push_back(entry);
  }
  { // Barely not touching, stationary is not in path.
    ShiftAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(798.45618655202292757,
                                     103.81941874331369036),
            bezier_geometry::Point2D(748.45618655202292757,
                                     103.81941874331346298),
            bezier_geometry::Point2D(773.45618655202292757,
                                     118.25317547305439803)),
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(703.42881472983594904,
                                     106.54760538034270212),
            bezier_geometry::Point2D(753.42881472983594904,
                                     106.54760538034270212),
            bezier_geometry::Point2D(728.42881472983549429,
                                     92.11384865060199445)),
        0,
        true,
        true,
        -1};
    testInputs.push_back(entry);
  }
  { // Touching at an endpoint, stationary is in path, moving along the
    // stationary curve's slope, stationary curve is concave at touch point.
    ShiftAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(1152.9041308429887067,
                                     1200.941917383140435),
            bezier_geometry::Point2D(352.90413084298870672,
                                     400.94191738314043505),
            bezier_geometry::Point2D(352.90413084298870672,
                                     1200.941917383140435)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(600, 300),
                                      bezier_geometry::Point2D(300, 600),
                                      bezier_geometry::Point2D(300, 300)),
        -1.381309450179175613,
        true,
        false,
        0};
    testInputs.push_back(entry);
  }
  { // Touch at neither curves' endpoint, stationary is not in path at
    // intersection, intersection is at an asymmetric point in each curve.
    ShiftAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(339.93005500749404746,
                                     436.15293719730630073),
            bezier_geometry::Point2D(364.93005500749404746,
                                     392.85166700808417772),
            bezier_geometry::Point2D(339.93005500749404746,
                                     407.28542373782488539)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(600, 300),
                                      bezier_geometry::Point2D(300, 600),
                                      bezier_geometry::Point2D(300, 300)),
        -4.24166907718649977,
        false,
        true,
        175.91292262897883347};
    testInputs.push_back(entry);
  }
  { // Touching at neither curves' endpoint, stationary is in path, curves are
    // basically horizontal mirror images with an overlap, causing extra cases
    // in solveSystem.
    ShiftAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(4411.9350024746709096,
                                     2621.1805812566844907),
            bezier_geometry::Point2D(4461.9350024746690906,
                                     2621.1805812566844907),
            bezier_geometry::Point2D(4436.9350024746709096,
                                     2606.7468245269428735)),
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(4501.2435499045359393,
                                     2615.6678209220481222),
            bezier_geometry::Point2D(4451.2435499045341203,
                                     2615.6678209220481222),
            bezier_geometry::Point2D(4476.2435499045359393,
                                     2630.1015776517888298)),
        0,
        true,
        true,
        0};
    testInputs.push_back(entry);
  }
  { // Touching at neither curves' endpoint, stationary is NOT in path, curves
    // are basically horizontal mirror images with an overlap, causing extra
    // cases in solveSystem.
    ShiftAgainstTestEntry entry = {
        // bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(4411.9350024746709096,
        // 2621.1805812566844907),
        // bezier_geometry::Point2D(4461.9350024746690906,
        // 2621.1805812566844907),
        // bezier_geometry::Point2D(4436.9350024746709096,
        // 2606.7468245269428735)),
        // bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(4501.2435499045359393,
        // 2615.6678209220481222),
        // bezier_geometry::Point2D(4451.2435499045341203,
        // 2615.6678209220481222),
        // bezier_geometry::Point2D(4476.2435499045359393,
        // 2630.1015776517888298)),
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(441.19350024746709096,
                                     262.11805812566844907),
            bezier_geometry::Point2D(446.19350024746690906,
                                     262.11805812566844907),
            bezier_geometry::Point2D(443.69350024746709096,
                                     260.67468245269428735)),
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(450.12435499045359393,
                                     261.56678209220481222),
            bezier_geometry::Point2D(445.12435499045341203,
                                     261.56678209220481222),
            bezier_geometry::Point2D(447.62435499045359393,
                                     263.01015776517888298)),
        0,
        false,
        true,
        -1};
    testInputs.push_back(entry);
  }
  { // Not touching, stationary is in path, results in a near constant zero
    // function when computing intersection.
    ShiftAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(-900.00000000000102318,
                                     357.73502691896243277),
            bezier_geometry::Point2D(-500.00000000000096634,
                                     357.73502691896248962),
            bezier_geometry::Point2D(-700.00000000000090949,
                                     242.26497308103770933)),
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(-300.00000000000011369, 600),
            bezier_geometry::Point2D(-600, 299.99999999999988631),
            bezier_geometry::Point2D(-300.00000000000005684,
                                     299.99999999999994316)),
        0,
        true,
        true,
        100.00000000000102318
        // 99.999984928773716092
        // 99.999973978977929801
    };
    testInputs.push_back(entry);
  }
  { // Infinite initial intersection, stationary is not in path.
    ShiftAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                      bezier_geometry::Point2D(2, 0),
                                      bezier_geometry::Point2D(1, 0)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                      bezier_geometry::Point2D(2, 0),
                                      bezier_geometry::Point2D(1, 0)),
        0,
        true,
        true,
        -1};
    testInputs.push_back(entry);
  }
  { // Infinite initial intersection, stationary is in path.
    ShiftAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                      bezier_geometry::Point2D(3, 1),
                                      bezier_geometry::Point2D(2, 0)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                      bezier_geometry::Point2D(3, 1),
                                      bezier_geometry::Point2D(2, 0)),
        0,
        true,
        true,
        0};
    testInputs.push_back(entry);
  }
  { // Touching a concave edge, stationary is not in path at the point of
    // intersection.
    ShiftAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(565.41219520853178437,
                                     307.30101316320872229),
            bezier_geometry::Point2D(615.41219520853178437,
                                     307.30101316320872229),
            bezier_geometry::Point2D(590.41219520853178437,
                                     292.86725643346801462))
            .shift(0.000100009,
                   std::numeric_limits<bezier_geometry::RealNum>::infinity(),
                   true, true),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(600, 300),
                                      bezier_geometry::Point2D(300, 600),
                                      bezier_geometry::Point2D(300, 300)),
        -0.33042898601829939764, false, true,
        // 192.42516178838602059
        192.4253452378237057};
    testInputs.push_back(entry);
  }
  { // Touching a concave edge, stationary is in path, path is along concave
    // edge at a sufficiently different slope.
    ShiftAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(1180.0117108588779047,
                                     1170.15036601145448),
            bezier_geometry::Point2D(380.01171085887881418,
                                     370.15036601145436634),
            bezier_geometry::Point2D(380.01171085887881418,
                                     1170.15036601145448)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(600, 300),
                                      bezier_geometry::Point2D(300, 600),
                                      bezier_geometry::Point2D(300, 300)),
        -0.97770530189798787735,
        false,
        true,
        9.1638953743514264261};
    testInputs.push_back(entry);
  }
  { // Touching, stationary is in path, these curves result in some different
    // cases in the polynomial system solver.
    ShiftAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(3998.3524027459957324,
                                     2714.9999999999990905),
            bezier_geometry::Point2D(3198.3524027459948229,
                                     1914.9999999999990905),
            bezier_geometry::Point2D(3198.3524027459948229,
                                     2714.9999999999990905)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(0, 2715),
                                      bezier_geometry::Point2D(5280, 2715),
                                      bezier_geometry::Point2D(2640, 2715)),
        std::numeric_limits<bezier_geometry::RealNum>::infinity(),
        true,
        true,
        0};
    testInputs.push_back(entry);
  }
  { // Stationary is not in the path, causes the initial shift equation to
    // evaluate to a constant.
    ShiftAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(-1014.3698659563326601,
                                     -3853.2364338085130839),
            bezier_geometry::Point2D(-1368.1929176688854568,
                                     -3666.6640669456373871),
            bezier_geometry::Point2D(-1137.4225886967951737,
                                     -3657.8103333012054463)),
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(0, 0),
            bezier_geometry::Point2D(-2462.7552425899657464,
                                     -4670.4642826057061029),
            bezier_geometry::Point2D(-1231.3776212949828732,
                                     -2335.2321413028530515)),
        0,
        true,
        true,
        -1};
    testInputs.push_back(entry);
  }
  { // Stationary is not in path at the initial intersection, moving curve's
    // endpoint is just barely not a blocker because it is farther along the
    // path of the shift.
    ShiftAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(5179.9999925129068288,
                                     2815.0446737944694178),
            bezier_geometry::Point2D(5154.9999925129195617,
                                     2858.3459439836919955),
            bezier_geometry::Point2D(5179.9999925129068288,
                                     2843.9121872539603828)),
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(5080, 2915.0000000000004547),
            bezier_geometry::Point2D(5180, 2815.0000000000004547),
            bezier_geometry::Point2D(5180, 2915.0000000000004547)),
        -1.8896833,
        false,
        true,
        54.653448401274459911};
    testInputs.push_back(entry);
  }
  { // Stationary is in the path, it is the same curve, just shifted.
    ShiftAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                      bezier_geometry::Point2D(1, 3),
                                      bezier_geometry::Point2D(2, 2)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(2, 1),
                                      bezier_geometry::Point2D(2, 3),
                                      bezier_geometry::Point2D(3, 2)),
        0,
        true,
        true,
        1};
    testInputs.push_back(entry);
  }
  { // Stationary is not in path, curves are straight lines, one's slope is
    // almost infinite in the shift direction.
    ShiftAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(3971.7096231345235537,
                                     1315.0000000000002274),
            bezier_geometry::Point2D(3971.7096231345235537,
                                     2914.9999999999972715),
            bezier_geometry::Point2D(3971.7096231345235537,
                                     2115.0000000000013642)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(0, 2915),
                                      bezier_geometry::Point2D(5280, 2915),
                                      bezier_geometry::Point2D(2640, 2915)),
        /*std::numeric_limits<bezier_geometry::RealNum>::infinity()*/
        1.76216e+07, false, false, -1};
    testInputs.push_back(entry);
  }
  { // Touching, stationary is in the path, shift slope is almost right along
    // the slope of the curves at the intersection point.
    ShiftAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(5079.9999999999990905,
                                     2284.960546735096159),
            bezier_geometry::Point2D(5079.999999999998181,
                                     684.96054673509388522),
            bezier_geometry::Point2D(5079.999999999998181,
                                     1484.9605467350936578)),
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(5105, 1251.5282433985471471),
            bezier_geometry::Point2D(5079.999999999998181,
                                     1208.2269732093250241),
            bezier_geometry::Point2D(5079.999999999998181,
                                     1237.0944866688055299)),
        3102727.129890563,
        true,
        true,
        0};
    testInputs.push_back(entry);
  }
  { // Touching, stationary is in the path, shift slope is almost right along
    // the slope of the curves at the intersection point.
    ShiftAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(302.25424859373651998,
                                     345.62929061784905116),
            bezier_geometry::Point2D(302.25424859373651998,
                                     1945.6292906178482554),
            bezier_geometry::Point2D(302.25424859373651998,
                                     1145.6292906178489375)),
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(267.70509831248386945,
                                     852.44717418524339791),
            bezier_geometry::Point2D(302.25424859373651998,
                                     900.00000000000102318),
            bezier_geometry::Point2D(302.25424859373651998,
                                     863.67287359973295224)),
        7309429.2729546502233,
        false,
        false,
        0};
    testInputs.push_back(entry);
  }
  { // Touching, stationary is in the path, shift slope is almost right along
    // the slope of the curves at the intersection point.
    ShiftAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(201.12712429686803262, 100),
            bezier_geometry::Point2D(201.12712429686803262,
                                     1700.0000000000004547),
            bezier_geometry::Point2D(201.12712429686803262,
                                     900.00000000000011369)),
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(201.12712429686848736,
                                     447.73504352908366855),
            bezier_geometry::Point2D(166.57797401561560946,
                                     495.28786934384129381),
            bezier_geometry::Point2D(201.12712429686848736,
                                     484.06216992935173948)),
        7754004.7733616204932,
        true,
        true,
        -1};
    testInputs.push_back(entry);
  }
  { // Small overlap that does not consitute a block, matching block point
    // would
    // between the two intersection points.
    ShiftAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(1345.0377281293119722,
                                     955.78947368421063402),
            bezier_geometry::Point2D(2145.0377281293121996,
                                     2341.4301197393124312),
            bezier_geometry::Point2D(1745.0377281293119722,
                                     1648.6097967117616463)),
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(1432.6081372441283293,
                                     1135.2898866598613949),
            bezier_geometry::Point2D(1467.1572875253809798,
                                     1182.8427124746190202),
            bezier_geometry::Point2D(1467.1572875253809798,
                                     1146.5155860743509493)),
        1.7320503399034026248,
        true,
        true,
        -1};
    testInputs.push_back(entry);
  }
  { // Small overlap that just barely does not constitute a block (on the
    // scaled
    // up version) on the endpoint of a straight line.
    ShiftAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(1434.180233769628785,
                                     1110.1888225672339559),
            bezier_geometry::Point2D(1154.8331708655150578,
                                     695.50193050667553507),
            bezier_geometry::Point2D(1294.5067023175720351,
                                     902.84537653695474546)),
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(966.41749896002192698,
                                     299.99999999999977263),
            bezier_geometry::Point2D(1766.417498960021927,
                                     1685.6406460551017972),
            bezier_geometry::Point2D(1366.417498960021927,
                                     992.8203230275507849)),
        -0.67363369388420257788,
        false,
        true,
        -1};
    testInputs.push_back(entry);
  }
  { // Stationary is in the shift path, very close to intersecting at the
    // blocking point, large scale coordinates.
    ShiftAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(4148.4336266492609866,
                                     803.35955843259489484),
            bezier_geometry::Point2D(4148.4336266492609866,
                                     503.35955843259489484),
            bezier_geometry::Point2D(4648.4336266492609866,
                                     653.35955843259489484)),
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(4580, 863.43734137805040518),
            bezier_geometry::Point2D(4380, 517.02717986427501273),
            bezier_geometry::Point2D(4380, 747.96728754012542595)),
        -0.29217711363060960839,
        true,
        false,
        0.00035332337720710529135};
    testInputs.push_back(entry);
  }
  { // Stationary is in the shift path, very close to intersecting at the
    // blocking point, large scale coordinates, necessitates the higher number
    // of Newton iterations.
    ShiftAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(4513.7014080796670896,
                                     1135.586915575024932),
            bezier_geometry::Point2D(4513.7014080796670896,
                                     835.58691557502493197),
            bezier_geometry::Point2D(5013.7014080796670896,
                                     985.58691557502493197)),
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(4980, 850.56388373950790083),
            bezier_geometry::Point2D(4580, 850.56388373950790083),
            bezier_geometry::Point2D(4780, 966.03393757743333481)),
        -5.9608539964891242846,
        true,
        false,
        0.0094933821741537190858};
    testInputs.push_back(entry);
  }
  { // Stationary is in the shift path, very close to intersecting at the
    // blocking point, large scale coordinates, necessitates an even higher
    // number of Newton iterations.
    ShiftAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(4372.3350283822674101,
                                     1090.5292520887542196),
            bezier_geometry::Point2D(4372.3350283822674101,
                                     790.52925208875433327),
            bezier_geometry::Point2D(4872.3350283822674101,
                                     940.52925208875433327)),
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(4933.3231512770844347,
                                     850.5553499465133882),
            bezier_geometry::Point2D(4533.3231512770844347,
                                     850.5553499465133882),
            bezier_geometry::Point2D(4733.3231512770844347,
                                     966.02540378443882219)),
        -1.8437774025124344934,
        true,
        false,
        0.00011381676960696838672};
    testInputs.push_back(entry);
  }
  { // Stationary is in the shift path, necessitates a very large number of
    // Newton iterations.
    ShiftAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(4299.0628571428569558,
                                     1284.4571428571427987),
            bezier_geometry::Point2D(4299.0628571428569558,
                                     984.45714285714279868),
            bezier_geometry::Point2D(4799.0628571428569558,
                                     1134.4571428571427987)),
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(4886.0261440945896538,
                                     959.72505588774765783),
            bezier_geometry::Point2D(4486.0261440945896538,
                                     959.72505588774765783),
            bezier_geometry::Point2D(4686.0261440945896538,
                                     1075.1951097256730918)),
        -2.125,
        true,
        false,
        87.122821902779946868};
    testInputs.push_back(entry);
  }
  { // Stationary is in the shift path, necessitates a very large number of
    // Newton iterations.
    ShiftAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(4448.4116317011757928,
                                     1107.9018918231890893),
            bezier_geometry::Point2D(4448.4116317011757928,
                                     807.90189182318908934),
            bezier_geometry::Point2D(4948.4116317011757928,
                                     957.90189182318908934)),
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(4975.7434466632075782,
                                     850.55534994651372926),
            bezier_geometry::Point2D(4575.7434466632075782,
                                     850.55534994651350189),
            bezier_geometry::Point2D(4775.7434466632075782,
                                     966.02540378443904956)),
        -2.02925886186352189,
        true,
        false,
        0.00032461460038396143827};
    testInputs.push_back(entry);
  }
  { // Stationary barely does not block moving, moving is inside concavity of
    // stationary.  Tests the 'nearly at crit' logic.
    ShiftAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(110.0524080892043628,
                                     2868.3454995809943284),
            bezier_geometry::Point2D(310.01198364050753753,
                                     2864.324536345312481),
            bezier_geometry::Point2D(210.03219586485590753,
                                     2866.3350179631534047)),
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(100.00000000000014211, 2815),
            bezier_geometry::Point2D(200.00000000000014211, 2915),
            bezier_geometry::Point2D(100.00000000000017053, 2915)),
        2.1539802557175136144,
        true,
        true,
        0.98322197385164689099};
    testInputs.push_back(entry);
  }
  for (std::vector<ShiftAgainstTestEntry>::iterator i = testInputs.begin();
       i != testInputs.end(); i++) {
    for (int j = 0; j < 5; j++) {
      for (int k = 0; k < 2; k++) {
        bool multiplierSet = false;
        bezier_geometry::BezierCurveQ moving(i->moving);
        bezier_geometry::BezierCurveQ stationary(i->stationary);
        bool right = i->right;
        bool up = i->up;
        if (k == 1) {
          bezier_geometry::BezierCurveQ temp(moving);
          moving = stationary;
          stationary = temp;
          right = !right;
          up = !up;
        }
        bezier_geometry::RealNum expectedDistance = i->expectedDistance;
        if (j == 1 || j == 3) {
          moving = bezier_geometry::BezierCurveQ(
              moving.valueAt(1), moving.valueAt(0), moving.getControl());
        } else if (j == 2 || j == 3) {
          stationary = bezier_geometry::BezierCurveQ(stationary.valueAt(1),
                                                     stationary.valueAt(0),
                                                     stationary.getControl());
        } else if (j == 4) {
          multiplierSet = true;
          const bezier_geometry::RealNum multiplier = 10000;
          moving = bezier_geometry::BezierCurveQ(
              bezier_geometry::Point2D(moving.valueAt(0).getX() * multiplier,
                                       moving.valueAt(0).getY() * multiplier),
              bezier_geometry::Point2D(moving.valueAt(1).getX() * multiplier,
                                       moving.valueAt(1).getY() * multiplier),
              bezier_geometry::Point2D(moving.getControl().getX() * multiplier,
                                       moving.getControl().getY() *
                                           multiplier));
          stationary = bezier_geometry::BezierCurveQ(
              bezier_geometry::Point2D(
                  stationary.valueAt(0).getX() * multiplier,
                  stationary.valueAt(0).getY() * multiplier),
              bezier_geometry::Point2D(
                  stationary.valueAt(1).getX() * multiplier,
                  stationary.valueAt(1).getY() * multiplier),
              bezier_geometry::Point2D(
                  stationary.getControl().getX() * multiplier,
                  stationary.getControl().getY() * multiplier));
          expectedDistance = expectedDistance == -1
                                 ? expectedDistance
                                 : expectedDistance * multiplier;
        }
        bezier_geometry::BezierCurveQ::ShiftAgainstResult result;
        try {
          debug() << "** BEFORE shiftAgainst ** moving:" << moving
                  << "stationary:" << stationary << "slope:" << i->slope
                  << "right:" << right << "up:" << up << std::endl;
          moving.shiftAgainst(stationary, i->slope, right, up, result);
        } catch (const std::string &msg) {
          throw msg;
        }
        debug() << "^"
                << "expected:" << bezier_geometry::toString(expectedDistance)
                << "actual:" << bezier_geometry::toString(result.distance)
                << "shiftParam:" << bezier_geometry::toString(result.param)
                << "inputShiftParam:"
                << bezier_geometry::toString(result.inputParam)
                << "value at shiftParam:"
                << (result.param >= 0 ? moving.valueAt(result.param).toString()
                                      : std::string())
                << "value at inputShiftParam:"
                << (result.inputParam >= 0
                        ? stationary.valueAt(result.inputParam).toString()
                        : std::string())
                << "blockedCWVerticalAngleStart:"
                << bezier_geometry::toString(result.blockedCWVerticalAngleStart)
                << "blockedCWVerticalAngleEnd:"
                << bezier_geometry::toString(result.blockedCWVerticalAngleEnd)
                << "shifted intersection:"
                << (result.distance >= 0
                        ? moving.shift(result.distance, i->slope, right, up)
                              .pointsOfIntersection(stationary)
                        : bezier_geometry::StaticVector<
                              std::pair<bezier_geometry::RealNum,
                                        bezier_geometry::RealNum>,
                              4>())
                << std::endl;
        CHECK(bezier_geometry::sufficientlyClose(result.distance,
                                                 expectedDistance),
              true);
        CHECK(result.blockedCWVerticalAngleStart < 0,
              result.blockedCWVerticalAngleEnd < 0);
        if (result.distance != -1) {
          {
            CHECK(result.blockedCWVerticalAngleStart >= 0, true);
            CHECK(result.blockedCWVerticalAngleEnd >= 0, true);
            CHECK(result.blockedCWVerticalAngleStart <= 360.0, true);
            CHECK(result.blockedCWVerticalAngleEnd <= 360.0, true);
            CHECK(bezier_geometry::sufficientlyClose(
                      result.blockedCWVerticalAngleStart,
                      result.blockedCWVerticalAngleEnd),
                  false);
            const bezier_geometry::Point2D movingBlockedPoint(
                moving.valueAt(result.param));
            const bezier_geometry::Point2D positiveVerticalPoint(
                movingBlockedPoint.shift(
                    100,
                    std::numeric_limits<bezier_geometry::RealNum>::infinity(),
                    true, true));
            const bezier_geometry::Point2D movingDirectionPoint(
                movingBlockedPoint.shift(100, i->slope, right, up));
            const bezier_geometry::Point2D startCWBlockedAnglePoint(
                positiveVerticalPoint.rotate(
                    movingBlockedPoint,
                    (-1.0) * result.blockedCWVerticalAngleStart));
            const bezier_geometry::Point2D endCWBlockedAnglePoint(
                positiveVerticalPoint.rotate(
                    movingBlockedPoint,
                    (-1.0) * result.blockedCWVerticalAngleEnd));
            const bezier_geometry::RealNum blockedAngleInterval =
                bezier_geometry::Point2D::getAngleBetween(
                    startCWBlockedAnglePoint, endCWBlockedAnglePoint,
                    movingBlockedPoint, true);
            if (startCWBlockedAnglePoint !=
                endCWBlockedAnglePoint) { // Start and end points would be
                                          // equal in a 360 degree interval.
              debug() << "Testing blocked angles -"
                      << "blockedCWVerticalAngleStart:"
                      << bezier_geometry::toString(
                             result.blockedCWVerticalAngleStart)
                      << "blockedCWVerticalAngleEnd:"
                      << bezier_geometry::toString(
                             result.blockedCWVerticalAngleEnd)
                      << std::endl;
              CHECK(bezier_geometry::Point2D::getAngleBetween(
                        startCWBlockedAnglePoint, movingDirectionPoint,
                        movingBlockedPoint, true) <= blockedAngleInterval,
                    true);
            } else { // TODO - add this same test for rotateAgainst.
              CHECK(result.distance, 0.0);
            }
          }
          if (result.distance > 0) {
            if (!bezier_geometry::sufficientlySmall(result.distance)) {
              CHECK(
                  bezier_geometry::sufficientlyClose(
                      moving.valueAt(result.param)
                          .distanceFrom(stationary.valueAt(result.inputParam)),
                      result.distance),
                  true);
              CHECK(bezier_geometry::sufficientlyCloseSlopes(
                        bezier_geometry::Point2D::getSlopeBetween(
                            moving.valueAt(result.param),
                            stationary.valueAt(result.inputParam)),
                        i->slope),
                    true);
            }
          } else {
            if (!multiplierSet) {
              /*
              A direct point comparison is not used here because intersections
              will be reported when the curves come sufficiently close to each
              other.  A direct comparison can fail when small calculation errors
              are added to the small distance between the curves.
              */
              CHECK(bezier_geometry::sufficientlyClose(
                        moving.valueAt(result.param).getX(),
                        stationary.valueAt(result.inputParam).getX()),
                    true);
              CHECK(bezier_geometry::sufficientlyClose(
                        moving.valueAt(result.param).getY(),
                        stationary.valueAt(result.inputParam).getY()),
                    true);
            }
          }
          bezier_geometry::StaticVector<
              std::pair<bezier_geometry::RealNum, bezier_geometry::RealNum>, 4>
              initialIntersection(moving.pointsOfIntersection(stationary));
          if (!bezier_geometry::BezierCurveQ::isIntersectionInfinite(
                  initialIntersection) &&
              initialIntersection.size() ==
                  0) { // These curves were not touching before the shift.
            CHECK(result.distance > 0, true);
          }
        }
      }
    }
  }
  TEST_END
}

typedef struct {
  bezier_geometry::BezierCurveQ moving;
  bezier_geometry::BezierCurveQ stationary;
  bezier_geometry::Point2D fulcrum;
  bool clockwise;
  bezier_geometry::RealNum expectedAngle;
} RotateAgainstTestEntry;

void rotateAgainstTest() {
  TEST_START
  std::vector<RotateAgainstTestEntry> testInputs;
  { // Touching at their farther points, stationary is behind the direction of
    // rotation.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(5, 5),
                                      bezier_geometry::Point2D(6, 1),
                                      bezier_geometry::Point2D(5.5, 3)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(5, 5),
                                      bezier_geometry::Point2D(5, 1),
                                      bezier_geometry::Point2D(5, 3)),
        bezier_geometry::Point2D(5, -5), true, 350.537677792};
    testInputs.push_back(entry);
  }
  { // Touching at their farther points, stationary is in the direction of
    // rotation.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(5, 5),
                                      bezier_geometry::Point2D(6, 1),
                                      bezier_geometry::Point2D(5.5, 3)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(5, 5),
                                      bezier_geometry::Point2D(5, 1),
                                      bezier_geometry::Point2D(5, 3)),
        bezier_geometry::Point2D(5, -5), false, 0};
    testInputs.push_back(entry);
  }
  { // Touching at their closer points, stationary is in the direction of
    // rotation.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(5, 5),
                                      bezier_geometry::Point2D(5, 1),
                                      bezier_geometry::Point2D(5, 3)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(5, 5),
                                      bezier_geometry::Point2D(6, 1),
                                      bezier_geometry::Point2D(5.5, 3)),
        bezier_geometry::Point2D(5, 10), false, 0};
    testInputs.push_back(entry);
  }
  { // Touching at their closer points, stationary is not in the direction of
    // rotation.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(5, 5),
                                      bezier_geometry::Point2D(5, 1),
                                      bezier_geometry::Point2D(5, 3)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(5, 5),
                                      bezier_geometry::Point2D(6, 1),
                                      bezier_geometry::Point2D(5.5, 3)),
        bezier_geometry::Point2D(5, 10), true, 353.70745860480906231};
    testInputs.push_back(entry);
  }
  { // Crossing.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                      bezier_geometry::Point2D(3, 3),
                                      bezier_geometry::Point2D(2, 2)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 3),
                                      bezier_geometry::Point2D(3, 1),
                                      bezier_geometry::Point2D(2, 2)),
        bezier_geometry::Point2D(2, -5), false, 0};
    testInputs.push_back(entry);
  }
  { // Touching at neither curves' endpoint, not crossing, stationary is not in
    // the direction of rotation.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                      bezier_geometry::Point2D(3, 3),
                                      bezier_geometry::Point2D(2, 2)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(0, 1),
                                      bezier_geometry::Point2D(2, 3),
                                      bezier_geometry::Point2D(3, 2)),
        bezier_geometry::Point2D(2, -5), true, 348.066490258535};
    testInputs.push_back(entry);
  }
  { // Touching at neither curves' endpoint, not crossing, stationary is in the
    // direction of rotation.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                      bezier_geometry::Point2D(3, 3),
                                      bezier_geometry::Point2D(2, 2)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(0, 1),
                                      bezier_geometry::Point2D(2, 3),
                                      bezier_geometry::Point2D(3, 2)),
        bezier_geometry::Point2D(2, -5), false, 0};
    testInputs.push_back(entry);
  }
  { // Touching at their closer points, stationary is in the direction of
    // rotation, touching at the fulcrum.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(5, 5),
                                      bezier_geometry::Point2D(5, 1),
                                      bezier_geometry::Point2D(5, 3)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(5, 5),
                                      bezier_geometry::Point2D(6, 1),
                                      bezier_geometry::Point2D(5.5, 3)),
        bezier_geometry::Point2D(5, 5), false, 14.03624346792648403};
    testInputs.push_back(entry);
  }
  { // Touching at their closer points, stationary is not in the direction of
    // rotation, touching at the fulcrum.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(5, 5),
                                      bezier_geometry::Point2D(5, 1),
                                      bezier_geometry::Point2D(5, 3)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(5, 5),
                                      bezier_geometry::Point2D(6, 1),
                                      bezier_geometry::Point2D(5.5, 3)),
        bezier_geometry::Point2D(5, 5), true, 345.96375653207348932};
    testInputs.push_back(entry);
  }
  { // Touching at their closer points, stationary is in the direction of
    // rotation, touching at the fulcrum, farther point is a smaller angle than
    // closer.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(5, 5),
                                      bezier_geometry::Point2D(5.1, 1),
                                      bezier_geometry::Point2D(5, 3)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(5, 5),
                                      bezier_geometry::Point2D(6, 1),
                                      bezier_geometry::Point2D(5.5, 3)),
        bezier_geometry::Point2D(5, 5), false, 12.604147283761845344};
    testInputs.push_back(entry);
  }
  { // Crossing at the fulcrum.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                      bezier_geometry::Point2D(3, 3),
                                      bezier_geometry::Point2D(2, 2)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 3),
                                      bezier_geometry::Point2D(3, 1),
                                      bezier_geometry::Point2D(2, 2)),
        bezier_geometry::Point2D(2, 2), false, 90};
    testInputs.push_back(entry);
  }
  { // Crossing at the fulcrum.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                      bezier_geometry::Point2D(3, 3),
                                      bezier_geometry::Point2D(2, 2)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 3),
                                      bezier_geometry::Point2D(3, 1),
                                      bezier_geometry::Point2D(2, 2)),
        bezier_geometry::Point2D(2, 2), true, 90};
    testInputs.push_back(entry);
  }
  { // Not touching, curving right moving against a vertical line, fulcrum is
    // very far away to make it almost a shift.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                      bezier_geometry::Point2D(1, 3),
                                      bezier_geometry::Point2D(2, 2)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(2, 1),
                                      bezier_geometry::Point2D(2, 3),
                                      bezier_geometry::Point2D(2, 2)),
        bezier_geometry::Point2D(2, -100), true, 0.28085266628351557294};
    testInputs.push_back(entry);
  }
  { // Not touching, curving right moving against an equivalent right curve,
    // fulcrum is very far away to make it almost a shift.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                      bezier_geometry::Point2D(1, 3),
                                      bezier_geometry::Point2D(2, 2)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(2, 1),
                                      bezier_geometry::Point2D(2, 3),
                                      bezier_geometry::Point2D(3, 2)),
        bezier_geometry::Point2D(2, -100), true, 0.55361018910656933922};
    testInputs.push_back(entry);
  }
  { // Not touching, curving right moving against a left curve, fulcrum is very
    // far away to make it almost a shift.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                      bezier_geometry::Point2D(1, 3),
                                      bezier_geometry::Point2D(2, 2)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(2, 1),
                                      bezier_geometry::Point2D(2, 3),
                                      bezier_geometry::Point2D(1.7, 2)),
        bezier_geometry::Point2D(2, -100), true, 0.19659993188662500119};
    testInputs.push_back(entry);
  }
  { // Not touching, slanting line moving against a vertical line, fulcrum is
    // very far away to make it almost a shift.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                      bezier_geometry::Point2D(3, 3),
                                      bezier_geometry::Point2D(2, 2)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(5, -100),
                                      bezier_geometry::Point2D(5, 100),
                                      bezier_geometry::Point2D(5, 0)),
        bezier_geometry::Point2D(2, -1000), true, 0.11424900186437306204};
    testInputs.push_back(entry);
  }
  { // Not touching, slanting line moving against a vertical line, fulcrum is
    // positioned so rotation does not result in intersection.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                      bezier_geometry::Point2D(3, 3),
                                      bezier_geometry::Point2D(2, 2)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(5, -100),
                                      bezier_geometry::Point2D(5, 100),
                                      bezier_geometry::Point2D(5, 0)),
        bezier_geometry::Point2D(-100, 0), true, -1};
    testInputs.push_back(entry);
  }
  { // Not touching, slanting line moving against a vertical line, fulcrum is
    // positioned so rotation does not result in intersection.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                      bezier_geometry::Point2D(3, 3),
                                      bezier_geometry::Point2D(2, 2)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(5, -100),
                                      bezier_geometry::Point2D(5, 100),
                                      bezier_geometry::Point2D(5, 0)),
        bezier_geometry::Point2D(0, 0), true, -1};
    testInputs.push_back(entry);
  }
  { // Not touching, slanting line moving against a vertical line.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                      bezier_geometry::Point2D(3, 3),
                                      bezier_geometry::Point2D(2, 2)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(5, -100),
                                      bezier_geometry::Point2D(5, 100),
                                      bezier_geometry::Point2D(5, 0)),
        bezier_geometry::Point2D(100, 0), true, 10.015638955823490619};
    testInputs.push_back(entry);
  }
  { // Touching at ends, touch does not result in the stationary blocking at
    // that point, does block at another point.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                      bezier_geometry::Point2D(3, 3),
                                      bezier_geometry::Point2D(2, 4)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(3, 3),
                                      bezier_geometry::Point2D(5, 5),
                                      bezier_geometry::Point2D(4, 2)),
        bezier_geometry::Point2D(3, -100), true, 0.55359096161619214538};
    testInputs.push_back(entry);
  }
  { // Not touching, one curve is inside the other, fulcrum is inside both
    // curves.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(9, 9),
                                      bezier_geometry::Point2D(11, 9),
                                      bezier_geometry::Point2D(10, 20)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(8.9, 8),
                                      bezier_geometry::Point2D(11.1, 8),
                                      bezier_geometry::Point2D(10, 80)),
        bezier_geometry::Point2D(10, 9), true, 7.0684552764196970642};
    testInputs.push_back(entry);
  }
  { // Not touching, one curve is inside the other, fulcrum is inside both
    // curves.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(9, 9),
                                      bezier_geometry::Point2D(11, 9),
                                      bezier_geometry::Point2D(10, 20)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(8.9, 8),
                                      bezier_geometry::Point2D(11.1, 8),
                                      bezier_geometry::Point2D(10, 80)),
        bezier_geometry::Point2D(10, 10), true, 4.9909578389853912483};
    testInputs.push_back(entry);
  }
  { // Not touching, one curve is inside the other, fulcrum is inside both
    // curves.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(9, 9),
                                      bezier_geometry::Point2D(11, 9),
                                      bezier_geometry::Point2D(10, 20)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(8, 8),
                                      bezier_geometry::Point2D(12, 8),
                                      bezier_geometry::Point2D(10, 80)),
        bezier_geometry::Point2D(10, 10), true,
        22.601938509327851534}; // 0.600475,0.957407
    testInputs.push_back(entry);
  }
  { // Not touching, one curve is inside the other, fulcrum is outside both
    // curves.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(9, 9),
                                      bezier_geometry::Point2D(11, 9),
                                      bezier_geometry::Point2D(10, 20)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(8, 8),
                                      bezier_geometry::Point2D(12, 8),
                                      bezier_geometry::Point2D(10, 80)),
        bezier_geometry::Point2D(10, 8), true, 14.900385073238837208};
    testInputs.push_back(entry);
  }
  { // Infinite initial intersection, stationary does not block.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 10),
                                      bezier_geometry::Point2D(2, 9),
                                      bezier_geometry::Point2D(1, 9)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 10),
                                      bezier_geometry::Point2D(2, 9),
                                      bezier_geometry::Point2D(1, 9)),
        bezier_geometry::Point2D(2, 0), true, 360};
    testInputs.push_back(entry);
  }
  { // Infinite initial intersection, stationary does block.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 10),
                                      bezier_geometry::Point2D(2, 9),
                                      bezier_geometry::Point2D(1, 9)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 10),
                                      bezier_geometry::Point2D(2, 9),
                                      bezier_geometry::Point2D(1, 9)),
        bezier_geometry::Point2D(1.9, 9.9), true, 0};
    testInputs.push_back(entry);
  }
  { // No initial intersection (just barely), stationary blocks, angle formed
    // with endpoint is extrememly small.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(2, 2),
                                      bezier_geometry::Point2D(5, 2),
                                      bezier_geometry::Point2D(3, 2)),
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(3, 0),
            bezier_geometry::Point2D(3, 1.99989999999999),
            bezier_geometry::Point2D(3, 1)),
        bezier_geometry::Point2D(-100, 0), true, 0.000055626481721375089248};
    testInputs.push_back(entry);
  }
  { // No initial intersection, stationary blocks, infinite intersection at
    // blocking point.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(5, 5),
                                      bezier_geometry::Point2D(5, 9),
                                      bezier_geometry::Point2D(5, 7)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(6, 4),
                                      bezier_geometry::Point2D(8, 4),
                                      bezier_geometry::Point2D(7, 4)),
        bezier_geometry::Point2D(5, 4), true, 90};
    testInputs.push_back(entry);
  }
  { // No initial intersection, stationary blocks, curves are differently
    // shaped, different (non-endpoint) parameters for both curves.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(0, 5),
                                      bezier_geometry::Point2D(0, 7),
                                      bezier_geometry::Point2D(1, 6)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 0),
                                      bezier_geometry::Point2D(2, -2),
                                      bezier_geometry::Point2D(20, -1)),
        bezier_geometry::Point2D(0, 0), true, 88.1412};
    testInputs.push_back(entry);
  }
  { // Infinite initial intersection, stationary does not block initially.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(1667.1568745401109481,
                                     1510.9777556651965824),
            bezier_geometry::Point2D(1667.1575038319786017,
                                     1010.97775566559244),
            bezier_geometry::Point2D(1667.1571891860448886,
                                     1260.9777556653943975)),
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(1667.1571211945270079, 1315),
            bezier_geometry::Point2D(1667.1571211945270079, 2915),
            bezier_geometry::Point2D(1667.1571211945270079, 2115)),
        bezier_geometry::Point2D(1567.1572043599537665, 1248.9213562373097375),
        false, 66.912273810348807501};
    testInputs.push_back(entry);
  }
  { // Infinite initial intersection, stationary does block initially.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(1467.1572875253809798, 1500),
            bezier_geometry::Point2D(1667.1572875253809798, 1500),
            bezier_geometry::Point2D(1567.1572875253809798, 1500)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1700, 1500),
                                      bezier_geometry::Point2D(1500, 1500),
                                      bezier_geometry::Point2D(1600, 1500)),
        bezier_geometry::Point2D(1567.1572875253809798, 1116.4213555373030431),
        true, 0};
    testInputs.push_back(entry);
  }
  { // No initial intersection, stationary lies in rotation, this case fails
    // for
    // the older implementation.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(1429.5381960962952235,
                                     892.40284227496658787),
            bezier_geometry::Point2D(1494.7774157626508895,
                                     1388.128415770348056),
            bezier_geometry::Point2D(1462.1578059294731702,
                                     1140.2656290226573219)),
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(1467.1572875253809798,
                                     1182.8427124746190202),
            bezier_geometry::Point2D(1432.6081372441283293,
                                     1230.3955382893766455),
            bezier_geometry::Point2D(1467.1572875253809798,
                                     1219.1698388748868638)),
        bezier_geometry::Point2D(1577.7458854690471526, 1252.1610050911958751),
        false,
        61.375027936068363488 // angle: 61.375 my: 0.924485 input: 0.721985
    };
    testInputs.push_back(entry);
  }
  { // Initial intersection does not block, small angle to blocking
    // intersection, very close to initial.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(1026.1734958109443596,
                                     1779.0108718550197864),
            bezier_geometry::Point2D(1480.2248968443257127,
                                     1988.3845216288721076),
            bezier_geometry::Point2D(1253.1991963276348088,
                                     1883.697696741945947)),
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(1169.2084678792318755,
                                     2159.0423267265186951),
            bezier_geometry::Point2D(1169.2084678792318755,
                                     1859.0423267265184677),
            bezier_geometry::Point2D(1669.2084678792321029,
                                     2009.0423267265186951)),
        bezier_geometry::Point2D(1355.1409416305482409, 1820.5857128203160755),
        true, 0.022455242055362204662
        // 0.011229674380666847255
    };
    // The distance crit condition doesn't scale to large numbers so the result
    // is a bit different.
    // testInputs.push_back(entry);
  }
  { // No initial intersection, stationary lies in rotation, this case fails
    // for
    // the older implementation due to another equation solution very close to
    // the right one.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(1138.3374723658384937,
                                     1824.5620592402849525),
            bezier_geometry::Point2D(1586.0725779447343484,
                                     2047.1227818133731944),
            bezier_geometry::Point2D(1362.2050251552864211,
                                     1935.8424205268290734)),
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(1264.3932127954708449,
                                     2206.7565445554882899),
            bezier_geometry::Point2D(1264.3932127954708449,
                                     1906.7565445554882899),
            bezier_geometry::Point2D(1764.3932127954708449,
                                     2056.7565445554882899)),
        bezier_geometry::Point2D(1409.5631903471721671, 1847.7101029053669663),
        true, 8.8031188421994563953};
    testInputs.push_back(entry);
  }
  { // No initial intersection, stationary lies in the path of rotation,
    // blocking point is the endpoint of the moving curve, requires a higher
    // number of Newton iterations.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(1438.4248538438291689,
                                     1980.3900700994627186),
            bezier_geometry::Point2D(1635.4593200783472184,
                                     2014.7036113868105076),
            bezier_geometry::Point2D(1536.94208696108808,
                                     1997.5468407431364994)),
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(1637.9428571428572923,
                                     2304.2514285714282778),
            bezier_geometry::Point2D(1637.9428571428572923,
                                     2004.2514285714285052),
            bezier_geometry::Point2D(2137.9428571428575196,
                                     2154.2514285714287325)),
        bezier_geometry::Point2D(1564.9645881079222818, 1836.6366475985278157),
        true, 4.8964711587413347615};
    testInputs.push_back(entry);
  }
  { // Curve lies in the path of rotation, solution point is an odd case where
    // there is another point that results in a slightly smaller angle but the
    // newton method never arrives at it.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(1508.89370735534294,
                                     2039.0612273624315094),
            bezier_geometry::Point2D(1318.9677712276238708,
                                     1576.5375644153239136),
            bezier_geometry::Point2D(1413.9307392914834054,
                                     1807.7993958888778252)),
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(1467.1572875253809798,
                                     1682.8427124746190202),
            bezier_geometry::Point2D(1432.6081372441283293,
                                     1730.3955382893766455),
            bezier_geometry::Point2D(1467.1572875253809798,
                                     1719.1698388748868638)),
        bezier_geometry::Point2D(1346.6623751091401573, 1907.2423165504465032),
        true,
        100.45401203469806717 // 100.45397608
                              // const bezier_geometry::RealNum myCorrectParam
        // = 8.7275853969909781204e-5; const bezier_geometry::RealNum
        // inputCorrectParam = 0.76722904288484927715;
    };
    testInputs.push_back(entry);
  }
  { // Rotation is blocked by an intersection with the stationary curve, the
    // intersection would not be detected for a shift block test.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 3),
                                      bezier_geometry::Point2D(3, 3),
                                      bezier_geometry::Point2D(2, 3)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(3, 5),
                                      bezier_geometry::Point2D(3, 3),
                                      bezier_geometry::Point2D(3, 4)),
        bezier_geometry::Point2D(3, 0), true, 0};
    testInputs.push_back(entry);
  }
  { // Rotation is blocked by an intersection with the stationary curve, the
    // intersection would not be detected for a shift block test.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 2.99),
                                      bezier_geometry::Point2D(3, 3),
                                      bezier_geometry::Point2D(2, 3)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(3, 5),
                                      bezier_geometry::Point2D(3, 3),
                                      bezier_geometry::Point2D(3, 4)),
        bezier_geometry::Point2D(3, 0), true, 0};
    testInputs.push_back(entry);
  }
  { // Rotation is blocked by an intersection with the stationary curve, the
    // intersection would not be detected for a shift block test.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 2.99),
                                      bezier_geometry::Point2D(3, 3),
                                      bezier_geometry::Point2D(2, 3)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 3.01),
                                      bezier_geometry::Point2D(3, 3),
                                      bezier_geometry::Point2D(2, 3)),
        bezier_geometry::Point2D(3, 0), true, 0};
    testInputs.push_back(entry);
  }
  { // Rotation is blocked by an intersection with the stationary curve, the
    // intersection would not be detected for a shift block test.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 2.99),
                                      bezier_geometry::Point2D(3, 3),
                                      bezier_geometry::Point2D(2, 3)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 3),
                                      bezier_geometry::Point2D(3, 3),
                                      bezier_geometry::Point2D(2, 3)),
        bezier_geometry::Point2D(3, 0), true, 0};
    testInputs.push_back(entry);
  }
  { // Rotation is not blocked by an intersection with the stationary curve, an
    // intersection would be detected by a shift block test.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 2.99),
                                      bezier_geometry::Point2D(3, 3),
                                      bezier_geometry::Point2D(2, 3)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 2.98),
                                      bezier_geometry::Point2D(3, 3),
                                      bezier_geometry::Point2D(2, 3)),
        bezier_geometry::Point2D(3, 0), true, 359.70807112830652841};
    testInputs.push_back(entry);
  }
  { // Rotation is blocked by an intersection with the stationary curve and not
    // on an endpoint, the intersection would not be detected for a shift block
    // test.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 3),
                                      bezier_geometry::Point2D(3, 3),
                                      bezier_geometry::Point2D(2, 3)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(2, 5),
                                      bezier_geometry::Point2D(2, 3),
                                      bezier_geometry::Point2D(2, 4)),
        bezier_geometry::Point2D(2, 0), true, 0};
    testInputs.push_back(entry);
  }
  { // Rotation is blocked by an intersection with the stationary curve and not
    // at an endpoint of either curve, the intersection would not be detected
    // for a shift block test.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 2.99),
                                      bezier_geometry::Point2D(3, 2.99),
                                      bezier_geometry::Point2D(2, 3.01)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 3),
                                      bezier_geometry::Point2D(3, 3),
                                      bezier_geometry::Point2D(2, 3)),
        bezier_geometry::Point2D(2, 0), true, 0};
    testInputs.push_back(entry);
  }
  { // Rotation is blocked by an intersection with the stationary curve and at
    // an endpoint of the stationary curve, the intersection would not be
    // detected for a shift block test.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 2.99),
                                      bezier_geometry::Point2D(3, 2.99),
                                      bezier_geometry::Point2D(2, 3.01)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(2, 3),
                                      bezier_geometry::Point2D(3, 3),
                                      bezier_geometry::Point2D(2.5, 3)),
        bezier_geometry::Point2D(2, 0), true, 0};
    testInputs.push_back(entry);
  }
  { // Rotation is not blocked, stationary lies in the path of rotation, the
    // block would not be detected by a shift block test.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                      bezier_geometry::Point2D(1, 3),
                                      bezier_geometry::Point2D(1, 2)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(2, 4),
                                      bezier_geometry::Point2D(4, 4),
                                      bezier_geometry::Point2D(3, 4)),
        bezier_geometry::Point2D(2, 3), true, 90};
    testInputs.push_back(entry);
  }
  { // Stationary is in the path of rotation, the rotated moving curve
    // intersects at both endpoints, only one of which blocks further rotation.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(1667.0301038605282429,
                                     2036.2903840731962646),
            bezier_geometry::Point2D(1667.2100478305642355,
                                     1536.2904164530300477),
            bezier_geometry::Point2D(1667.1200758455463529,
                                     1786.2904002631132698)),
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(1667.1616936713262476,
                                     1970.6493396695125284),
            bezier_geometry::Point2D(1667.1616936713262476,
                                     1670.6493396695125284),
            bezier_geometry::Point2D(2167.1616936713262476,
                                     1820.6493396695125284)),
        bezier_geometry::Point2D(1567.1594881260464263, 1676.7597671794060261),
        true, 0.021050046076685965946};
    testInputs.push_back(entry);
  }
  { // Stationary blocks the rotation by 'distance concavity'.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(5, 1),
                                      bezier_geometry::Point2D(6, 2),
                                      bezier_geometry::Point2D(6, 1)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(4, 1),
                                      bezier_geometry::Point2D(6, 1),
                                      bezier_geometry::Point2D(5, 1)),
        bezier_geometry::Point2D(5, 0), true, 0};
    testInputs.push_back(entry);
  }
  { // Not touching initially, stationary blocks, but the endpoint that the
    // moving is blocked at requires sufficient Newton iterations.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(1375.192755399556745,
                                     1301.1355522482097058),
            bezier_geometry::Point2D(1737.7320144156258266,
                                     1645.4686550310811981),
            bezier_geometry::Point2D(1556.4623849075912858,
                                     1473.3021036396455656)),
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(1817.9422255423112347,
                                     1861.0217587498466401),
            bezier_geometry::Point2D(1817.942225542311462,
                                     1561.0217587498466401),
            bezier_geometry::Point2D(2317.942225542311462,
                                     1711.0217587498466401)),
        bezier_geometry::Point2D(1636.5164599264351182, 1389.015275553732863),
        true, 27.602601960926648417};
    testInputs.push_back(entry);
  }
  { // Not touching initially, stationary blocks, but the endpoint that the
    // moving is blocked at requires sufficient Newton iterations.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(1425.7667623018717222,
                                     1273.3031089955011339),
            bezier_geometry::Point2D(1864.1806653298397123,
                                     1513.7053752738859203),
            bezier_geometry::Point2D(1644.9737138158557173,
                                     1393.5042421346934134)),
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(1900.4079360354392065,
                                     1747.7313152431925118),
            bezier_geometry::Point2D(1900.4079360354389792,
                                     1447.7313152431925118),
            bezier_geometry::Point2D(2400.4079360354389792,
                                     1597.7313152431925118)),
        bezier_geometry::Point2D(1680.8242589607318678, 1328.124752723204665),
        true, 17.261201051373500803};
    testInputs.push_back(entry);
  }
  // Removed because the scaled up versions don't land on distance crits.  This
  // is ok.
  //  { // Blocked on initial intersection, is a border case when calculating
  //  distance crits.
  //    RotateAgainstTestEntry entry = {
  //      bezier_geometry::BezierCurveQ(
  //        bezier_geometry::Point2D(1558.0564326098453876,
  //        1189.6346510659582236),
  //        bezier_geometry::Point2D(1367.8452475284218508,
  //        1251.4384136530406977),
  //        bezier_geometry::Point2D(1462.9508400691336192,
  //        1220.5365323594994607)),
  //      bezier_geometry::BezierCurveQ(
  //        bezier_geometry::Point2D(1467.1572875253809798,
  //        1182.8427124746190202),
  //        bezier_geometry::Point2D(1432.6081372441283293,
  //        1230.3955382893766455),
  //        bezier_geometry::Point2D(1467.1572875253809798,
  //        1219.1698388748868638)),
  //      bezier_geometry::Point2D(1509.8730932090018086, 1468.156188305233627),
  //      true,
  //      0
  //    };
  //    testInputs.push_back(entry);
  //  }
  { // Blocked by initial intersection, requires that the shift block logic
    // accounts for a case when there is a non-crit intersection but the slopes
    // aren't close.  Also requires 'refined' recursive polynomial roots.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(1667.1542367539454972,
                                     1480.9341758208724968),
            bezier_geometry::Point2D(1667.1593539516609326,
                                     980.93417584705821355),
            bezier_geometry::Point2D(1667.1567953528031012,
                                     1230.9341758339653552)),
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(1667.1584744845338264,
                                     1366.8666701125966938),
            bezier_geometry::Point2D(1667.1584744845342811,
                                     1066.8666701125966938),
            bezier_geometry::Point2D(2167.1584744845340538,
                                     1216.8666701125971485)),
        bezier_geometry::Point2D(1667.1584744798203701, 1216.8671306603912399),
        false, 0};
    testInputs.push_back(entry);
  }
  { // Initially intersecting, but not to the extent that causes a block; tests
    // the 'aligned crit' case, different scaled up results so not included by
    // default.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(1203.3544922030243924,
                                     1135.2193366427989076),
            bezier_geometry::Point2D(1703.3027897872916583,
                                     1128.0290833376027422),
            bezier_geometry::Point2D(1453.328640995158139,
                                     1131.6242099902008249)),
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(1376.7064378066336303,
                                     1153.4534498599953167),
            bezier_geometry::Point2D(1432.6081372441283293,
                                     1135.2898866598613949),
            bezier_geometry::Point2D(1398.0589869628756787,
                                     1124.0641872453718406)),
        bezier_geometry::Point2D(1412.3152349281199349, 1032.203700124572606),
        true, 0.036451542233020120531};
    // testInputs.push_back(entry);
  }
  { // Stationary lies in the path of rotation, stationary is an extremely
    // concave curve, resulting in multiple intersection results for valid
    // parameters.  Tests the 'testSolution' functionality.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(1707.5108601507320145,
                                     1404.5361736489226132),
            bezier_geometry::Point2D(1630.1451781218761425,
                                     910.55788339295406786),
            bezier_geometry::Point2D(1668.8280191363039648,
                                     1157.5470285209382837)),
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(1709.0241102181398674,
                                     1151.7336394948335965),
            bezier_geometry::Point2D(1709.5430094438220294,
                                     1151.8893092625382906),
            bezier_geometry::Point2D(1680.548771274344972,
                                     1247.5941029008006353)),
        bezier_geometry::Point2D(1587.8918735741010551, 1287.0527852405327849),
        true, 73.943831409884737127};
    testInputs.push_back(entry);
  }
  { // Testing the limits of a curve's distance interval size.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                      bezier_geometry::Point2D(1, 3),
                                      bezier_geometry::Point2D(1, 2)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 2),
                                      bezier_geometry::Point2D(2, 2),
                                      bezier_geometry::Point2D(1.5, 2)),
        bezier_geometry::Point2D(1, -10000), true, 0};
    testInputs.push_back(entry);
  }
  { // Rotate a circle arc approximation about its fulcrum - this tests the
    // difference between rotational distance magnitudes and their approximation
    // as perpendicular magnitudes.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(4837.5534915189055027,
                                     222.70458730385507806),
            bezier_geometry::Point2D(4315.0720909739357012,
                                     10.411719369133606961),
            bezier_geometry::Point2D(4615.5552427110442295,
                                     19.977183841283476795)),
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(3486.9027659759508424,
                                     965.69038736007962598),
            bezier_geometry::Point2D(3486.9027659759503877,
                                     1165.6903873600795123),
            bezier_geometry::Point2D(3486.9027659759508424,
                                     1065.6903873600795123)),
        bezier_geometry::Point2D(4289.1985814182025933, 823.18497470261263516),
        false, 237.64555601396469342};
    testInputs.push_back(entry);
  }
  { // Barely not touching at the fulcrum, proper solution is very close to an
    // endpoint of the stationary curve.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(3670.9958165403281782,
                                     1188.4201213949363591),
            bezier_geometry::Point2D(3656.7729463835712522,
                                     1387.9137552896147554),
            bezier_geometry::Point2D(3663.8843814619499426,
                                     1288.1669383422756709)),
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(4021.2905806788257905,
                                     384.71246905987572973),
            bezier_geometry::Point2D(3670.3468493138971098,
                                     1203.5978603116964223),
            bezier_geometry::Point2D(3688.2087825683397568,
                                     726.60942705587217461)),
        bezier_geometry::Point2D(3670.996650483207759, 1188.4201808507750684),
        true, 1.3807662546566106254};
    testInputs.push_back(entry);
  }
  { // One of the curves' calculation for a common distance results in an
    // invalid value that must be checked.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(2336.8776300551121494,
                                     1175.4016335374185473),
            bezier_geometry::Point2D(2324.2775088648509154,
                                     1375.0043312796854025),
            bezier_geometry::Point2D(2330.5775694599815324,
                                     1275.2029824085518612)),
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(2324.2781111731510464,
                                     1374.9947899141889138),
            bezier_geometry::Point2D(2726.5137366954613753,
                                     2169.9421392088834182),
            bezier_geometry::Point2D(2372.3933189975637106,
                                     1849.8862942553851099)),
        bezier_geometry::Point2D(2324.2781111731510464, 1374.9947899141891412),
        true, 9.3974303998768817792};
    testInputs.push_back(entry);
  }
  { // An initial intersection blocks the shift, but requires the 'mostly away
    // from shift direction' test cases to be detected.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(1693.9378774471515499,
                                     1609.3184309387893336),
            bezier_geometry::Point2D(1241.2386295832029646,
                                     1821.5998266931396756),
            bezier_geometry::Point2D(1467.5882535151772572,
                                     1715.4591288159645046)),
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(1417.1572875253809798,
                                     1732.8427124746190202),
            bezier_geometry::Point2D(1464.7101133401386051,
                                     1698.2935621933663697),
            bezier_geometry::Point2D(1453.4844139256488234,
                                     1732.8427124746190202)),
        bezier_geometry::Point2D(1440.4910902804258512, 1728.1656302285432503),
        false, 0};
    testInputs.push_back(entry);
  }
  { // No initial intersection, moving is blocked at an endpoint; a standard
    // case except the stationary curve's calculation to match the moving
    // curve's endpoint distance is an odd case.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(1197.1168903461043556,
                                     1952.7206253528293018),
            bezier_geometry::Point2D(1453.0658037812845578,
                                     1523.1974551846174109),
            bezier_geometry::Point2D(1325.0913470636944567,
                                     1737.9590402687233563)),
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(945.28488624563749454,
                                     1701.6845100492535039),
            bezier_geometry::Point2D(945.28488624563726717,
                                     1401.6845100492535039),
            bezier_geometry::Point2D(1445.2848862456373809,
                                     1551.6845100492528218)),
        bezier_geometry::Point2D(1187.2954542253205545, 1578.4995968816410823),
        true, 130.02955800306321521};
    testInputs.push_back(entry);
  }
  { // No initial intersection, rotation is blocked by an endpoint, but this
    // tests a special case of curve intersection where only one intersection
    // must be reported.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(3206.3749143929258025,
                                     1556.7435898461899342),
            bezier_geometry::Point2D(4000.8485508819476308,
                                     1462.873248822468895),
            bezier_geometry::Point2D(3603.6117326374369441,
                                     1509.8084193343295283)),
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(4337.5814267138903233,
                                     1662.8230608988017138),
            bezier_geometry::Point2D(4069.3775031061213667,
                                     1646.9989054728671363),
            bezier_geometry::Point2D(4206.090284148191131,
                                     1610.6601562276850927)),
        bezier_geometry::Point2D(4000.8485508819485403, 1462.8732488224691224),
        false, 217.44005916635646258};
    testInputs.push_back(entry);
  }
  { // No initial intersection, rotation is blocked by an endpoint, but this
    // tests a special case of curve intersection where only one intersection
    // must be reported.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(1621.4883691118716342,
                                     1971.0861800747929919),
            bezier_geometry::Point2D(1706.178588783253872,
                                     1478.3108086406953134),
            bezier_geometry::Point2D(1663.8334789475627531,
                                     1724.698494357744039)),
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(1387.7680249107572763,
                                     1642.3918627558716707),
            bezier_geometry::Point2D(1369.6044617106233545,
                                     1698.2935621933663697),
            bezier_geometry::Point2D(1358.3787622961340276,
                                     1663.7444119121134918)),
        bezier_geometry::Point2D(1634.1992824249043679, 1897.1269312571548653),
        false, 297.17172965238330562};
    testInputs.push_back(entry);
  }
  { // No initial intersection, rotation is blocked by an endpoint, but this
    // tests a special case of curve intersection where only one intersection
    // must be reported.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(235.31494613716722597,
                                     56.220764755192863049),
            bezier_geometry::Point2D(310.81033245508378968,
                                     82.686717311421375598),
            bezier_geometry::Point2D(273.06263929612550783,
                                     69.453741033307119324)),
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(342.95999831586445907,
                                     95.16806798041249067),
            bezier_geometry::Point2D(342.9777560360663756,
                                     95.16806798041249067),
            bezier_geometry::Point2D(342.96887717596628136,
                                     85.16806798041249067)),
        bezier_geometry::Point2D(304.19384431602666155, 101.56056389090051653),
        false, 114.64615437092952277};
    testInputs.push_back(entry);
    // *** Multiply these values by 10 to test the hard cases.
  }
  { // No initial intersection, tests the most dense initial guesses in the
    // search for rotate interesection points.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(3115.7105496573303753,
                                     827.22464067972794055),
            bezier_geometry::Point2D(3389.1161349538151626,
                                     916.55743693505257852),
            bezier_geometry::Point2D(3237.4698245694630714,
                                     917.62610346776637016)),
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(2597.0066800830509237,
                                     919.36613605946968164),
            bezier_geometry::Point2D(3397.0017474585547461,
                                     916.55683456922008645),
            bezier_geometry::Point2D(2997.0042137708028349,
                                     917.9614853143448272)),
        bezier_geometry::Point2D(3385.9218746960359567, 463.28434596457418593),
        true, 1.6701374483266548943};
    testInputs.push_back(entry);
  }
  { // No initial intersection, stationary blocks at a point that its slope
    // changes very rapidly; requires the 'close along curve' cases for the
    // slope equivalence.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(1038.8202140313708242,
                                     829.06364676783891809),
            bezier_geometry::Point2D(1238.0273222026135045,
                                     811.27241145235973363),
            bezier_geometry::Point2D(1138.4237681169920506,
                                     820.16802911009926902)),
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(1239.0824599968818802,
                                     823.06821868923759666),
            bezier_geometry::Point2D(1239.0827868114815828,
                                     823.06818950098556797),
            bezier_geometry::Point2D(1230.1868967198893188,
                                     723.46465974673833443)),
        bezier_geometry::Point2D(1013.678797252788172, 547.55711599693142944),
        false, 347.2316087794695818};
    // testInputs.push_back(entry); // Currently this does not scale up to the
    // huge numbers.
  }
  { // No initial intersection, stationary blocks at neither curve's endpoint;
    // tests some odd cases in reducing polynomial functions.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(1245.2578214693144218,
                                     2176.1798668946794351),
            bezier_geometry::Point2D(2036.8134569540929988,
                                     2292.1094831809768948),
            bezier_geometry::Point2D(1641.0356392117037103,
                                     2234.1446750378281649)),
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(1874.2569825453952035,
                                     2189.8637448316744667),
            bezier_geometry::Point2D(1849.2569825453952035,
                                     2214.8637448316744667),
            bezier_geometry::Point2D(1849.2569825453952035,
                                     2189.8637448316744667)),
        bezier_geometry::Point2D(1552.1527091902514712, 2261.4131145816754724),
        false, 151.81102954332155264};
    testInputs.push_back(entry);
  }
  { // Initial intersection that is not at a distance crit, but the curves are
    // extremely close at a common distance crit, causing counterintuitive
    // results.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(929.40816013068274515,
                                     1416.2067197544245118),
            bezier_geometry::Point2D(609.22855919281118986,
                                     2149.340420482515583),
            bezier_geometry::Point2D(769.3183596617469675,
                                     1782.7735701184697064)),
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(957.85236667299307101,
                                     1378.657105544056094),
            bezier_geometry::Point2D(844.90220047072011766,
                                     1856.0227703512341577),
            bezier_geometry::Point2D(814.54406551790862068,
                                     1596.794207213044956)),
        bezier_geometry::Point2D(896.15770085408917112, 1490.2973551647951354),
        true, 4.3555170130753796665e-5};
    testInputs.push_back(entry);
  }
  { // Odd case in finding endpoint cases - one curve has 2 matching distances
    // for one of the other curve's endpoints and requires the right Newton
    // starting guess.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(1068.5205597993021911,
                                     2091.193331857569774),
            bezier_geometry::Point2D(1136.1235237297823915,
                                     1798.90949845617547),
            bezier_geometry::Point2D(1589.4617641001998436,
                                     2057.7230217076735244)),
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(1287.0464826032985002,
                                     1646.9589669595807209),
            bezier_geometry::Point2D(1164.806518809232557,
                                     1805.2542330025846695),
            bezier_geometry::Point2D(1225.9265007062656423,
                                     1726.1065999810825815)),
        bezier_geometry::Point2D(1217.1518209318630852, 1941.0576534649262612),
        false, 4.3578554677399798933};
    testInputs.push_back(entry);
  }
  { // The endpoint block is within the minimum angle determined by distance
    // intervals, but a smaller angle can be found through the standard search.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(212.96660644575734977,
                                     2093.1109465182848908),
            bezier_geometry::Point2D(407.61965097282023862,
                                     2321.3875161796177053),
            bezier_geometry::Point2D(-70.167820726265745179,
                                     2531.670972227389484)),
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(100.00000000000011369, 2300),
            bezier_geometry::Point2D(200.00000000000011369, 2400),
            bezier_geometry::Point2D(100.00000000000014211, 2400)),
        bezier_geometry::Point2D(147.59629828729475776, 2343.7410347108479982),
        true, 10.829577793597318092};
    testInputs.push_back(entry);
  }
  { // No initial touch, but the eventual block point is extremely close to the
    // fulcrum.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(3943.1542300784531108,
                                     1352.9427065115091864),
            bezier_geometry::Point2D(3143.1611915067592236,
                                     1349.605306800917333),
            bezier_geometry::Point2D(3543.1577107926059398,
                                     1351.2740066562132597)),
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(2743.7263710692986933,
                                     2392.8526147559573474),
            bezier_geometry::Point2D(2887.0371288909441319,
                                     2437.1473852439698931),
            bezier_geometry::Point2D(3473.8973379019057575,
                                     284.44673371816770668)),
        bezier_geometry::Point2D(3143.8711406075622108, 1349.6082794523376833),
        false, 0.15779138725405814081};
    /*
     * Can't be tested as the result angle varies slightly when the curves are
     * reversed due to the extreme proximity to the fulcrum - even tiny
     * parameter changes have a significant effect on the resulting angle.
     * */
    // testInputs.push_back(entry);
  }
  { // No initial intersection, blocking point is an neither curve's endpoint,
    // not close to the fulcrum; the blocking point is where the slope changes
    // rapidly so this tests finding and identifying solutions with large RoCs.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(1728.6549838410132907,
                                     1838.4948138001052484),
            bezier_geometry::Point2D(1154.9236688983983186,
                                     2396.0182467194927085),
            bezier_geometry::Point2D(1441.7893263697058046,
                                     2117.2565302597986374)),
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(1813.7413106745721052,
                                     1277.3924666254702061),
            bezier_geometry::Point2D(1665.4072021926390335,
                                     1299.685789860692239),
            bezier_geometry::Point2D(1892.1290831161700225,
                                     2303.6001836007449128)),
        bezier_geometry::Point2D(1615.9356800672624104, 1948.0298026694340479),
        true, 172.8977090852264098};
    testInputs.push_back(entry);
  }
  { // Intersection at the rotation fulcrum, endpoint of one curve.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(2666.1202935037463249,
                                     1091.205756616855524),
            bezier_geometry::Point2D(1867.8822020089153284,
                                     1038.1402378057687201),
            bezier_geometry::Point2D(2267.0012477563309403,
                                     1064.6729972113121221)),
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(2688.2113303618202735,
                                     1093.6809161950875477),
            bezier_geometry::Point2D(2538.611664985004154,
                                     1082.7291985606418621),
            bezier_geometry::Point2D(2523.8414189603195155,
                                     2311.7258100594744974)),
        bezier_geometry::Point2D(2538.6116649850046088, 1082.7291985606418621),
        true, 93.114775166115549609};
    testInputs.push_back(entry);
  }
  { // Multiple invalid matching Newton solutions - requires testing of the
    // resulting blocked intervals.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(2683.4452230702645465,
                                     1635.7635893738242885),
            bezier_geometry::Point2D(2261.2889374988267264,
                                     2315.3105491584492484),
            bezier_geometry::Point2D(2472.3670802845458638,
                                     1975.5370692661367684)),
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(2223.3044349464785228,
                                     2377.9247163847744559),
            bezier_geometry::Point2D(2353.6949767024066205,
                                     2452.0752836152109921),
            bezier_geometry::Point2D(3390.8714719836016229,
                                     476.52727922852454867)),
        bezier_geometry::Point2D(2324.1024430873076199, 2126.5013267698436721),
        true, 0.031512853384807866519};
    testInputs.push_back(entry);
  }
  { // Multiple invalid matching Newton solutions - requires testing of the
    // resulting blocked intervals.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(2161.2025177287723636,
                                     1709.5908595135551877),
            bezier_geometry::Point2D(1751.1403874820175588,
                                     2396.503553629403541),
            bezier_geometry::Point2D(1956.1714526053949612,
                                     2053.0472065714793644)),
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(1762.4275254075819248,
                                     2379.0809981571928802),
            bezier_geometry::Point2D(1894.1063058791137337,
                                     2450.919001842778016),
            bezier_geometry::Point2D(2896.2585704357156828,
                                     457.37546365654770852)),
        bezier_geometry::Point2D(1881.6880194465493332, 2138.0666919979430531),
        true, 0.022099720264987847418};
    testInputs.push_back(entry);
  }
  { // Multiple intersections, one at the fulcrum; the non-fulcrum intersection
    // does not block.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(1510.9349150410494076,
                                     1665.612567013851276),
            bezier_geometry::Point2D(1143.982330641430508,
                                     2005.2385389336095614),
            bezier_geometry::Point2D(1327.4586228412399578,
                                     1835.4255529737304187)),
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(1417.1572875253809798,
                                     1732.8427124746190202),
            bezier_geometry::Point2D(1464.7101133401386051,
                                     1698.2935621933663697),
            bezier_geometry::Point2D(1453.4844139256488234,
                                     1732.8427124746190202)),
        bezier_geometry::Point2D(1450.9208228534962473, 1721.1574664221161584),
        false, 0.025782110596269194186};
    // Cannot include because the small overlap allowing the non-fulcrum
    // intersection to not block does not scale up.
    // testInputs.push_back(entry);
  }
  { // Both straight lines, intersection is infinite if the moving is rotated
    // to
    // align its distance crit with that of the stationary.  Requires 'aligned
    // crit' cases before endopints.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(10.000000000000227374,
                                     1644.254229094602124),
            bezier_geometry::Point2D(10.000193681901123455,
                                     1444.2542290946960293),
            bezier_geometry::Point2D(10.000096840950618571,
                                     1544.2542290946489629)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(10, 0),
                                      bezier_geometry::Point2D(10, 2500),
                                      bezier_geometry::Point2D(10, 1250)),
        bezier_geometry::Point2D(810.00019368152618426, 1644.2548693344658659),
        true, 5.548577740230039928e-5};
    testInputs.push_back(entry);
  }
  { // The curves don't quite intersect at a common distance crit that would
    // block the rotation - tests the functionality that would reduce the angle
    // to 0 if it's near 360 for aligned distance crits.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(750.27698357137933272,
                                     534.73540374107369644),
            bezier_geometry::Point2D(743.35512101519498174,
                                     234.81526800698645729),
            bezier_geometry::Point2D(1246.6829451834325937,
                                     373.23889828038954875)),
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(739.22929948747355411,
                                     600.86012970490264706),
            bezier_geometry::Point2D(1012.7386232531931682,
                                     528.80008659348925448),
            bezier_geometry::Point2D(839.95393981462666488,
                                     428.07544626633620055)),
        bezier_geometry::Point2D(815.85208047131004605, 439.80165882102988917),
        true, 0};
    // Cannot include because the small overlap allowing the non-fulcrum
    // intersection to not block does not scale up.
    // testInputs.push_back(entry);
  }
  { // Not intersecting at the fulcrum, but each curve's closest point
    // qualifies
    // as a fulcrum intersection for aligned distance crit cases.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(1372.478272538247893,
                                     248.61533171072366599),
            bezier_geometry::Point2D(802.89506525557226269,
                                     60.00205014873881737),
            bezier_geometry::Point2D(1119.1222158239074815,
                                     59.378156382618698217)),
        bezier_geometry::BezierCurveQ(
            bezier_geometry::Point2D(989.84383156794592651, 60),
            bezier_geometry::Point2D(189.84383156794592651, 60),
            bezier_geometry::Point2D(589.84383156794592651, 60)),
        bezier_geometry::Point2D(804.97299484048380691, 60.000000000000454747),
        true, 0};
    testInputs.push_back(entry);
  }
  { // Intersecting at the fulcrum, but there is another, better, non-endpoint
    // solution.
    RotateAgainstTestEntry entry = {
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                      bezier_geometry::Point2D(3, 1),
                                      bezier_geometry::Point2D(2, 10)),
        bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(2.9, 1),
                                      bezier_geometry::Point2D(1, 6.5),
                                      bezier_geometry::Point2D(2.9, 6.5)),
        bezier_geometry::Point2D(2.8829942205605529892, 1.9914454290549752358),
        true, 2.3410356836418100279};
    testInputs.push_back(entry);
  }
  for (std::vector<RotateAgainstTestEntry>::iterator i = testInputs.begin();
       i != testInputs.end(); i++) {
    for (int j = 0; j < 2; j++) {
      for (int k = 0; k < 4; k++) {
        for (int l = 0; l < 2; l++) {
          bezier_geometry::BezierCurveQ moving(i->moving);
          bezier_geometry::BezierCurveQ stationary(i->stationary);
          bezier_geometry::Point2D fulcrum(i->fulcrum);
          bool multiplierSet = false;
          bezier_geometry::RealNum expectedAngle = i->expectedAngle;
          bool clockwise = i->clockwise;
          if (j == 1) {
            moving = i->stationary;
            stationary = i->moving;
            clockwise = !clockwise;
          }
          if (k == 1 || k == 3) {
            moving = bezier_geometry::BezierCurveQ(
                moving.valueAt(1), moving.valueAt(0), moving.getControl());
          }
          if (k == 2 || k == 3) {
            stationary = bezier_geometry::BezierCurveQ(stationary.valueAt(1),
                                                       stationary.valueAt(0),
                                                       stationary.getControl());
          }
          if (l == 1) {
            multiplierSet = true;
            bezier_geometry::RealNum multiplier = 1000;
            moving = bezier_geometry::BezierCurveQ(
                bezier_geometry::Point2D(moving.valueAt(0).getX() * multiplier,
                                         moving.valueAt(0).getY() * multiplier),
                bezier_geometry::Point2D(moving.valueAt(1).getX() * multiplier,
                                         moving.valueAt(1).getY() * multiplier),
                bezier_geometry::Point2D(
                    moving.getControl().getX() * multiplier,
                    moving.getControl().getY() * multiplier));
            stationary = bezier_geometry::BezierCurveQ(
                bezier_geometry::Point2D(
                    stationary.valueAt(0).getX() * multiplier,
                    stationary.valueAt(0).getY() * multiplier),
                bezier_geometry::Point2D(
                    stationary.valueAt(1).getX() * multiplier,
                    stationary.valueAt(1).getY() * multiplier),
                bezier_geometry::Point2D(
                    stationary.getControl().getX() * multiplier,
                    stationary.getControl().getY() * multiplier));
            fulcrum = bezier_geometry::Point2D(fulcrum.getX() * multiplier,
                                               fulcrum.getY() * multiplier);
          }
          try {
            bezier_geometry::BezierCurveQ::RotateAgainstResult result;
            debug() << "*** BEFORE rotateAgainst ***";
            moving.rotateAgainst(stationary, fulcrum, clockwise, result);
            debug() << "^"
                    << "moving:" << moving << "stationary:" << stationary
                    << "fulcrum:" << fulcrum << "clockwise:" << clockwise
                    << "outputAngle:" << bezier_geometry::toString(result.angle)
                    << "outputParam:" << bezier_geometry::toString(result.param)
                    << "outputInputParam:"
                    << bezier_geometry::toString(result.inputParam)
                    << "blockedCWVerticalAngleStart:"
                    << result.blockedCWVerticalAngleStart
                    << "blockedCWVerticalAngleEnd:"
                    << result.blockedCWVerticalAngleEnd << "expectedAngle:"
                    << bezier_geometry::toString(expectedAngle)
                    << "rotated intersection:"
                    << (result.angle >= 0
                            ? moving
                                  .rotate(fulcrum, result.angle *
                                                       (clockwise ? -1.0 : 1.0))
                                  .pointsOfIntersection(stationary)
                            : bezier_geometry::StaticVector<
                                  std::pair<bezier_geometry::RealNum,
                                            bezier_geometry::RealNum>,
                                  4>())
                    << "moving point:"
                    << (result.param >= 0
                            ? moving.valueAt(result.param)
                                  .rotate(fulcrum, result.angle *
                                                       (clockwise ? -1.0 : 1.0))
                                  .toString()
                            : std::string())
                    << "stationary point:"
                    << (result.inputParam >= 0
                            ? stationary.valueAt(result.inputParam).toString()
                            : std::string())
                    << std::endl;
            CHECK(
                bezier_geometry::sufficientlyClose(result.angle, expectedAngle),
                true);
            CHECK(result.blockedCWVerticalAngleStart < 0,
                  result.blockedCWVerticalAngleEnd < 0);
            if (result.angle >= 0) {
              const bezier_geometry::Point2D movingBlockedPoint(
                  moving.valueAt(result.param)
                      .rotate(fulcrum,
                              result.angle * (clockwise ? -1.0 : 1.0)));
              if (result.blockedCWVerticalAngleStart < 0 ||
                  result.blockedCWVerticalAngleEnd < 0) {
                // Straight lines can report no blocked interval even when
                // they're not on the fulcrum.
                // CHECK(movingBlockedPoint, fulcrum);
              } else {
                CHECK(result.blockedCWVerticalAngleStart <= 360.0, true);
                CHECK(result.blockedCWVerticalAngleEnd <= 360.0, true);
                if (movingBlockedPoint != fulcrum) {
                  const bezier_geometry::Point2D positiveVerticalPoint(
                      movingBlockedPoint.shift(
                          100,
                          std::numeric_limits<
                              bezier_geometry::RealNum>::infinity(),
                          true, true));
                  const bezier_geometry::Point2D movingDirectionPoint(
                      movingBlockedPoint.shift(
                          100,
                          (-1.0) / bezier_geometry::Point2D::getSlopeBetween(
                                       fulcrum, movingBlockedPoint),
                          clockwise ==
                              (movingBlockedPoint.getY() > fulcrum.getY()),
                          clockwise ==
                              (movingBlockedPoint.getX() < fulcrum.getX())));
                  const bezier_geometry::Point2D startCWBlockedAnglePoint(
                      positiveVerticalPoint.rotate(
                          movingBlockedPoint,
                          (-1.0) * result.blockedCWVerticalAngleStart));
                  const bezier_geometry::Point2D endCWBlockedAnglePoint(
                      positiveVerticalPoint.rotate(
                          movingBlockedPoint,
                          (-1.0) * result.blockedCWVerticalAngleEnd));
                  const bezier_geometry::RealNum blockedAngleInterval =
                      bezier_geometry::Point2D::getAngleBetween(
                          startCWBlockedAnglePoint, endCWBlockedAnglePoint,
                          movingBlockedPoint, true);
                  if (startCWBlockedAnglePoint !=
                      endCWBlockedAnglePoint) { // Start and end points would
                                                // be equal in a 360 degree
                                                // interval.
                    debug()
                        << "^"
                        << "blockedAngleInterval:"
                        << bezier_geometry::toString(blockedAngleInterval)
                        << "movingDirectionPoint:" << movingDirectionPoint
                        << "movingBlockedPoint:" << movingBlockedPoint
                        << "startCWBlockedAnglePoint:"
                        << startCWBlockedAnglePoint
                        << "endCWBlockedAnglePoint:" << endCWBlockedAnglePoint
                        << std::endl;
                    CHECK(bezier_geometry::Point2D::getAngleBetween(
                              startCWBlockedAnglePoint, movingDirectionPoint,
                              movingBlockedPoint, true) <= blockedAngleInterval,
                          true);
                  }
                }
              }
            }
            bezier_geometry::StaticVector<
                std::pair<bezier_geometry::RealNum, bezier_geometry::RealNum>,
                4>
                rotatedIntersection;
            if (result.angle == 0) {
              rotatedIntersection = moving.pointsOfIntersection(stationary);
              if (!bezier_geometry::BezierCurveQ::isIntersectionInfinite(
                      rotatedIntersection) &&
                  rotatedIntersection.empty()) {
                if (moving.valueAt(result.param) !=
                    stationary.valueAt(result.inputParam)) {
                  CHECK(bezier_geometry::sufficientlySmall(
                            moving.valueAt(moving.minDistance(fulcrum))
                                .distanceFrom(fulcrum)) &&
                            bezier_geometry::sufficientlySmall(
                                stationary
                                    .valueAt(stationary.minDistance(fulcrum))
                                    .distanceFrom(fulcrum)),
                        true);
                }
                //                CHECK(moving.valueAt(result.param),
                //                stationary.valueAt(result.inputParam));
              }
              // CHECK(bezier_geometry::BezierCurveQ::isIntersectionInfinite(rotatedIntersection)
              // || rotatedIntersection.size() > 0, true);
            } else if (result.angle != -1) {
              const bezier_geometry::BezierCurveQ rotated(moving.rotate(
                  fulcrum, result.angle * (clockwise ? -1.0 : 1.0)));
              rotatedIntersection = rotated.pointsOfIntersection(stationary);
              debug() << "^ rotatedIntersection:" << rotatedIntersection
                      << std::endl;
              CHECK(bezier_geometry::BezierCurveQ::isIntersectionInfinite(
                        rotatedIntersection) ||
                        rotatedIntersection.size() > 0,
                    true);
            }
            if (!bezier_geometry::BezierCurveQ::isIntersectionInfinite(
                    rotatedIntersection) &&
                rotatedIntersection.size() > 0) {
              bezier_geometry::Point2D movingRotatedTestPoint(
                  moving.valueAt(result.param)
                      .rotate(fulcrum,
                              (clockwise ? -1.0 : 1.0) * expectedAngle));
              bezier_geometry::Point2D stationaryTestPoint(
                  stationary.valueAt(result.inputParam));
              if (movingRotatedTestPoint != stationaryTestPoint) {
                if (!multiplierSet) {
                  debug() << "movingRotatedTestPoint:" << movingRotatedTestPoint
                          << "stationaryTestPoint:" << stationaryTestPoint
                          << std::endl;
                  debug() << "Output parameters from rotateAgainst did not "
                             "yield matching points."
                          << std::endl;
                  CHECK(1, 0);
                } else {
                  CHECK(bezier_geometry::sufficientlySmall(
                            movingRotatedTestPoint.distanceFrom(
                                stationaryTestPoint) /
                            100.0),
                        true);
                }
              }
            }
          } catch (const std::string &msg) {
            throw msg;
          }
        }
      }
    }
  }
  TEST_END
}

void rotateAgainstCircleTest() {
  TEST_START
  typedef struct {
    bezier_geometry::BezierCurveQ curve;
    bezier_geometry::Point2D rotationFulcrum;
    bool clockwise;
    bezier_geometry::CircleArc circleArc;
    bezier_geometry::RealNum expectedAngle;
  } RotateAgainstCircleTestEntry;
  for (const RotateAgainstCircleTestEntry &currentEntry :
       std::initializer_list<RotateAgainstCircleTestEntry>{
           {// Non-endpoint of curve touches non-endpoint of circle arc.
            bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                          bezier_geometry::Point2D(3, 1),
                                          bezier_geometry::Point2D(2, 10)),
            bezier_geometry::Point2D(3, 0), true,
            bezier_geometry::CircleArc(bezier_geometry::Point2D(5, 5), 1.0, 0,
                                       360),
            19.027536705948094209},
           {// Non-endpoint of curve touches non-endpoint of circle arc.
            bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                          bezier_geometry::Point2D(3, 1),
                                          bezier_geometry::Point2D(2, 10)),
            bezier_geometry::Point2D(3, 0), false,
            bezier_geometry::CircleArc(bezier_geometry::Point2D(5, 5), 1, 0,
                                       360),
            311.42396197008099534},
           {// Non-endpoint of curve touches non-endpoint of circle arc.
            bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                          bezier_geometry::Point2D(3, 1),
                                          bezier_geometry::Point2D(2, 10)),
            bezier_geometry::Point2D(3, 0), true,
            bezier_geometry::CircleArc(bezier_geometry::Point2D(5, 5), 1.0, 180,
                                       1),
            19.027536705948094209},
           {// Non-endpoint of curve touches start of circle arc.
            bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                          bezier_geometry::Point2D(3, 1),
                                          bezier_geometry::Point2D(2, 10)),
            bezier_geometry::Point2D(3, 0), false,
            bezier_geometry::CircleArc(bezier_geometry::Point2D(5, 5), 1.0, 180,
                                       1),
            313.3254853050739257},
           {// Non-endpoint of curve touches start of circle arc.
            bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                          bezier_geometry::Point2D(3, 1),
                                          bezier_geometry::Point2D(2, 10)),
            bezier_geometry::Point2D(3, 0), false,
            bezier_geometry::CircleArc(bezier_geometry::Point2D(5, 5), 1.0, 181,
                                       1),
            313.46594514321759561},
           {// Non-endpoint of curve touches end of circle arc.
            bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                          bezier_geometry::Point2D(3, 1),
                                          bezier_geometry::Point2D(2, 10)),
            bezier_geometry::Point2D(3, 0), true,
            bezier_geometry::CircleArc(bezier_geometry::Point2D(5, 5), 1.0, 90,
                                       265),
            19.078578673131737276},
           {// Endpoint of curve touches non-endpoint of circle arc.
            bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                          bezier_geometry::Point2D(3, 1),
                                          bezier_geometry::Point2D(2, 10)),
            bezier_geometry::Point2D(3, 0), false,
            bezier_geometry::CircleArc(bezier_geometry::Point2D(1, -0.5), 1.0,
                                       0, 360),
            14.088941411490109701}}) {
    bezier_geometry::RealNum outputAngle, outputParam;
    bezier_geometry::Point2D outputCirclePoint;
    debug() << "^^^^ BEFORE rotateAgainstCircleArc" << std::endl;
    currentEntry.curve.rotateAgainstCircleArc(
        currentEntry.circleArc, currentEntry.rotationFulcrum,
        currentEntry.clockwise, outputAngle, outputParam, outputCirclePoint);
    debug()
        << "^^^^ AFTER rotateAgainstCircleArc -"
        << "outputParam:" << bezier_geometry::toString(outputParam)
        << "outputCirclePoint:" << outputCirclePoint
        << "angle with positive vertical:"
        << bezier_geometry::Point2D::getAngleBetween(
               currentEntry.circleArc.getFulcrum().shift(
                   currentEntry.circleArc.getRadius(),
                   std::numeric_limits<bezier_geometry::RealNum>::infinity(),
                   true, true),
               outputCirclePoint, currentEntry.circleArc.getFulcrum(), true)
        << "outputAngle:" << bezier_geometry::toString(outputAngle)
        << "expected:" << currentEntry.expectedAngle << std::endl;
    CHECK(bezier_geometry::sufficientlyClose(outputAngle,
                                             currentEntry.expectedAngle),
          true);
    if (outputAngle >= 0) {
      const bezier_geometry::Point2D curvePoint(
          currentEntry.curve.valueAt(outputParam));
      CHECK(bezier_geometry::sufficientlyClose(
                bezier_geometry::Point2D::getAngleBetween(
                    curvePoint, outputCirclePoint, currentEntry.rotationFulcrum,
                    currentEntry.clockwise),
                outputAngle),
            true);
      CHECK(bezier_geometry::sufficientlyClose(
                outputCirclePoint.distanceFrom(
                    currentEntry.circleArc.getFulcrum()),
                currentEntry.circleArc.getRadius()),
            true);
      CHECK(curvePoint.rotate(currentEntry.rotationFulcrum,
                              outputAngle *
                                  (currentEntry.clockwise ? (-1.0) : 1.0)),
            outputCirclePoint);
      const bezier_geometry::BezierCurveQ rotated(currentEntry.curve.rotate(
          currentEntry.rotationFulcrum,
          outputAngle * (currentEntry.clockwise ? (-1.0) : 1.0)));
      const bezier_geometry::Point2D rotatedPoint(rotated.valueAt(outputParam));
      if (!bezier_geometry::sufficientlyCloseSlopes(
              rotated.rateOfChangeAtParam(outputParam),
              (-1.0) /
                  bezier_geometry::Point2D::getSlopeBetween(
                      currentEntry.circleArc.getFulcrum(), rotatedPoint))) {
        const bezier_geometry::CircleArc::PointInArc pointInArc =
            currentEntry.circleArc.liesWithinArc(rotatedPoint);
        CHECK((!currentEntry.circleArc.isFullCircle() &&
               (pointInArc == bezier_geometry::CircleArc::PointInArc::START ||
                pointInArc == bezier_geometry::CircleArc::PointInArc::END)) ||
                  rotatedPoint == rotated.valueAt(0) ||
                  rotatedPoint == rotated.valueAt(1),
              true);
      }
    }
  }
  TEST_END
}

void shiftAgainstCircleTest() {
  TEST_START
  typedef struct {
    bezier_geometry::BezierCurveQ curve;
    bezier_geometry::RealNum slope;
    bool right;
    bool up;
    bezier_geometry::CircleArc circleArc;
    bezier_geometry::RealNum expectedDistance;
  } ShiftAgainstCircleTestEntry;
  for (const ShiftAgainstCircleTestEntry &currentEntry :
       std::initializer_list<ShiftAgainstCircleTestEntry>{
           {// Non-endpoint of curve touches non-endpoint of arc, touch point
            // slope matches input slope.
            bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                          bezier_geometry::Point2D(1, 3),
                                          bezier_geometry::Point2D(5, 2)),
            0, true, true,
            bezier_geometry::CircleArc(bezier_geometry::Point2D(5, 2), 1, 0,
                                       360),
            1},
           {// Non-endpoint of curve touches non-endpoint of arc, touch point
            // slope does NOT match input slope.
            bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1.1),
                                          bezier_geometry::Point2D(1, 3.1),
                                          bezier_geometry::Point2D(5, 2.1)),
            0, true, true,
            bezier_geometry::CircleArc(bezier_geometry::Point2D(5, 2), 1, 0,
                                       360),
            1.004005129268780383},
           {// Non-endpoint of curve touches non-endpoint of arc.
            bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                          bezier_geometry::Point2D(1, 3),
                                          bezier_geometry::Point2D(5, 2)),
            0, true, true,
            bezier_geometry::CircleArc(bezier_geometry::Point2D(5, 2), 1, 260,
                                       280),
            1},
           {// Non-endpoint of curve touches endpoint of arc.
            bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                          bezier_geometry::Point2D(1, 3),
                                          bezier_geometry::Point2D(5, 2)),
            0, true, true,
            bezier_geometry::CircleArc(bezier_geometry::Point2D(5, 2), 1, 280,
                                       260),
            1.0754996262018829967},
           {// Endpoint of curve touches non-endpoint of arc.
            bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                          bezier_geometry::Point2D(1, 3),
                                          bezier_geometry::Point2D(5, 2)),
            std::numeric_limits<bezier_geometry::RealNum>::infinity(), true,
            false,
            bezier_geometry::CircleArc(bezier_geometry::Point2D(1, -1), 1, 270,
                                       90),
            1},
           {// Endpoint of curve touches endpoint of arc.
            bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                          bezier_geometry::Point2D(1, 3),
                                          bezier_geometry::Point2D(5, 2)),
            std::numeric_limits<bezier_geometry::RealNum>::infinity(), true,
            false,
            bezier_geometry::CircleArc(bezier_geometry::Point2D(2, -3), 1, 180,
                                       270),
            4},
           {// Endpoint of curve not blocked by endpoint of arc.
            bezier_geometry::BezierCurveQ(bezier_geometry::Point2D(1, 1),
                                          bezier_geometry::Point2D(1, 3),
                                          bezier_geometry::Point2D(5, 2)),
            std::numeric_limits<bezier_geometry::RealNum>::infinity(), true,
            false,
            bezier_geometry::CircleArc(bezier_geometry::Point2D(0, -3), 1, 90,
                                       180),
            -1}}) {
    bezier_geometry::RealNum outputDistance, outputParam;
    bezier_geometry::Point2D outputCirclePoint;
    debug() << "*** BEFORE shiftAgainstCircleArc ***" << std::endl;
    currentEntry.curve.shiftAgainstCircleArc(
        currentEntry.circleArc, currentEntry.slope, currentEntry.right,
        currentEntry.up, outputDistance, outputParam, outputCirclePoint);
    debug() << "*** AFTER shiftAgainstCircleArc ***"
            << "expected distance:"
            << bezier_geometry::toString(currentEntry.expectedDistance)
            << "outputDistance" << bezier_geometry::toString(outputDistance)
            << "outputParam" << bezier_geometry::toString(outputParam)
            << "outputCirclePoint" << outputCirclePoint
            << bezier_geometry::toString(outputCirclePoint.distanceFrom(
                   currentEntry.circleArc.getFulcrum()))
            << "curve point:"
            << (outputParam >= 0
                    ? currentEntry.curve.valueAt(outputParam).toString()
                    : "")
            << std::endl;
    CHECK(bezier_geometry::sufficientlyClose(outputDistance,
                                             currentEntry.expectedDistance),
          true);
    if (outputDistance >= 0) {
      const bezier_geometry::Point2D curvePoint(
          currentEntry.curve.valueAt(outputParam));
      CHECK(bezier_geometry::sufficientlyClose(
                curvePoint.distanceFrom(outputCirclePoint), outputDistance),
            true);
      CHECK(bezier_geometry::sufficientlyClose(
                outputCirclePoint.distanceFrom(
                    currentEntry.circleArc.getFulcrum()),
                currentEntry.circleArc.getRadius()),
            true);
      CHECK(curvePoint.shift(outputDistance, currentEntry.slope,
                             currentEntry.right, currentEntry.up),
            outputCirclePoint);
      if (!bezier_geometry::sufficientlyCloseSlopes(
              currentEntry.curve.rateOfChangeAtParam(outputParam),
              (-1.0) / bezier_geometry::Point2D::getSlopeBetween(
                           currentEntry.circleArc.getFulcrum(),
                           outputCirclePoint))) {
        const bezier_geometry::CircleArc::PointInArc pointInArc =
            currentEntry.circleArc.liesWithinArc(outputCirclePoint);
        CHECK((!currentEntry.circleArc.isFullCircle() &&
               (pointInArc == bezier_geometry::CircleArc::PointInArc::START ||
                pointInArc == bezier_geometry::CircleArc::PointInArc::END)) ||
                  curvePoint == currentEntry.curve.valueAt(0) ||
                  curvePoint == currentEntry.curve.valueAt(1),
              true);
      }
    }
  }
  TEST_END
}

int main(int argc, char **argv) {
  std::cout << "%SUITE_STARTING% BezierCurveQTest" << std::endl;
  std::cout << "%SUITE_STARTED%" << std::endl;
  basicFunctionalityTest();
  pointsOfIntersectionTest();
  shiftAgainstTest();
  rotateAgainstTest();
  rotateAgainstCircleTest();
  shiftAgainstCircleTest();
  std::cout << "%SUITE_FINISHED% time=0" << std::endl;
  return (EXIT_SUCCESS);
}
