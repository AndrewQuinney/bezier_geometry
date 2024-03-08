#include <stdlib.h>

#include <cmath>

#include "GeometryUtil.hpp"
#include "Point2D.hpp"
#include "TestUtils.hpp"

void coordinatesDistanceEquivalenceTest() {
  TEST_START
  typedef std::pair<
      std::pair<std::pair<bezier_geometry::RealNum, bezier_geometry::RealNum>,
                std::pair<bezier_geometry::RealNum, bezier_geometry::RealNum>>,
      bezier_geometry::RealNum>
      paramEntry;
  std::vector<paramEntry> testInputs;
  {
    testInputs.push_back(paramEntry());
    testInputs.back().first.first.first = 0;
    testInputs.back().first.first.second = 0;
    testInputs.back().first.second.first = 0;
    testInputs.back().first.second.second = 1;
    testInputs.back().second = 1;
  }
  {
    testInputs.push_back(paramEntry());
    testInputs.back().first.first.first = 0;
    testInputs.back().first.first.second = 0;
    testInputs.back().first.second.first = 1;
    testInputs.back().first.second.second = 0;
    testInputs.back().second = 1;
  }
  {
    testInputs.push_back(paramEntry());
    testInputs.back().first.first.first = 0;
    testInputs.back().first.first.second = 0;
    testInputs.back().first.second.first = 1;
    testInputs.back().first.second.second = 1;
    testInputs.back().second = std::sqrt(2);
  }
  {
    testInputs.push_back(paramEntry());
    testInputs.back().first.first.first = 0;
    testInputs.back().first.first.second = 0;
    testInputs.back().first.second.first = 0.9;
    testInputs.back().first.second.second = 1;
    testInputs.back().second = std::sqrt(1.81);
  }
  {
    testInputs.push_back(paramEntry());
    testInputs.back().first.first.first = 0;
    testInputs.back().first.first.second = 0;
    testInputs.back().first.second.first = 0;
    testInputs.back().first.second.second = 0;
    testInputs.back().second = 0;
  }
  {
    testInputs.push_back(paramEntry());
    testInputs.back().first.first.first = -5;
    testInputs.back().first.first.second = 6;
    testInputs.back().first.second.first = 12;
    testInputs.back().first.second.second = -10;
    testInputs.back().second = 23.345235;
  }
  for (std::vector<paramEntry>::iterator i = testInputs.begin();
       i != testInputs.end(); i++) {
    bezier_geometry::Point2D first(i->first.first.first, i->first.first.second);
    bezier_geometry::Point2D second(i->first.second.first,
                                    i->first.second.second);
    CHECK(first.getX(), i->first.first.first);
    CHECK(first.getY(), i->first.first.second);
    CHECK(second.getX(), i->first.second.first);
    CHECK(second.getY(), i->first.second.second);
    CHECK(bezier_geometry::sufficientlyClose(first.distanceFrom(second),
                                             i->second),
          true);
    CHECK(bezier_geometry::sufficientlyClose(second.distanceFrom(first),
                                             i->second),
          true);
    CHECK(first == second, first.distanceFrom(second) == 0);
  }
  TEST_END
}

void shiftRotateTest() {
  TEST_START {
    bezier_geometry::Point2D first(3, 2);
    bezier_geometry::Point2D second(4, 2);
    CHECK(second.rotate(first, 90), bezier_geometry::Point2D(3, 3));
    CHECK(second.rotate(first, -90), bezier_geometry::Point2D(3, 1));
    CHECK(second.rotate(first, -180), bezier_geometry::Point2D(2, 2));
    CHECK(second.rotate(first, 180), bezier_geometry::Point2D(2, 2));
    CHECK(second.rotate(first, 45), bezier_geometry::Point2D(3.70711, 2.70711));
    CHECK(second.rotate(first, -45),
          bezier_geometry::Point2D(3.70711, 1.29289));
    CHECK(second.rotate(first, 0), second);
    CHECK(second.rotate(first, 360), second);
    CHECK(second.rotate(first, 720), second);
  }
  const int RANDOM_RANGE = 1000;
  for (int i = 0; i < 1000; i++) {
    bezier_geometry::Point2D first(randomReal(0, RANDOM_RANGE),
                                   randomReal(0, RANDOM_RANGE));
    bezier_geometry::Point2D second(randomReal(0, RANDOM_RANGE),
                                    randomReal(0, RANDOM_RANGE));
    {
      bezier_geometry::RealNum rotationAngle = randomReal(0, RANDOM_RANGE);
      bezier_geometry::Point2D rotated(second.rotate(first, rotationAngle));
      CHECK_FUZZY(first.distanceFrom(second), first.distanceFrom(rotated));
      CHECK(bezier_geometry::sufficientlyClose(
                bezier_geometry::Point2D::getAngleBetween(
                    second, rotated, first, rotationAngle < 0),
                std::fabs(rotationAngle) -
                    (360.0 * ((int)(std::fabs(rotationAngle) / 360.0)))),
            true);
    }
    for (int j = 0; j < 2; j++) {
      bezier_geometry::RealNum shiftDistance =
          std::fabs(randomReal(0, RANDOM_RANGE)) + 1;
      bezier_geometry::RealNum shiftSlope =
          j == 0 ? randomReal(0, RANDOM_RANGE)
                 : std::numeric_limits<bezier_geometry::RealNum>::infinity();
      bool shiftRight = randomReal(0, RANDOM_RANGE) > 0;
      bool shiftUp = randomReal(0, RANDOM_RANGE) > 0;
      bezier_geometry::Point2D shifted(
          first.shift(shiftDistance, shiftSlope, shiftRight, shiftUp));
      CHECK(bezier_geometry::sufficientlyClose(shifted.distanceFrom(first),
                                               shiftDistance),
            true);
      if (!std::isinf(shiftSlope)) {
        CHECK(bezier_geometry::sufficientlyClose(
                  (shifted.getY() - first.getY()) /
                      (shifted.getX() - first.getX()),
                  shiftSlope),
              true);
        CHECK(shifted.getX() > first.getX(), shiftRight);
      } else {
        CHECK(bezier_geometry::sufficientlyClose(shifted.getX(), first.getX()),
              true);
        CHECK(shifted.getY() > first.getY(), shiftUp);
      }
      CHECK(shifted.shift(shiftDistance, shiftSlope, !shiftRight, !shiftUp),
            first);
    }
  }
  TEST_END
}

typedef struct {
  bezier_geometry::Point2D first;
  bezier_geometry::Point2D second;
  bezier_geometry::Point2D third;
  bool colinear;
} ColinearTestEntry;

void colinearTest() {
  TEST_START
  std::vector<ColinearTestEntry> testInputs;
  {
    ColinearTestEntry currentEntry = {bezier_geometry::Point2D(1, 2),
                                      bezier_geometry::Point2D(1, 3),
                                      bezier_geometry::Point2D(1, -50), true};
    testInputs.push_back(currentEntry);
  }
  {
    ColinearTestEntry currentEntry = {bezier_geometry::Point2D(2, 1),
                                      bezier_geometry::Point2D(3, 1),
                                      bezier_geometry::Point2D(-50, 1), true};
    testInputs.push_back(currentEntry);
  }
  {
    ColinearTestEntry currentEntry = {bezier_geometry::Point2D(1, 1),
                                      bezier_geometry::Point2D(3, 2),
                                      bezier_geometry::Point2D(5, 3), true};
    testInputs.push_back(currentEntry);
  }
  {
    ColinearTestEntry currentEntry = {bezier_geometry::Point2D(1, 1),
                                      bezier_geometry::Point2D(3, 2),
                                      bezier_geometry::Point2D(4.99, 3), false};
    testInputs.push_back(currentEntry);
  }
  const int RANDOM_RANGE = 1000;
  for (std::vector<ColinearTestEntry>::iterator i = testInputs.begin();
       i != testInputs.end(); i++) {
    CHECK(bezier_geometry::Point2D::colinear(i->first, i->second, i->third),
          i->colinear);
    for (int j = 0; j < 50; j++) {
      bezier_geometry::Point2D fulcrum(randomReal(0, RANDOM_RANGE),
                                       randomReal(0, RANDOM_RANGE));
      bezier_geometry::RealNum rotationAngle = randomReal(0, RANDOM_RANGE);
      CHECK(bezier_geometry::Point2D::colinear(
                i->first.rotate(fulcrum, rotationAngle),
                i->second.rotate(fulcrum, rotationAngle),
                i->third.rotate(fulcrum, rotationAngle)),
            i->colinear);
    }
  }
  TEST_END
}

void slopeTest() {
  TEST_START
  CHECK(bezier_geometry::Point2D::getSlopeBetween(
            bezier_geometry::Point2D(1, 1), bezier_geometry::Point2D(2, 2)),
        1.0);
  CHECK(bezier_geometry::Point2D::getSlopeBetween(
            bezier_geometry::Point2D(2, 2), bezier_geometry::Point2D(1, 1)),
        1.0);
  CHECK(bezier_geometry::Point2D::getSlopeBetween(
            bezier_geometry::Point2D(3, 2), bezier_geometry::Point2D(1, 1)),
        0.5);
  CHECK(bezier_geometry::Point2D::getSlopeBetween(
            bezier_geometry::Point2D(2, 2), bezier_geometry::Point2D(1, 2)),
        0.0);
  CHECK(bezier_geometry::Point2D::getSlopeBetween(
            bezier_geometry::Point2D(1, 1), bezier_geometry::Point2D(2, 0)),
        -1.0);
  CHECK(bezier_geometry::Point2D::getSlopeBetween(
            bezier_geometry::Point2D(2, 0), bezier_geometry::Point2D(1, 1)),
        -1.0);
  {
    bezier_geometry::RealNum testSlope =
        bezier_geometry::Point2D::getSlopeBetween(
            bezier_geometry::Point2D(1, 1), bezier_geometry::Point2D(1, 2));
    CHECK(std::isinf(testSlope) && testSlope > 0, true);
  }
  {
    bezier_geometry::RealNum testSlope =
        bezier_geometry::Point2D::getSlopeBetween(
            bezier_geometry::Point2D(1, 2), bezier_geometry::Point2D(1, 1));
    CHECK(std::isinf(testSlope) && testSlope < 0, true);
  }
  TEST_END
}

int main(int argc, char **argv) {
  std::cout << "%SUITE_STARTING% Point2DTest" << std::endl;
  std::cout << "%SUITE_STARTED%" << std::endl;
  coordinatesDistanceEquivalenceTest();
  shiftRotateTest();
  colinearTest();
  slopeTest();
  std::cout << "%SUITE_FINISHED% time=0" << std::endl;
  return (EXIT_SUCCESS);
}
