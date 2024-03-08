#include "GeometryUtil.hpp"
#include "TestUtils.hpp"

void sufficientlySmallTest() {
  TEST_START
  CHECK(bezier_geometry::sufficientlySmall(0), true);
  CHECK(bezier_geometry::sufficientlySmall(1), false);
  CHECK(bezier_geometry::sufficientlySmall(-1), false);
  CHECK(bezier_geometry::sufficientlySmall(0.00009), true);
  CHECK(bezier_geometry::sufficientlySmall(-0.00009), true);
  CHECK(bezier_geometry::sufficientlySmall(0.0001), false);
  CHECK(bezier_geometry::sufficientlySmall(-0.0001), false);
  TEST_END
}

void sufficientlyCloseTest() {
  TEST_START
  typedef std::pair<
      std::pair<bezier_geometry::RealNum, bezier_geometry::RealNum>, bool>
      paramEntry;
  std::vector<paramEntry> paramEntries;
  {
    paramEntries.push_back(paramEntry());
    paramEntries.back().first.first = 0;
    paramEntries.back().first.second = 0;
    paramEntries.back().second = true;
  }
  {
    paramEntries.push_back(paramEntry());
    paramEntries.back().first.first = 0;
    paramEntries.back().first.second = 1;
    paramEntries.back().second = false;
  }
  {
    paramEntries.push_back(paramEntry());
    paramEntries.back().first.first = 1;
    paramEntries.back().first.second = 1;
    paramEntries.back().second = true;
  }
  {
    paramEntries.push_back(paramEntry());
    paramEntries.back().first.first = 1;
    paramEntries.back().first.second = 0.9998;
    paramEntries.back().second = false;
  }
  {
    paramEntries.push_back(paramEntry());
    paramEntries.back().first.first = 1;
    paramEntries.back().first.second = 0.9999;
    paramEntries.back().second = true;
  }
  {
    paramEntries.push_back(paramEntry());
    paramEntries.back().first.first = 1;
    paramEntries.back().first.second = 1.00011;
    paramEntries.back().second = false;
  }
  {
    paramEntries.push_back(paramEntry());
    paramEntries.back().first.first = 1;
    paramEntries.back().first.second = 1.0001;
    paramEntries.back().second = true;
  }
  {
    paramEntries.push_back(paramEntry());
    paramEntries.back().first.first = 1;
    paramEntries.back().first.first =
        std::numeric_limits<bezier_geometry::RealNum>::infinity();
    paramEntries.back().first.second = 1;
    paramEntries.back().second = false;
  }
  {
    paramEntries.push_back(paramEntry());
    paramEntries.back().first.first = 1;
    paramEntries.back().first.first =
        std::numeric_limits<bezier_geometry::RealNum>::infinity();
    paramEntries.back().first.second = 2;
    paramEntries.back().first.second =
        std::numeric_limits<bezier_geometry::RealNum>::infinity();
    paramEntries.back().second = true;
  }
  {
    paramEntries.push_back(paramEntry());
    paramEntries.back().first.first = 5;
    paramEntries.back().first.first = paramEntries.back().first.first / 99999;
    paramEntries.back().first.second = 0;
    paramEntries.back().second = true;
  }
  {
    paramEntries.push_back(paramEntry());
    paramEntries.back().first.first = 5;
    paramEntries.back().first.first = paramEntries.back().first.first / 9999;
    paramEntries.back().first.second = 0;
    paramEntries.back().second = false;
  }
  for (std::vector<paramEntry>::iterator i = paramEntries.begin();
       i != paramEntries.end(); i++) {
    CHECK(bezier_geometry::sufficientlyClose(i->first.first, i->first.second),
          i->second);
    CHECK(bezier_geometry::sufficientlyClose(i->first.second, i->first.first),
          i->second);
    CHECK(bezier_geometry::sufficientlyClose(i->first.first * -1,
                                             i->first.second * -1),
          i->second);
    CHECK(bezier_geometry::sufficientlyClose(i->first.second * -1,
                                             i->first.first * -1),
          i->second);
  }
  TEST_END
}

typedef struct {
  bezier_geometry::RealNum slope1;
  bezier_geometry::RealNum slope2;
  bool expectedResult;
} SufficientlyCloseSlopesTestEntry;

void sufficientlyCloseSlopesTest() {
  TEST_START
  std::vector<SufficientlyCloseSlopesTestEntry> testEntries;
  testEntries.push_back({1, 0, false});
  testEntries.push_back({1, 1.001, false});
  testEntries.push_back({1, 1, true});
  testEntries.push_back({1,
                         // 1.00009,
                         bezier_geometry::rotateSlope(1, 0.00009), true});
  testEntries.push_back({1,
                         // 1.0001,
                         bezier_geometry::rotateSlope(1, 0.00008), true});
  testEntries.push_back(
      {std::numeric_limits<bezier_geometry::RealNum>::infinity(), 1.0001,
       false});
  testEntries.push_back(
      {std::numeric_limits<bezier_geometry::RealNum>::infinity(), 10000000,
       true});
  testEntries.push_back(
      {static_cast<bezier_geometry::RealNum>(
           (-1.0) * std::numeric_limits<bezier_geometry::RealNum>::infinity()),
       10000000, true});
  testEntries.push_back(
      {std::numeric_limits<bezier_geometry::RealNum>::infinity(), -10000000,
       /*false*/ true});
  testEntries.push_back(
      {std::numeric_limits<bezier_geometry::RealNum>::infinity(), 1000000,
       /*false*/ true});
  for (const SufficientlyCloseSlopesTestEntry &currentEntry : testEntries) {
    debug() << "*** BEFORE comparing slopes -"
            << "slope1:" << bezier_geometry::toString(currentEntry.slope1)
            << "slope2:" << bezier_geometry::toString(currentEntry.slope2)
            << "expected:" << currentEntry.expectedResult;
    CHECK(bezier_geometry::sufficientlyCloseSlopes(currentEntry.slope1,
                                                   currentEntry.slope2),
          currentEntry.expectedResult);
    CHECK(bezier_geometry::sufficientlyCloseSlopes(currentEntry.slope2,
                                                   currentEntry.slope1),
          currentEntry.expectedResult);
    bezier_geometry::RealNum outputSlope1;
    bezier_geometry::RealNum outputSlope2;
    bezier_geometry::sufficientlyDifferentSlopes(currentEntry.slope1,
                                                 outputSlope1, outputSlope2);
    CHECK(bezier_geometry::sufficientlyCloseSlopes(currentEntry.slope1,
                                                   outputSlope1),
          false);
    CHECK(bezier_geometry::sufficientlyCloseSlopes(currentEntry.slope1,
                                                   outputSlope2),
          false);
    bezier_geometry::sufficientlyDifferentSlopes(currentEntry.slope2,
                                                 outputSlope1, outputSlope2);
    CHECK(bezier_geometry::sufficientlyCloseSlopes(currentEntry.slope2,
                                                   outputSlope1),
          false);
    CHECK(bezier_geometry::sufficientlyCloseSlopes(currentEntry.slope2,
                                                   outputSlope2),
          false);
  }
  TEST_END
}

int main(int argc, char **argv) {
  std::cout << "%SUITE_STARTING% GeometryUtilTest" << std::endl;
  std::cout << "%SUITE_STARTED%" << std::endl;
  sufficientlySmallTest();
  sufficientlyCloseTest();
  sufficientlyCloseSlopesTest();
  std::cout << "%SUITE_FINISHED% time=0" << std::endl;
  return (EXIT_SUCCESS);
}
