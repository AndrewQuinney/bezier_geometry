#include <stdlib.h>

#include <cmath>

#include "TestUtils.hpp"

void infinityTest() {
  TEST_START
  CHECK(std::isinf(std::numeric_limits<bezier_geometry::RealNum>::infinity()),
        true);
  CHECK(std::isinf(std::numeric_limits<bezier_geometry::RealNum>::infinity() *
                   -1),
        true);
  CHECK(std::isinf(5.0), false);
  CHECK(std::numeric_limits<bezier_geometry::RealNum>::infinity() > 5, true);
  CHECK(std::numeric_limits<bezier_geometry::RealNum>::infinity() < 5, false);
  CHECK(std::numeric_limits<bezier_geometry::RealNum>::infinity() * -1 > 5,
        false);
  CHECK(std::numeric_limits<bezier_geometry::RealNum>::infinity() * -1 < 5,
        true);
  CHECK(std::isinf(5.0 / 0.0), true);
  CHECK(std::isinf(std::numeric_limits<double>::infinity()), true);
  CHECK(std::isinf(std::numeric_limits<float>::infinity()), true);
  CHECK(std::isinf(std::numeric_limits<bezier_geometry::RealNum>::infinity()),
        true);
  CHECK(std::isinf(std::numeric_limits<bezier_geometry::RealNum>::infinity() *
                   (-1.0)),
        true);
  {
    bezier_geometry::RealNum bigValue = 99999999999;
    CHECK(std::isinf(bigValue), false);
    for (int i = 0; i < 5; i++) {
      bigValue = bigValue * bigValue;
    }
    CHECK(std::isinf(bigValue), true);
  }
  {
    bezier_geometry::RealNum bigValue = 99999999999;
    CHECK(std::isinf(bigValue), false);
    for (int i = 0; i < 40; i++) {
      bigValue = bigValue / 0.000000001;
    }
    CHECK(std::isinf(bigValue), true);
  }
  TEST_END
}

void NaNTest() {
  TEST_START
  CHECK(std::isnan(0.0 / 0.0), true);
  CHECK(std::isnan(std::numeric_limits<float>::quiet_NaN()), true);
  CHECK(std::isnan(std::numeric_limits<float>::signaling_NaN()), true);
  CHECK(std::isnan(std::numeric_limits<double>::quiet_NaN()), true);
  CHECK(std::isnan(std::numeric_limits<double>::signaling_NaN()), true);
  CHECK(std::isnan(std::numeric_limits<bezier_geometry::RealNum>::quiet_NaN()),
        true);
  CHECK(std::isnan(
            std::numeric_limits<bezier_geometry::RealNum>::signaling_NaN()),
        true);
  TEST_END
}

int main(int argc, char **argv) {
  std::cout << "%SUITE_STARTING% MathTest" << std::endl;
  std::cout << "%SUITE_STARTED%" << std::endl;
  infinityTest();
  NaNTest();
  std::cout << "%SUITE_FINISHED% time=0" << std::endl;
  return (EXIT_SUCCESS);
}
