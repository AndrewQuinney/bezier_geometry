#include "GeometryUtil.hpp"

#include <math.h>

#include <cmath>
#include <iomanip>
#include <sstream>

#include "Point2D.hpp"

namespace bezier_geometry {
const RealNum ACCEPTABLE_ERROR_MARGIN = 0.0001;
const RealNum RADIANS_DEGREES_COEFF = 180.0 / M_PI;

bool sufficientlySmall(const RealNum &input) {
  return std::fabs(input) < ACCEPTABLE_ERROR_MARGIN;
}

bool sufficientlyClose(const RealNum &input1, const RealNum &input2) {
  if (std::isinf(input1) != std::isinf(input2)) {
    return false;
  } else if (std::isinf(input1) && std::isinf(input2)) {
    return true;
  }
  return sufficientlySmall(input1 - input2);
}

bool sufficientlyCloseSlopes(const RealNum &slope1, const RealNum &slope2) {
  const RealNum aTanDiff = std::fabs(std::atan(slope1) - std::atan(slope2));
  return sufficientlySmall(
      radiansToDegrees(aTanDiff <= M_PI_2 ? aTanDiff : (M_PI - aTanDiff)));
}

namespace {
RealNum binarySearchForSufficientlyDifferent(const RealNum &slope,
                                             RealNum upperBoundATan) {
  RealNum lowerBoundATan = std::atan(slope);
  for (int i = 0; i < 10; i++) {
    RealNum currentGuessATan = (lowerBoundATan + upperBoundATan) / 2.0;
    if (sufficientlyCloseSlopes(std::tan(currentGuessATan), slope)) {
      lowerBoundATan = currentGuessATan;
    } else {
      upperBoundATan = currentGuessATan;
    }
  }
  return std::tan(upperBoundATan);
}
} // namespace

void sufficientlyDifferentSlopes(const RealNum &slope, RealNum &outputSlope1,
                                 RealNum &outputSlope2) {
  const RealNum slopeATan = std::atan(slope);
  {
    RealNum upperBoundATan = slopeATan;
    while (sufficientlyCloseSlopes(slope, std::tan(upperBoundATan))) {
      upperBoundATan += ACCEPTABLE_ERROR_MARGIN;
    }
    outputSlope1 = binarySearchForSufficientlyDifferent(slope, upperBoundATan);
  }
  {
    RealNum upperBoundATan = slopeATan;
    while (sufficientlyCloseSlopes(slope, std::tan(upperBoundATan))) {
      upperBoundATan -= ACCEPTABLE_ERROR_MARGIN;
    }
    outputSlope2 = binarySearchForSufficientlyDifferent(slope, upperBoundATan);
  }
}

/*
-returns the slope that results in rotating the input slope

-the input rotation angle is in degrees
*/
RealNum rotateSlope(const RealNum &slope, const RealNum &rotationAngle) {
  return std::tan(std::atan(slope) + degreesToRadians(rotationAngle));
}

bool fuzzyEquals(const RealNum &input1, const RealNum &input2) {
  return fuzzyIsNull(input1 - input2);
}

bool fuzzyIsNull(const RealNum &input) {
  return std::fabs(input) < 0.000000000001;
}

RealNum radiansToDegrees(const RealNum &radians) {
  return RADIANS_DEGREES_COEFF * radians;
}

RealNum degreesToRadians(const RealNum &degrees) {
  return degrees / RADIANS_DEGREES_COEFF;
}

std::string toString(const RealNum &input) {
  std::stringstream stream;
  stream << std::fixed << std::setprecision(20) << input;
  return stream.str();
}
} // namespace bezier_geometry
