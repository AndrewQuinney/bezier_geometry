#ifndef GeometryUtil_HPP_
#define GeometryUtil_HPP_

#include "BezierGeometryGlobal.hpp"

namespace bezier_geometry {
/**
 * \defgroup GeometryUtil Global Utilities
 * @{
 */
/**
 * \defgroup SufficientlyClose Close Enough Logic
 *
 * Systems that make use of this module will often generate edge cases where the
 * results of previous calculations are relied upon for subsequent logic.
 * Therefore it is important that calculation results never contradict one
 * another - for example, if an object can move left 1 unit without colliding
 * then the moved object should be able to move right 1 unit if nothing else has
 * changed. To ensure that this is the case when working with iteratively
 * calculated floating point values, an error margin must be tolerated. This
 * logic forms the core of this error tolerance.
 * @{
 */
/**
 * \brief The maximum acceptable error margin.
 *
 * This applies to the deliberately variability-tolerant calculations the module
 * performs.
 */
extern const RealNum ACCEPTABLE_ERROR_MARGIN;
/**
 * \brief Tests if an input number is small enough to be considered 0.
 *
 * @return True if the absolute value of the difference between the input value
 * and 0 is small enough.
 */
bool sufficientlySmall(const RealNum &input /**< The value to be tested. */
);
/**
 * \brief Tests if two values are close enough to be considered equal.
 *
 * The ordering of the parameters will not affect the output.
 *
 * @return True if the difference between the two input values is small enough.
 */
bool sufficientlyClose(
    const RealNum &input1, /**< The first value to be tested for equality. */
    const RealNum &input2  /**< The second value to be tested for equality. */
);
/**
 * \brief Tests if two slope values are close enough to be considered equal.
 *
 * Testing slopes for equality cannot be done with their numeric values, as
 * differences are highly exaggerated at high values. Instead the angle between
 * the lines formed by the origin and the 2 slopes is tested and if it is
 * sufficiently small then the slopes are considered equal. Note that this
 * implies that very large negative and very large positive slopes are
 * considered equal.
 *
 * The ordering of the parameters will not affect the output.
 *
 * @return True if the angle between the two input slopes is small enough.
 */
bool sufficientlyCloseSlopes(
    const RealNum
        &slope1, /**< The first slope value to be tested for equality. */
    const RealNum
        &slope2 /**< The second slope value to be tested for equality. */
);
/**
 * \brief Calculates 2 slopes that are not close enough to the input slope.
 *
 * Given an input slope, calculate 2 additional slopes that are as close as
 * possible to the input slope while not being considered close enough to be
 * considered equal. The 2 slopes are derived by rotating the input slope a
 * small amount clockwise and counterclockwise. \see ClockwiseLogic
 */
void sufficientlyDifferentSlopes(
    const RealNum &slope, /**< The input slope for which different slopes are to
                             be calculated. */
    RealNum &outputSlope1, /**< Destination for the first calculated slope,
                              derived from a counterclockwise rotation. */
    RealNum &outputSlope2  /**< Destination for the second calculated slope,
                              derived from a clockwise rotation. */
);
/**@}*/
/**
 * \brief Calculate the resulting slope if a slope is rotated.
 *
 * Given an input slope and an angle of rotation, calculate the slope that would
 * result from rotating a line with the given slope that passes through the
 * origin about the origin by a given angle.
 *
 * \see DirectionalLogic
 * \see ClockwiseLogic
 *
 * @return The slope that would result from the input slope and rotation.
 */
RealNum rotateSlope(
    const RealNum &slope, /**< The slope that is to be rotated. */
    const RealNum
        &rotationAngle /**< The angle (in degrees) that the input slope is to be
                          rotated. Negative values are clockwise. */
);
/**
 * \brief Test of two values are almost exactly equal.
 *
 * This is not the same logic as 'close enough'. Instead it compares two values
 * for equality with a very small variation tolerance to compensate for the
 * general inconsistency of testing floating-point values for equality. The
 * ordering of input parameters will not affect the output. \see
 * SufficientlyClose
 *
 * @return True if the 2 input values are virtually equal.
 */
bool fuzzyEquals(const RealNum &input1, /**< The first value to be compared. */
                 const RealNum &input2  /**< The second value to be compared. */
);
/**
 * \brief Test if a value is almost exactly 0.
 *
 * Unlike the 'close enough' logic, this tests for equality with only a very
 * small tolerance for error to compensate for the inconsistency of floating
 * point equivalence. \see fuzzyEquals
 *
 * @return True if the input value is virtually 0.
 */
bool fuzzyIsNull(const RealNum &input /**< The value to be tested for near
                                         equivalence to 0. */
);
/**
 * \brief Convert radians to degrees.
 *
 * @return The input radians converted to degrees.
 */
RealNum radiansToDegrees(
    const RealNum &radians /**< The radian value to be converted to degrees. */
);
/**
 * \brief Convert degrees to radians.
 *
 * @return The input degrees converted to radians.
 */
RealNum degreesToRadians(
    const RealNum &degrees /**< The degree value to be converted to radians. */
);
/**
 * \brief Convert the input number to a string.
 *
 * The input number is represented as a string in base 10 with a precision of up
 * to 20 decimal points.
 *
 * @return A string representation of the input number.
 */
std::string
toString(const RealNum &input /**< The numeric value to be converted. */
);
/**@}*/
} // namespace bezier_geometry

#endif
