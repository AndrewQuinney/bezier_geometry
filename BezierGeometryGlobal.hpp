/**
 * @file
 * \brief Global definitions shared throughout the module.
 *
 * Constant definitions which may be used to alter fundamental pieces of
 * behaviour.
 */
#ifndef BEZIERGEOMETRYGLOBAL_HPP
#define BEZIERGEOMETRYGLOBAL_HPP

#include <ostream>

namespace bezier_geometry {
/**
 * \brief The fundamental real number for all values/calculations.
 *
 * This type is used for all numeric parameters and calculations for all classes
 * and operations. It must support standard numeric operations and is expected
 * to have a sufficiently large range and precision. Generally this is a double,
 * but may be switched over to a float or some kind of 'big decimal' class,
 * depending on resource constraints.
 */
typedef double RealNum;

/**
 * \brief Debug output stream.
 *
 * The module outputs a large volume of debugging data that allows for logging
 * and reproduction of its internal processes. This value represents the output
 * stream that will be used to print this data. By default, this value is
 * STDOUT. This value is not cached and the method is called repeatedly for
 * every debug statement.
 * @return The output stream where debug data is to be written.
 */
std::ostream &debug_out();
} // namespace bezier_geometry

/**
 * \brief Call (or skip) debug output based on compile flags.
 *
 * All debug output from the module is done by calling 'debug() << ...', which
 * is an invocation of this macro. This effectively becomes a no-op when NDEBUG
 * is defined on compilation, meaning that in addition to no debug output being
 * generated, no parameters to the function are evaluated either.
 */
#ifdef NDEBUG
#define debug                                                                  \
  while (false)                                                                \
  bezier_geometry::debug_out
#else
#define debug bezier_geometry::debug_out
#endif

#endif
