#ifndef TestUtils_HPP
#define TestUtils_HPP

#include <iostream>
#include <regex>

#include "BezierGeometryGlobal.hpp"

const std::regex extractSimpleName(".*[\\/]([^\\/]+)\\.[^.\\/]+",
                                   std::regex::ECMAScript);
bezier_geometry::RealNum
randomReal(const bezier_geometry::RealNum &lowerInclusive,
           const bezier_geometry::RealNum &upperInclusive) {
  return lowerInclusive +
         (std::rand() * ((upperInclusive - lowerInclusive) / RAND_MAX));
}

#define TEST_START                                                             \
  std::cout << "%TEST_STARTED% " << __func__ << " ("                           \
            << std::regex_replace(std::string(__FILE__), extractSimpleName,    \
                                  "$1")                                        \
            << ")" << std::endl;                                               \
  const auto __test_start__(std::chrono::high_resolution_clock::now());

#define TEST_END                                                               \
  std::cout << "%TEST_FINISHED% time="                                         \
            << std::chrono::duration_cast<std::chrono::seconds>(               \
                   std::chrono::high_resolution_clock::now() - __test_start__) \
                   .count()                                                    \
            << " " << __func__ << " ("                                         \
            << std::regex_replace(std::string(__FILE__), extractSimpleName,    \
                                  "$1")                                        \
            << ")" << std::endl;

#define CHECK(input, expected)                                                 \
  if (!((expected) == (input))) {                                              \
    std::cout << "%TEST_FAILED% time=0 testname=" << __func__ << " ("          \
              << std::regex_replace(std::string(__FILE__), extractSimpleName,  \
                                    "$1")                                      \
              << ") message=LINE: " << __LINE__ << " EXPECTED: " << (expected) \
              << " FOUND: " << (input) << std::endl;                           \
  }

#define CHECK_FUZZY(input, expected)                                           \
  if (!bezier_geometry::fuzzyEquals((input), (expected))) {                    \
    std::cout << "%TEST_FAILED% time=0 testname=" << __func__ << " ("          \
              << std::regex_replace(std::string(__FILE__), extractSimpleName,  \
                                    "$1")                                      \
              << ") message=LINE: " << __LINE__                                \
              << " EXPECTED: " << bezier_geometry::toString(expected)          \
              << " FOUND: " << bezier_geometry::toString(input) << std::endl;  \
  }
#endif
