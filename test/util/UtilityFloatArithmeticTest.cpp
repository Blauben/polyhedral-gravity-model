#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "polyhedralGravity/util/UtilityFloatArithmetic.h"

/** Contains the positive infinity values for doubles */
constexpr static double INF = std::numeric_limits<double>::infinity();

TEST(UtilityFloatArithmeticTest, TestAlmostEqualUlps) {
    // Checking the signess && identity
    ASSERT_FALSE(polyhedralGravity::util::almostEqualUlps(3.0, 4.0));
    ASSERT_FALSE(polyhedralGravity::util::almostEqualUlps(-3.0, -4.0));
    ASSERT_FALSE(polyhedralGravity::util::almostEqualUlps(-3.0, 4.0));
    ASSERT_FALSE(polyhedralGravity::util::almostEqualUlps(3.0, -4.0));
    ASSERT_TRUE(polyhedralGravity::util::almostEqualUlps(1.0, 1.0));

    // Some random values, which are equal
    ASSERT_TRUE(polyhedralGravity::util::almostEqualUlps(9.40569e-05, 9.40569e-05));
    ASSERT_TRUE(polyhedralGravity::util::almostEqualUlps(-0.000150712, -0.000150712));
    ASSERT_TRUE(polyhedralGravity::util::almostEqualUlps(0.000135291, 0.000135291));
    ASSERT_TRUE(polyhedralGravity::util::almostEqualUlps(-8.63978e-05, -8.63978e-05));

    /* FROM HERE IT GETS INTERESTING */

    // Checking the relative difference
    // The ULP distance is always greater than 4 for these "big" epsilons
    ASSERT_FALSE(polyhedralGravity::util::almostEqualUlps(3.0, 3.0 + 1e-9));
    ASSERT_FALSE(polyhedralGravity::util::almostEqualUlps(3.0, 3.0 + 1e-10));
    ASSERT_FALSE(polyhedralGravity::util::almostEqualUlps(3.0, 3.0 + 1e-11));

    // Checking the the sensitivity towards the next floating point
    // Note: The default maximal ULPS distance is set to 4
    // The ULP distance is one time higher than 4 leading to false, one time lower equal leading to true
    // Can be constexpr starting with C++23
    const double fourHops = std::nextafter(std::nextafter(std::nextafter(std::nextafter(3.0, INF), INF), INF), INF);
    const double fiveHops = std::nextafter(fourHops, INF);
    ASSERT_TRUE(polyhedralGravity::util::almostEqualUlps(3.0, fourHops));
    ASSERT_FALSE(polyhedralGravity::util::almostEqualUlps(3.0, fiveHops));


}

TEST(UtilityFloatArithmeticTest, TestAlmostEqualRelative) {
    // Checking the signess && identity
    ASSERT_FALSE(polyhedralGravity::util::almostEqualRelative(3.0, 4.0));
    ASSERT_FALSE(polyhedralGravity::util::almostEqualRelative(-3.0, -4.0));
    ASSERT_FALSE(polyhedralGravity::util::almostEqualRelative(-3.0, 4.0));
    ASSERT_FALSE(polyhedralGravity::util::almostEqualRelative(3.0, -4.0));
    ASSERT_TRUE(polyhedralGravity::util::almostEqualRelative(1.0, 1.0));

    // Some random values, which are equal
    ASSERT_TRUE(polyhedralGravity::util::almostEqualRelative(9.40569e-05, 9.40569e-05));
    ASSERT_TRUE(polyhedralGravity::util::almostEqualRelative(-0.000150712, -0.000150712));
    ASSERT_TRUE(polyhedralGravity::util::almostEqualRelative(0.000135291, 0.000135291));
    ASSERT_TRUE(polyhedralGravity::util::almostEqualRelative(-8.63978e-05, -8.63978e-05));

    /* FROM HERE IT GETS INTERESTING */

    // Checking the relative difference
    // Note: 1e-10 is the sensitvity of almostEqualRelative
    // The method returns true if the relative difference is smaller equal than 1e-10
    ASSERT_FALSE(polyhedralGravity::util::almostEqualRelative(3.0, 3.0 + 1e-9));
    ASSERT_TRUE(polyhedralGravity::util::almostEqualRelative(3.0, 3.0 + 1e-10));
    ASSERT_TRUE(polyhedralGravity::util::almostEqualRelative(3.0, 3.0 + 1e-11));

    // Checking the the sensitivity towards the next floating point
    // A ULP distance of 4 or 5 is still really small than our relative sensitivity of 1e-10
    // Can be constexpr starting with C++23
    const double fourHops = std::nextafter(std::nextafter(std::nextafter(std::nextafter(3.0, INF), INF), INF), INF);
    const double fiveHops = std::nextafter(fourHops, INF);
    ASSERT_TRUE(polyhedralGravity::util::almostEqualRelative(3.0, fourHops));
    ASSERT_TRUE(polyhedralGravity::util::almostEqualRelative(3.0, fiveHops));
}
