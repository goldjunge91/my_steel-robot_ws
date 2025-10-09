//
// Created by AI Assistant - 7.10.2025
// Basic Math and Utility Tests
//

#include "gtest/gtest.h"
#include <cmath>

// Test mathematical constants and conversions
class MathUtilsTest : public ::testing::Test {
};

// Test 1: RPM to Rad/s Conversion
TEST_F(MathUtilsTest, RPMtoRadPerSecConversion) {
        // From MotorPID::setSpeedRadPS()
        float rps = 1.0; // rad/s
        float rpm = rps / (M_PI * 2.0);
        rpm = rpm * 60.0;

        // 1 rad/s = 60/(2*PI) RPM ≈ 9.549 RPM
        EXPECT_NEAR(rpm, 9.549, 0.01);
}

// Test 2: Wheel Circumference Calculation
TEST_F(MathUtilsTest, WheelCircumferenceIsCorrect) {
        const double WHEEL_RADIUS = 0.065; // meters
        double circumference = WHEEL_RADIUS * 2.0 * M_PI;

        // Expected: 2 * PI * 0.065 ≈ 0.408 meters
        EXPECT_NEAR(circumference, 0.408, 0.01);
}

// Test 3: Radians to Degrees Conversion
TEST_F(MathUtilsTest, RadiansToDegrees) {
        double radians = M_PI;
        double degrees = radians / M_PI * 180.0;

        EXPECT_DOUBLE_EQ(degrees, 180.0);
}

// Test 4: Degrees to Radians Conversion
TEST_F(MathUtilsTest, DegreesToRadians) {
        double degrees = 90.0;
        double radians = degrees * M_PI / 180.0;

        EXPECT_DOUBLE_EQ(radians, M_PI / 2.0);
}

// Test 5: Angle Wrapping (-PI to PI)
TEST_F(MathUtilsTest, AngleWrapping) {
        auto wrapAngle = [](double angle) {
                while (angle > M_PI) angle -= 2.0 * M_PI;
                while (angle < -M_PI) angle += 2.0 * M_PI;
                return angle;
                };

        EXPECT_NEAR(wrapAngle(3.5 * M_PI), -0.5 * M_PI, 0.001);
        EXPECT_NEAR(wrapAngle(-3.5 * M_PI), 0.5 * M_PI, 0.001);
}

// Test 6: Distance Calculation (2D)
TEST_F(MathUtilsTest, DistanceCalculation2D) {
        double x1 = 0.0, y1 = 0.0;
        double x2 = 3.0, y2 = 4.0;

        double distance = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));

        EXPECT_DOUBLE_EQ(distance, 5.0); // 3-4-5 triangle
}

// Test 7: Velocity to Motor Speed Conversion
TEST_F(MathUtilsTest, VelocityToMotorSpeed) {
        const double WHEEL_RADIUS = 0.065;
        double velocityMPS = 1.0; // 1 meter per second
        double circumference = WHEEL_RADIUS * 2.0 * M_PI;

        double rps = (velocityMPS / circumference) * (2 * M_PI);

        // Expected: ~15.38 rad/s
        EXPECT_NEAR(rps, 15.38, 0.1);
}

// Test 8: Sign Function
TEST_F(MathUtilsTest, SignFunction) {
        auto sign = [](double val) {
                if (val > 0) return 1;
                if (val < 0) return -1;
                return 0;
                };

        EXPECT_EQ(sign(5.0), 1);
        EXPECT_EQ(sign(-5.0), -1);
        EXPECT_EQ(sign(0.0), 0);
}

// Test 9: Clamp Function
TEST_F(MathUtilsTest, ClampFunction) {
        auto clamp = [](double value, double min, double max) {
                if (value < min) return min;
                if (value > max) return max;
                return value;
                };

        EXPECT_DOUBLE_EQ(clamp(5.0, 0.0, 10.0), 5.0);
        EXPECT_DOUBLE_EQ(clamp(-5.0, 0.0, 10.0), 0.0);
        EXPECT_DOUBLE_EQ(clamp(15.0, 0.0, 10.0), 10.0);
}

// Test 10: Linear Interpolation
TEST_F(MathUtilsTest, LinearInterpolation) {
        auto lerp = [](double a, double b, double t) {
                return a + t * (b - a);
                };

        EXPECT_DOUBLE_EQ(lerp(0.0, 10.0, 0.5), 5.0);
        EXPECT_DOUBLE_EQ(lerp(0.0, 10.0, 0.0), 0.0);
        EXPECT_DOUBLE_EQ(lerp(0.0, 10.0, 1.0), 10.0);
}
