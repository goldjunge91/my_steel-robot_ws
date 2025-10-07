//
// Created by AI Assistant - 7.10.2025
// Unit Tests for Odometry Calculations (Simplified Mathematical Tests)
//

#include "gtest/gtest.h"
#include <cmath>

// Robot constants from DDD.h
#define WHEEL_RADIUS 0.065
#define WHEEL_DEPTH 0.055
#define WHEELS_SEP 0.204
#define WHEELS_OFFSET 0.010

struct OdomState {
        double x;
        double y;
        double a; // angle in radians
};

// Simplified odometry - just test the math concepts
class SimpleOdometry {
public:
        SimpleOdometry() {
                odom = { 0.0, 0.0, 0.0 };
        }

        // Simplified: both wheels same distance = straight line
        void moveStraight(double distance) {
                odom.x += distance * cos(odom.a);
                odom.y += distance * sin(odom.a);
        }

        // Simplified: rotate in place
        void rotate(double angleChange) {
                odom.a += angleChange;
        }

        // Simplified differential drive: left and right wheel distances
        void updateDifferential(double leftDist, double rightDist) {
                double avgDist = (leftDist + rightDist) / 2.0;
                double deltaAngle = (rightDist - leftDist) / WHEELS_SEP;

                // Update angle first
                double newAngle = odom.a + deltaAngle;

                // Update position
                odom.x += avgDist * cos((odom.a + newAngle) / 2.0);
                odom.y += avgDist * sin((odom.a + newAngle) / 2.0);
                odom.a = newAngle;
        }

        OdomState getOdom() const { return odom; }
        void reset() { odom = { 0.0, 0.0, 0.0 }; }

private:
        OdomState odom;
};

// Test Fixture
class OdometryTest : public ::testing::Test {
protected:
        void SetUp() override {
                odom = new SimpleOdometry();
        }

        void TearDown() override {
                delete odom;
        }

        SimpleOdometry* odom;
};

// Test 1: Initial State
TEST_F(OdometryTest, InitialStateIsZero) {
        OdomState state = odom->getOdom();
        EXPECT_DOUBLE_EQ(state.x, 0.0);
        EXPECT_DOUBLE_EQ(state.y, 0.0);
        EXPECT_DOUBLE_EQ(state.a, 0.0);
}

// Test 2: Straight Forward Motion
TEST_F(OdometryTest, StraightForwardMovement) {
        odom->moveStraight(1.0); // Move 1 meter forward

        OdomState state = odom->getOdom();
        EXPECT_NEAR(state.x, 1.0, 0.001);
        EXPECT_NEAR(state.y, 0.0, 0.001);
        EXPECT_NEAR(state.a, 0.0, 0.001);
}

// Test 3: Backward Motion
TEST_F(OdometryTest, BackwardMovement) {
        odom->moveStraight(-1.0); // Move 1 meter backward

        OdomState state = odom->getOdom();
        EXPECT_NEAR(state.x, -1.0, 0.001);
        EXPECT_NEAR(state.y, 0.0, 0.001);
}

// Test 4: Pure Rotation
TEST_F(OdometryTest, PureRotation) {
        odom->rotate(M_PI / 2); // Rotate 90 degrees

        OdomState state = odom->getOdom();
        EXPECT_NEAR(state.x, 0.0, 0.001);
        EXPECT_NEAR(state.y, 0.0, 0.001);
        EXPECT_NEAR(state.a, M_PI / 2, 0.001);
}

// Test 5: Move After Rotation
TEST_F(OdometryTest, MoveAfterRotation) {
        odom->rotate(M_PI / 2);      // Rotate 90 degrees
        odom->moveStraight(1.0);     // Move 1 meter in new direction

        OdomState state = odom->getOdom();
        EXPECT_NEAR(state.x, 0.0, 0.001); // Should move in Y direction
        EXPECT_NEAR(state.y, 1.0, 0.001);
        EXPECT_NEAR(state.a, M_PI / 2, 0.001);
}

// Test 6: Differential Drive - Both Wheels Equal (Straight)
TEST_F(OdometryTest, DifferentialDriveStraight) {
        odom->updateDifferential(0.5, 0.5); // Both wheels 0.5m

        OdomState state = odom->getOdom();
        EXPECT_NEAR(state.x, 0.5, 0.001);
        EXPECT_NEAR(state.y, 0.0, 0.001);
        EXPECT_NEAR(state.a, 0.0, 0.001);
}

// Test 7: Differential Drive - Turn
TEST_F(OdometryTest, DifferentialDriveTurn) {
        odom->updateDifferential(0.3, 0.5); // Right wheel travels more

        OdomState state = odom->getOdom();
        double expectedAngle = (0.5 - 0.3) / WHEELS_SEP;

        EXPECT_NE(state.a, 0.0); // Should have rotated
        EXPECT_NEAR(state.a, expectedAngle, 0.001);
}

// Test 8: Reset Functionality
TEST_F(OdometryTest, ResetClearsOdometry) {
        odom->moveStraight(1.0);
        odom->rotate(M_PI / 4);
        odom->reset();

        OdomState state = odom->getOdom();
        EXPECT_DOUBLE_EQ(state.x, 0.0);
        EXPECT_DOUBLE_EQ(state.y, 0.0);
        EXPECT_DOUBLE_EQ(state.a, 0.0);
}
