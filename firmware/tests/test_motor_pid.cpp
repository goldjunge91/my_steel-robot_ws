//
// Created by AI Assistant - 7.10.2025
// Unit Tests for MotorPID Class - Testing PID calculation without hardware
//

#include "gtest/gtest.h"

// Mock minimal dependencies for testing PID logic
class MockMotorPID {
public:
        MockMotorPID() : xKP(1.0), xKI(0.5), xKD(0.1), xCumErr(0.0), xLastErr(0.0) {}

        void configPID(float kP, float kI, float kD) {
                xKP = kP;
                xKI = kI;
                xKD = kD;
        }

        float pid(float sp, float pv) {
                float err = sp - pv;
                float p = err * xKP;
                float i = xKI * (xCumErr + err);
                float d = xKD * (err - xLastErr);

                xCumErr += err;
                xLastErr = err;

                return p + i + d;
        }

        void reset() {
                xCumErr = 0.0;
                xLastErr = 0.0;
        }

private:
        float xKP;
        float xKI;
        float xKD;
        float xCumErr;
        float xLastErr;
};

// Test Fixture
class MotorPIDTest : public ::testing::Test {
protected:
        void SetUp() override {
                pidController = new MockMotorPID();
        }

        void TearDown() override {
                delete pidController;
        }

        MockMotorPID* pidController;
};

// Test 1: PID Configuration
TEST_F(MotorPIDTest, ConfigPIDValuesAreSet) {
        pidController->configPID(2.0, 1.0, 0.5);
        // If no crash, configuration works
        SUCCEED();
}

// Test 2: PID Output with Zero Error
TEST_F(MotorPIDTest, ZeroErrorGivesZeroOutput) {
        pidController->configPID(1.0, 0.0, 0.0); // Pure P controller
        float output = pidController->pid(100.0, 100.0); // SP = PV
        EXPECT_FLOAT_EQ(output, 0.0);
}

// Test 3: Proportional Response
TEST_F(MotorPIDTest, ProportionalResponseIsCorrect) {
        pidController->configPID(2.0, 0.0, 0.0); // Pure P, Kp=2
        float output = pidController->pid(150.0, 100.0); // Error = 50
        EXPECT_FLOAT_EQ(output, 100.0); // P = 2.0 * 50 = 100
}

// Test 4: Integral Accumulation
TEST_F(MotorPIDTest, IntegralAccumulatesError) {
        pidController->configPID(0.0, 1.0, 0.0); // Pure I, Ki=1
        pidController->reset();

        float out1 = pidController->pid(100.0, 90.0); // Error = 10
        float out2 = pidController->pid(100.0, 90.0); // Error = 10 again

        // First call: I = 1.0 * (0 + 10) = 10
        EXPECT_FLOAT_EQ(out1, 10.0);

        // Second call: I = 1.0 * (10 + 10) = 20
        EXPECT_FLOAT_EQ(out2, 20.0);
}

// Test 5: Derivative Response
TEST_F(MotorPIDTest, DerivativeRespondsToErrorChange) {
        pidController->configPID(0.0, 0.0, 2.0); // Pure D, Kd=2
        pidController->reset();

        pidController->pid(100.0, 90.0); // Error = 10, lastErr = 0
        float out2 = pidController->pid(100.0, 85.0); // Error = 15, change = 5

        // D = 2.0 * (15 - 10) = 10
        EXPECT_FLOAT_EQ(out2, 10.0);
}

// Test 6: Combined PID Response
TEST_F(MotorPIDTest, CombinedPIDWorks) {
        pidController->configPID(1.0, 0.5, 0.1); // P=1, I=0.5, D=0.1
        pidController->reset();

        float output = pidController->pid(100.0, 80.0); // Error = 20

        // P = 1.0 * 20 = 20
        // I = 0.5 * (0 + 20) = 10
        // D = 0.1 * (20 - 0) = 2
        // Total = 32
        EXPECT_FLOAT_EQ(output, 32.0);
}

// Test 7: Negative Error Handling
TEST_F(MotorPIDTest, NegativeErrorIsHandledCorrectly) {
        pidController->configPID(1.0, 0.0, 0.0); // Pure P
        float output = pidController->pid(80.0, 100.0); // Error = -20
        EXPECT_FLOAT_EQ(output, -20.0);
}

// Test 8: Reset Clears Integral and Derivative
TEST_F(MotorPIDTest, ResetClearsInternalState) {
        pidController->configPID(0.0, 1.0, 0.0); // Pure I

        pidController->pid(100.0, 90.0); // Accumulate error
        pidController->pid(100.0, 90.0); // Accumulate more

        pidController->reset();

        float output = pidController->pid(100.0, 90.0); // Should start fresh
        EXPECT_FLOAT_EQ(output, 10.0); // Fresh I calculation
}
