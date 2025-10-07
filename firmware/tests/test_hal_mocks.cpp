#include <gtest/gtest.h>
#include "hal/mocks/I2CMock.hpp"
#include "hal/mocks/GPIOMock.hpp"

/**
 * @brief Tests for HAL I2C Mock
 *
 * These tests demonstrate how to use the I2C mock for testing
 * I2C-based sensors without real hardware.
 */
class I2CMockTest : public ::testing::Test {
protected:
        void SetUp() override {
                i2c_mock.reset();
        }

        hal::mocks::I2CMock i2c_mock;
};

// ============================================================================
// Basic I2C Mock Tests
// ============================================================================

TEST_F(I2CMockTest, InitializeI2CBus) {
        EXPECT_TRUE(i2c_mock.init(0, 400000));
        EXPECT_TRUE(i2c_mock.is_initialized());
}

TEST_F(I2CMockTest, ConfigurePins) {
        i2c_mock.configure_pins(16, 17, true);
        // No assertion needed, just checking it doesn't crash
}

TEST_F(I2CMockTest, WriteRegisterValue) {
        i2c_mock.init(0, 400000);

        // Simulate writing register 0x06 = 0x01 on device 0x68
        uint8_t data[] = { 0x06, 0x01 };  // reg_addr, value
        int written = i2c_mock.write_blocking(0, 0x68, data, 2, false);

        EXPECT_EQ(written, 2);
        EXPECT_EQ(i2c_mock.get_register(0x68, 0x06), 0x01);
}

TEST_F(I2CMockTest, ReadRegisterValue) {
        i2c_mock.init(0, 400000);

        // Pre-set a register value
        i2c_mock.set_register(0x68, 0x00, 0xEA);  // WHO_AM_I register

        // Read it back using write_read pattern
        uint8_t value = 0;
        bool success = i2c_mock.write_read(0, 0x68, 0x00, &value, 1);

        EXPECT_TRUE(success);
        EXPECT_EQ(value, 0xEA);
}

TEST_F(I2CMockTest, ReadMultipleRegisters) {
        i2c_mock.init(0, 400000);

        // Simulate accelerometer data (6 bytes)
        uint8_t accel_data[] = { 0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC };
        i2c_mock.set_registers(0x68, 0x2D, accel_data, 6);

        // Read them back
        uint8_t buffer[6] = { 0 };
        bool success = i2c_mock.write_read(0, 0x68, 0x2D, buffer, 6);

        EXPECT_TRUE(success);
        for (int i = 0; i < 6; i++) {
                EXPECT_EQ(buffer[i], accel_data[i]);
        }
}

TEST_F(I2CMockTest, ErrorSimulation) {
        i2c_mock.init(0, 400000);
        i2c_mock.simulate_error(true);

        // All operations should fail
        EXPECT_FALSE(i2c_mock.init(1, 100000));

        uint8_t data[] = { 0x00, 0x01 };
        EXPECT_EQ(i2c_mock.write_blocking(0, 0x68, data, 2, false), -1);

        uint8_t value = 0;
        EXPECT_FALSE(i2c_mock.write_read(0, 0x68, 0x00, &value, 1));
}

TEST_F(I2CMockTest, TransactionCounting) {
        i2c_mock.init(0, 400000);

        EXPECT_EQ(i2c_mock.get_write_count(), 0);
        EXPECT_EQ(i2c_mock.get_read_count(), 0);

        uint8_t data[] = { 0x06, 0x01 };
        i2c_mock.write_blocking(0, 0x68, data, 2, false);
        EXPECT_EQ(i2c_mock.get_write_count(), 1);

        uint8_t value = 0;
        i2c_mock.write_read(0, 0x68, 0x00, &value, 1);
        EXPECT_EQ(i2c_mock.get_write_count(), 2);  // write_read does both
        EXPECT_EQ(i2c_mock.get_read_count(), 1);
}

// ============================================================================
// GPIO Mock Tests
// ============================================================================

class GPIOMockTest : public ::testing::Test {
protected:
        void SetUp() override {
                gpio_mock.reset();
        }

        hal::mocks::GPIOMock gpio_mock;
};

TEST_F(GPIOMockTest, InitializePin) {
        gpio_mock.init(16);
        EXPECT_TRUE(gpio_mock.is_initialized(16));
        EXPECT_FALSE(gpio_mock.is_initialized(17));
}

TEST_F(GPIOMockTest, SetDirection) {
        gpio_mock.init(16);
        gpio_mock.set_dir(16, hal::mocks::GPIOMock::OUTPUT);

        EXPECT_EQ(gpio_mock.get_direction(16), hal::mocks::GPIOMock::OUTPUT);
}

TEST_F(GPIOMockTest, SetFunction) {
        gpio_mock.init(16);
        gpio_mock.set_function(16, hal::mocks::GPIOMock::FUNC_I2C);

        EXPECT_EQ(gpio_mock.get_function(16), hal::mocks::GPIOMock::FUNC_I2C);
}

TEST_F(GPIOMockTest, ReadWriteValue) {
        gpio_mock.init(16);
        gpio_mock.set_dir(16, hal::mocks::GPIOMock::OUTPUT);

        gpio_mock.put(16, true);
        EXPECT_TRUE(gpio_mock.get(16));

        gpio_mock.put(16, false);
        EXPECT_FALSE(gpio_mock.get(16));
}

TEST_F(GPIOMockTest, PullUpPullDown) {
        gpio_mock.init(16);

        gpio_mock.pull_up(16);
        EXPECT_TRUE(gpio_mock.is_pullup_enabled(16));
        EXPECT_FALSE(gpio_mock.is_pulldown_enabled(16));

        gpio_mock.pull_down(16);
        EXPECT_FALSE(gpio_mock.is_pullup_enabled(16));
        EXPECT_TRUE(gpio_mock.is_pulldown_enabled(16));

        gpio_mock.disable_pulls(16);
        EXPECT_FALSE(gpio_mock.is_pullup_enabled(16));
        EXPECT_FALSE(gpio_mock.is_pulldown_enabled(16));
}

// ============================================================================
// Simulated IMU Sensor Test (Example)
// ============================================================================

/**
 * @brief Example test showing how to simulate an IMU sensor
 *
 * This demonstrates the pattern for testing I2C sensors:
 * 1. Initialize mock with expected register values
 * 2. Perform operations
 * 3. Verify results
 */
TEST_F(I2CMockTest, SimulatedIMUSensorReading) {
        const uint8_t IMU_ADDR = 0x68;
        const uint8_t WHO_AM_I_REG = 0x00;
        const uint8_t ACCEL_XOUT_H = 0x2D;

        // Initialize I2C
        i2c_mock.init(0, 400000);
        i2c_mock.configure_pins(16, 17, true);

        // Simulate WHO_AM_I register response
        i2c_mock.set_register(IMU_ADDR, WHO_AM_I_REG, 0xEA);

        // Verify sensor identity
        uint8_t who_am_i = 0;
        EXPECT_TRUE(i2c_mock.write_read(0, IMU_ADDR, WHO_AM_I_REG, &who_am_i, 1));
        EXPECT_EQ(who_am_i, 0xEA);

        // Simulate accelerometer reading: X=1.0g, Y=0.0g, Z=0.0g
        // Raw value for 1.0g at ±2g range: 16384 (0x4000)
        uint8_t accel_data[] = {
            0x40, 0x00,  // X high, low
            0x00, 0x00,  // Y high, low
            0x00, 0x00   // Z high, low
        };
        i2c_mock.set_registers(IMU_ADDR, ACCEL_XOUT_H, accel_data, 6);

        // Read accelerometer data
        uint8_t buffer[6] = { 0 };
        EXPECT_TRUE(i2c_mock.write_read(0, IMU_ADDR, ACCEL_XOUT_H, buffer, 6));

        // Verify data
        int16_t accel_x = (buffer[0] << 8) | buffer[1];
        int16_t accel_y = (buffer[2] << 8) | buffer[3];
        int16_t accel_z = (buffer[4] << 8) | buffer[5];

        EXPECT_EQ(accel_x, 16384);  // 1.0g
        EXPECT_EQ(accel_y, 0);
        EXPECT_EQ(accel_z, 0);

        // Convert to g (using same formula as real IMU)
        const float ACC_SENS = 16384.0f;
        float x_g = static_cast<float>(accel_x) / ACC_SENS;

        EXPECT_FLOAT_EQ(x_g, 1.0f);
}
