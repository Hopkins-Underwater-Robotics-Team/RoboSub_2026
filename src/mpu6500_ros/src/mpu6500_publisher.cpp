#include <memory>
#include <chrono>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"

// Include your MPU6500 driver headers
extern "C" {
    #include "driver_mpu6500.h"
    #include "driver_mpu6500_interface.h"
}

using namespace std::chrono_literals;

// Dummy SPI functions since we're not using SPI
extern "C" {
    uint8_t mpu6500_interface_spi_init(void) { return 0; }
    uint8_t mpu6500_interface_spi_deinit(void) { return 0; }
    uint8_t mpu6500_interface_spi_read(uint8_t reg, uint8_t *buf, uint16_t len) { 
        (void)reg; (void)buf; (void)len;
        return 1; // Always fail
    }
    uint8_t mpu6500_interface_spi_write(uint8_t reg, uint8_t *buf, uint16_t len) { 
        (void)reg; (void)buf; (void)len;
        return 1; // Always fail
    }
    
    // Required callback functions
    void mpu6500_interface_receive_callback(uint8_t type) {
        // Empty callback - we're polling instead of using interrupts
        (void)type; // Suppress unused parameter warning
    }
    
    void mpu6500_interface_dmp_tap_callback(uint8_t count, uint8_t direction) {
        // Empty callback - not using DMP tap detection
        (void)count;
        (void)direction;
    }
    
    void mpu6500_interface_dmp_orient_callback(uint8_t orientation) {
        // Empty callback - not using DMP orientation detection
        (void)orientation;
    }
}

class MPU6500Publisher : public rclcpp::Node
{
public:
    MPU6500Publisher() : Node("mpu6500_publisher")
    {
        // Parameters
        this->declare_parameter<std::string>("frame_id", "imu_link");
        this->declare_parameter<std::string>("topic_imu", "/imu/data_raw");
        this->declare_parameter<std::string>("topic_mag", "/imu/mag");
        this->declare_parameter<double>("publish_rate", 100.0);
        this->declare_parameter<int>("i2c_address", 0x68);
        
        this->get_parameter("frame_id", frame_id_);
        this->get_parameter("topic_imu", topic_imu_);
        this->get_parameter("topic_mag", topic_mag_);
        this->get_parameter("publish_rate", publish_rate_);
        this->get_parameter("i2c_address", i2c_address_);

        // Initialize publishers
        imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(topic_imu_, 10);
        mag_publisher_ = this->create_publisher<sensor_msgs::msg::MagneticField>(topic_mag_, 10);

        // Initialize MPU6500
        if (initialize_mpu6500() != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize MPU6500");
            return;
        }

        // Create timer for publishing
        auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_));
        timer_ = this->create_wall_timer(
            timer_period, 
            std::bind(&MPU6500Publisher::timer_callback, this)
        );

        RCLCPP_INFO(this->get_logger(), "MPU6500 Publisher initialized successfully");
    }

    ~MPU6500Publisher()
    {
        // Deinitialize MPU6500
        if (mpu6500_deinit(&handle_) != 0) {
            RCLCPP_WARN(this->get_logger(), "Failed to deinitialize MPU6500");
        }
        mpu6500_interface_iic_deinit();
    }

private:
    int initialize_mpu6500()
    {
        // Initialize the handle structure to zero first
        memset(&handle_, 0, sizeof(handle_));
        
        // Use the driver macros to link functions properly
        DRIVER_MPU6500_LINK_DEBUG_PRINT(&handle_, mpu6500_interface_debug_print);
        DRIVER_MPU6500_LINK_DELAY_MS(&handle_, mpu6500_interface_delay_ms);
        DRIVER_MPU6500_LINK_RECEIVE_CALLBACK(&handle_, mpu6500_interface_receive_callback);
        
        // Link I2C functions
        DRIVER_MPU6500_LINK_IIC_INIT(&handle_, mpu6500_interface_iic_init);
        DRIVER_MPU6500_LINK_IIC_DEINIT(&handle_, mpu6500_interface_iic_deinit);
        DRIVER_MPU6500_LINK_IIC_READ(&handle_, mpu6500_interface_iic_read);
        DRIVER_MPU6500_LINK_IIC_WRITE(&handle_, mpu6500_interface_iic_write);
        
        // Link SPI dummy functions
        DRIVER_MPU6500_LINK_SPI_INIT(&handle_, mpu6500_interface_spi_init);
        DRIVER_MPU6500_LINK_SPI_DEINIT(&handle_, mpu6500_interface_spi_deinit);
        DRIVER_MPU6500_LINK_SPI_READ(&handle_, mpu6500_interface_spi_read);
        DRIVER_MPU6500_LINK_SPI_WRITE(&handle_, mpu6500_interface_spi_write);

        // Initialize the MPU6500
        RCLCPP_INFO(this->get_logger(), "Initializing MPU6500...");
        uint8_t result = mpu6500_init(&handle_);
        if (result != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize MPU6500 sensor, error code: %d", result);
            return -1;
        }

        // Set the interface to I2C
        RCLCPP_INFO(this->get_logger(), "Setting interface to I2C...");
        result = mpu6500_set_interface(&handle_, MPU6500_INTERFACE_IIC);
        if (result != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set MPU6500 interface to I2C, error code: %d", result);
            return -1;
        }

        // Set the address (0x68 corresponds to MPU6500_ADDRESS_AD0_LOW)
        RCLCPP_INFO(this->get_logger(), "Setting I2C address to 0x68...");
        result = mpu6500_set_addr_pin(&handle_, MPU6500_ADDRESS_AD0_LOW);
        if (result != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set MPU6500 address, error code: %d", result);
            return -1;
        }

        // Disable sleep mode first
        RCLCPP_INFO(this->get_logger(), "Disabling sleep mode...");
        result = mpu6500_set_sleep(&handle_, MPU6500_BOOL_FALSE);
        if (result != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to disable sleep mode, error code: %d", result);
            return -1;
        }

        // Set clock source
        RCLCPP_INFO(this->get_logger(), "Setting clock source...");
        result = mpu6500_set_clock(&handle_, MPU6500_CLOCK_PLL_GYRO_X_REFERENCE);
        if (result != 0) {
            RCLCPP_WARN(this->get_logger(), "Failed to set clock source, error code: %d", result);
        }

        // Set accelerometer range (±2g)
        RCLCPP_INFO(this->get_logger(), "Setting accelerometer range to ±2g...");
        result = mpu6500_set_accelerometer_range(&handle_, MPU6500_ACCELEROMETER_RANGE_2G);
        if (result != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set accelerometer range, error code: %d", result);
            return -1;
        }

        // Set gyroscope range (±2000dps)
        RCLCPP_INFO(this->get_logger(), "Setting gyroscope range to ±2000dps...");
        result = mpu6500_set_gyroscope_range(&handle_, MPU6500_GYROSCOPE_RANGE_2000DPS);
        if (result != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set gyroscope range, error code: %d", result);
            return -1;
        }

        RCLCPP_INFO(this->get_logger(), "MPU6500 initialized successfully on I2C bus with address 0x68");
        return 0;
    }

    void timer_callback()
    {
        int16_t accel_raw_data[3];
        float accel_g_data[3];
        int16_t gyro_raw_data[3];
        float gyro_dps_data[3];
        uint16_t temp_raw;

        // Read IMU data using the correct function signature
        uint8_t result = mpu6500_read(&handle_, 
                                     &accel_raw_data, &accel_g_data,
                                     &gyro_raw_data, &gyro_dps_data,
                                     &temp_raw);
        
        if (result != 0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                                "Failed to read IMU data, error: %d", result);
            return;
        }

        // Read temperature separately
        int16_t temp_raw_int16;
        float temperature;
        if (mpu6500_read_temperature(&handle_, &temp_raw_int16, &temperature) != 0) {
            temperature = 0.0; // Default value
        }

        // Get current timestamp
        auto now = this->get_clock()->now();

        // Publish IMU message using the converted data
        publish_imu_data(accel_g_data, gyro_dps_data, temperature, now);
    }

    void publish_imu_data(float accel[3], float gyro[3], float temp, rclcpp::Time timestamp)
    {
        auto imu_msg = sensor_msgs::msg::Imu();
        
        imu_msg.header.frame_id = frame_id_;
        imu_msg.header.stamp = timestamp;

        // Convert acceleration from g to m/s²
        imu_msg.linear_acceleration.x = accel[0] * 9.80665;
        imu_msg.linear_acceleration.y = accel[1] * 9.80665;
        imu_msg.linear_acceleration.z = accel[2] * 9.80665;

        // Convert gyroscope from dps to rad/s
        imu_msg.angular_velocity.x = gyro[0] * M_PI / 180.0;
        imu_msg.angular_velocity.y = gyro[1] * M_PI / 180.0;
        imu_msg.angular_velocity.z = gyro[2] * M_PI / 180.0;

        // Set orientation covariance to unknown (we don't have orientation data)
        imu_msg.orientation_covariance[0] = -1;

        // Set covariance matrices
        for (int i = 0; i < 9; i++) {
            imu_msg.linear_acceleration_covariance[i] = 0.0;
            imu_msg.angular_velocity_covariance[i] = 0.0;
        }
        // Set diagonal elements to reasonable values
        imu_msg.linear_acceleration_covariance[0] = 0.01;
        imu_msg.linear_acceleration_covariance[4] = 0.01;
        imu_msg.linear_acceleration_covariance[8] = 0.01;
        
        imu_msg.angular_velocity_covariance[0] = 0.01;
        imu_msg.angular_velocity_covariance[4] = 0.01;
        imu_msg.angular_velocity_covariance[8] = 0.01;

        imu_publisher_->publish(imu_msg);

        // Also publish a dummy magnetometer message (MPU6500 doesn't have magnetometer)
        publish_dummy_mag_data(timestamp);

        // Log temperature occasionally for debugging
        static int counter = 0;
        if (++counter % 100 == 0) {
            RCLCPP_INFO(this->get_logger(), "IMU Temperature: %.2f°C, Accel: [%.3f, %.3f, %.3f] g", 
                       temp, accel[0], accel[1], accel[2]);
        }
    }

    void publish_dummy_mag_data(rclcpp::Time timestamp)
    {
        auto mag_msg = sensor_msgs::msg::MagneticField();
        
        mag_msg.header.frame_id = frame_id_;
        mag_msg.header.stamp = timestamp;

        // Set dummy magnetometer data (pointing north)
        mag_msg.magnetic_field.x = 0.0;
        mag_msg.magnetic_field.y = 2.5e-5;  // Approximate Earth's magnetic field
        mag_msg.magnetic_field.z = 0.0;

        // Set covariance
        for (int i = 0; i < 9; i++) {
            mag_msg.magnetic_field_covariance[i] = 0.0;
        }
        mag_msg.magnetic_field_covariance[0] = 1e-6;
        mag_msg.magnetic_field_covariance[4] = 1e-6;
        mag_msg.magnetic_field_covariance[8] = 1e-6;

        mag_publisher_->publish(mag_msg);
    }

    // ROS 2 components
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_publisher_;
    
    // MPU6500 handle
    mpu6500_handle_t handle_;
    
    // Parameters
    std::string frame_id_;
    std::string topic_imu_;
    std::string topic_mag_;
    double publish_rate_;
    int i2c_address_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<MPU6500Publisher>();
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}