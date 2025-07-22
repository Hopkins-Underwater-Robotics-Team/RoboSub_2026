#include <memory>
#include <chrono>
#include <cmath>
#include <cstring>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"

// Include your MPU6500 driver headers
extern "C" {
    #include "driver_mpu6500.h"
    #include "driver_mpu6500_interface.h"
}

using namespace std::chrono_literals;

// Global callback functions (must be extern "C" for C linkage)
extern "C" {
    // Dummy SPI functions
    uint8_t mpu6500_interface_spi_init(void) { return 0; }
    uint8_t mpu6500_interface_spi_deinit(void) { return 0; }
    uint8_t mpu6500_interface_spi_read(uint8_t reg, uint8_t *buf, uint16_t len) { 
        (void)reg; (void)buf; (void)len;
        return 1; 
    }
    uint8_t mpu6500_interface_spi_write(uint8_t reg, uint8_t *buf, uint16_t len) { 
        (void)reg; (void)buf; (void)len;
        return 1; 
    }
    
    // Required callback functions
    void receive_callback_impl(uint8_t type) {
        printf("MPU6500: Received interrupt type: %d\n", type);
        (void)type;
    }
    
    void dmp_tap_callback_impl(uint8_t count, uint8_t direction) {
        printf("MPU6500: DMP tap detected - count: %d, direction: %d\n", count, direction);
        (void)count; (void)direction;
    }
    
    void dmp_orient_callback_impl(uint8_t orientation) {
        printf("MPU6500: DMP orientation changed: %d\n", orientation);
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
        if (mpu6500_deinit(&handle_) != 0) {
            RCLCPP_WARN(this->get_logger(), "Failed to deinitialize MPU6500");
        }
        mpu6500_interface_iic_deinit();
    }

private:
    int initialize_mpu6500()
    {
        RCLCPP_INFO(this->get_logger(), "Initializing MPU6500...");
        
        // Zero out the handle first
        memset(&handle_, 0, sizeof(handle_));
        
        // Set up all function pointers
        handle_.debug_print = mpu6500_interface_debug_print;
        handle_.delay_ms = mpu6500_interface_delay_ms;
        handle_.receive_callback = receive_callback_impl;
        handle_.dmp_tap_callback = dmp_tap_callback_impl;
        handle_.dmp_orient_callback = dmp_orient_callback_impl;
        
        // I2C functions
        handle_.iic_init = mpu6500_interface_iic_init;
        handle_.iic_deinit = mpu6500_interface_iic_deinit;
        handle_.iic_read = mpu6500_interface_iic_read;
        handle_.iic_write = mpu6500_interface_iic_write;
        
        // SPI functions
        handle_.spi_init = mpu6500_interface_spi_init;
        handle_.spi_deinit = mpu6500_interface_spi_deinit;
        handle_.spi_read = mpu6500_interface_spi_read;
        handle_.spi_write = mpu6500_interface_spi_write;

        // Set the interface to I2C BEFORE calling init
        RCLCPP_INFO(this->get_logger(), "Setting interface to I2C...");
        uint8_t result = mpu6500_set_interface(&handle_, MPU6500_INTERFACE_IIC);
        if (result != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set I2C interface, error: %d", result);
            return -1;
        }

        // Set the address BEFORE calling init  
        RCLCPP_INFO(this->get_logger(), "Setting I2C address to 0x68...");
        result = mpu6500_set_addr_pin(&handle_, MPU6500_ADDRESS_AD0_LOW);
        if (result != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set address, error: %d", result);
            return -1;
        }

        // NOW call mpu6500_init - it should have the correct address
        RCLCPP_INFO(this->get_logger(), "Calling mpu6500_init...");
        result = mpu6500_init(&handle_);
        if (result != 0) {
            RCLCPP_ERROR(this->get_logger(), "mpu6500_init failed with error code: %d", result);
            return -1;
        }

        // Wake up the device
        RCLCPP_INFO(this->get_logger(), "Waking up device...");
        result = mpu6500_set_sleep(&handle_, MPU6500_BOOL_FALSE);
        if (result != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to wake up device, error: %d", result);
            return -1;
        }

        // Set ranges
        RCLCPP_INFO(this->get_logger(), "Setting accelerometer range...");
        result = mpu6500_set_accelerometer_range(&handle_, MPU6500_ACCELEROMETER_RANGE_2G);
        if (result != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set accel range, error: %d", result);
            return -1;
        }

        RCLCPP_INFO(this->get_logger(), "Setting gyroscope range...");
        result = mpu6500_set_gyroscope_range(&handle_, MPU6500_GYROSCOPE_RANGE_2000DPS);
        if (result != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set gyro range, error: %d", result);
            return -1;
        }

        RCLCPP_INFO(this->get_logger(), "MPU6500 initialized successfully!");
        return 0;
    }

    void timer_callback()
    {
        int16_t accel_raw_data[3];
        float accel_g_data[3];
        int16_t gyro_raw_data[3];
        float gyro_dps_data[3];
        uint16_t temp_raw;

        uint8_t result = mpu6500_read(&handle_, 
                                     &accel_raw_data, &accel_g_data,
                                     &gyro_raw_data, &gyro_dps_data,
                                     &temp_raw);
        
        if (result != 0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                                "Failed to read IMU data, error: %d", result);
            return;
        }

        int16_t temp_raw_int16;
        float temperature;
        if (mpu6500_read_temperature(&handle_, &temp_raw_int16, &temperature) != 0) {
            temperature = 0.0;
        }

        auto now = this->get_clock()->now();
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

        // Set orientation covariance to unknown
        imu_msg.orientation_covariance[0] = -1;

        // Set covariance matrices
        for (int i = 0; i < 9; i++) {
            imu_msg.linear_acceleration_covariance[i] = 0.0;
            imu_msg.angular_velocity_covariance[i] = 0.0;
        }
        imu_msg.linear_acceleration_covariance[0] = 0.01;
        imu_msg.linear_acceleration_covariance[4] = 0.01;
        imu_msg.linear_acceleration_covariance[8] = 0.01;
        
        imu_msg.angular_velocity_covariance[0] = 0.01;
        imu_msg.angular_velocity_covariance[4] = 0.01;
        imu_msg.angular_velocity_covariance[8] = 0.01;

        imu_publisher_->publish(imu_msg);

        static int counter = 0;
        if (++counter % 100 == 0) {
            RCLCPP_INFO(this->get_logger(), "IMU Temperature: %.2f°C, Accel: [%.3f, %.3f, %.3f] g", 
                       temp, accel[0], accel[1], accel[2]);
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_publisher_;
    
    mpu6500_handle_t handle_;
    
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