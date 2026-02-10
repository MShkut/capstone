#ifndef REALSENSE_IMU_H
#define REALSENSE_IMU_H

#include "imu_interface.h"
#include <sensor_msgs/msg/imu.h>

class RealsenseIMU : public IMUInterface
{
private:
    geometry_msgs__msg__Vector3 accel_data_;
    geometry_msgs__msg__Vector3 gyro_data_;
    bool data_received_ = false;

public:
    RealsenseIMU() : IMUInterface() {}

    // Called by micro-ROS subscription callback
    void updateIMUData(const sensor_msgs__msg__Imu * msg)
    {
        accel_data_ = msg->linear_acceleration;
        gyro_data_ = msg->angular_velocity;
        data_received_ = true;
    }

    geometry_msgs__msg__Vector3 readAccelerometer() override
    {
        return accel_data_;
    }

    geometry_msgs__msg__Vector3 readGyroscope() override
    {
        return gyro_data_;
    }

    bool startSensor() override
    {
        // No hardware initialization needed
        // Calibration will wait for first data
        return true;
    }

    bool hasData()
    {
        return data_received_;
    }
};

#endif
