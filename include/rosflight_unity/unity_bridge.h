/**
 * @file unity_bridge.h
 * @brief Communication bridge between Unity and ROSflight SIL
 * @author Parker Lusk <parkerclusk@gmail.com>
 * @date 13 April 2019
 * 
 * Inspired by rosflight_firmware/UDPBoard.
 */

#pragma once

#include <iostream>
#include <string>
#include <thread>
#include <functional>

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

namespace rosflight_unity
{

  class UnityBridge
  {
  public:
    UnityBridge();
    ~UnityBridge();
    
    /**
     * @brief      Initialize the Unity bridge UDP connection
     *
     * @param[in]  bindHost    The bind host
     * @param[in]  bindPort    The bind port
     * @param[in]  remoteHost  The remote host
     * @param[in]  remotePort  The remote port
     */
    void init(std::string bindHost = "127.0.0.1", uint16_t bindPort = 2908,
              std::string remoteHost = "127.0.0.1", uint16_t remotePort = 2908);

    /**
     * @brief      Register a callback to be run on each Unity physics step
     *
     * @param[in]  fn    The callback with signature
     *                      fn(int32_t secs, int32_t nsecs)
     */
    void onPhysicsUpdate(std::function<void(int32_t,int32_t)> fn);

    /**
     * @brief      Check if there is new IMU data to read
     *
     * @return     True if has new imu data, False otherwise.
     */
    bool hasNewImuData() const { return newImuData_; };

    /**
     * @brief      Read the new IMU data. Coordinate frame is FRD.
     *
     * @param      accel  Accelerometer data (x, y, z)
     * @param      gyro   Gyro data (x, y, z)
     */
    void getNewImuData(float accel[3], float gyro[3]);

  private:
    // largest expected packet size
    static constexpr int MAX_PKT_LEN = 100;

    // registered callback for Unity physics step
    std::function<void(int32_t,int32_t)> cbPhysics_;

    // IMU data
    bool newImuData_ = false;
    float accel_[3], gyro_[3];

    // Use re-entrant mutex so we don't cause a deadlock when
    // async_read_end (which has the lock) calls cbPhysics_,
    // which then runs the firmware, which eventually calls
    // getNewImuData, which also acquires a lock.
    using MutexLock = boost::lock_guard<boost::recursive_mutex>;
    boost::recursive_mutex write_mutex_;
    boost::recursive_mutex read_mutex_;

    boost::asio::io_service io_service_;
    boost::thread io_thread_;

    boost::asio::ip::udp::socket socket_;
    boost::asio::ip::udp::endpoint bind_endpoint_;
    boost::asio::ip::udp::endpoint remote_endpoint_;

    uint8_t read_buffer_[MAX_PKT_LEN];

    void async_read();
    void async_read_end(const boost::system::error_code &error, size_t bytes_transferred);

    /**
     * @brief      Parse IMU message from Unity-side (SimCom)
     *
     * @param[in]  buf    The buffer of data to parse
     * @param[in]  len    The length of the buffer
     * @param      accel  Parsed accel data
     * @param      gyro   Parsed gyro data
     */
    void parse_imu_msg(const uint8_t* buf, size_t len, float accel[3], float gyro[3]);
  };

} // ns rosflight_unity
