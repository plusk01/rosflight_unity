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
#include <list>
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
  private:
    static constexpr int SIMCOM_MSG_TYPE_LEN = 1;
    enum class SimComMsg : uint8_t {
      SIMCONFIG = 0x00,
      VEHCONFIG = 0x01,
      IMU = 0x02,
      MOTORCMD = 0x03
    };

    // largest expected packet size
    static constexpr int MAX_PKT_LEN = 100;
    struct Buffer
    {
      uint8_t data[MAX_PKT_LEN];
      size_t len;
      size_t pos;

      Buffer() : len(0), pos(0) {}

      Buffer(const uint8_t *src, size_t length) : len(length), pos(0)
      {
        assert(length <= MAX_PKT_LEN); //! \todo Do something less catastrophic here
        memcpy(data, src, length);
      }

      const uint8_t * dpos() const { return data + pos; }
      size_t nbytes() const { return len - pos; }
      void add_byte(uint8_t byte) { data[len++] = byte; }
      uint8_t consume_byte() { return data[pos++]; }
      bool empty() const { return pos >= len; }
      bool full() const { return len >= MAX_PKT_LEN; }
    };

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
              std::string remoteHost = "127.0.0.1", uint16_t remotePort = 2909);

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

    /**
     * @brief      Configure the Unity simulation
     */
    void doConfigSim();

    /**
     * @brief      Configure physical parameters of simulation vehicle
     */
    void doConfigVehicle();

    /**
     * @brief      Send motor commands to simulation
     *
     * @param      motors     The motor values
     * @param[in]  numMotors  The number of motors
     */
    void doMotorCmd(float const * motors, size_t numMotors);

  private:
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
    // std::list<Buffer*> read_queue_;

    std::list<Buffer*> write_queue_;
    bool write_in_progress_ = false;

    void async_read();
    void async_read_end(const boost::system::error_code &error, size_t bytes_transferred);

    void async_write(bool check_write_state);
    void async_write_end(const boost::system::error_code &error, size_t bytes_transferred);
    void write(uint8_t const * buf, size_t len);

    //
    // Parsers
    //

    /**
     * @brief      Parse IMU message from Unity-side (SimCom)
     *
     * @param[in]  buf    The buffer of data to parse
     * @param[in]  len    The length of the buffer
     * @param      accel  Parsed accel data
     * @param      gyro   Parsed gyro data
     */
    void parse_imu_msg(const uint8_t* buf, size_t len, float accel[3], float gyro[3]);

    //
    // Packers
    //

    /**
     * @brief      Pack a SimConfig Message
     *
     * @param      buf   The buffer to fill with message data
     *
     * @return     Length of message
     */
    size_t pack_simconfig_msg(uint8_t * buf);

    /**
     * @brief      Pack a VehConfig Message
     *
     * @param      buf   The buffer to fill with message data
     *
     * @return     Length of message
     */
    size_t pack_vehconfig_msg(uint8_t * buf);

    /**
     * @brief      Pack a MotorCmd Message
     *
     * @param      motors     Motor values
     * @param[in]  numMotors  The number of motors
     * @param      buf        The buffer to fill with message data
     *
     * @return     Length of message
     */
    size_t pack_motorcmd_msg(float const * motors, size_t numMotors, uint8_t * buf);
  };

} // ns rosflight_unity
