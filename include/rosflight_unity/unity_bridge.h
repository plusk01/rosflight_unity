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
  private:
    struct Buffer
    {
      static constexpr int MAX_PKT_LEN = 100;
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
    
    void init(std::string bindHost = "127.0.0.1", uint16_t bindPort = 2908,
              std::string remoteHost = "127.0.0.1", uint16_t remotePort = 2908);

    void onPhysicsUpdate(std::function<void(int32_t,int32_t)> fn);

    bool hasNewImuData() const { return newImuData_; };

  private:
    std::function<void(int32_t,int32_t)> cbPhysics_;

    bool newImuData_ = false;

    // boost::recursive_mutex write_mutex_;
    // boost::recursive_mutex read_mutex_;

    boost::asio::io_service io_service_;
    boost::thread io_thread_;

    boost::asio::ip::udp::socket socket_;
    boost::asio::ip::udp::endpoint bind_endpoint_;
    boost::asio::ip::udp::endpoint remote_endpoint_;

    uint8_t read_buffer_[Buffer::MAX_PKT_LEN];

    void async_read();
    void async_read_end(const boost::system::error_code &error, size_t bytes_transferred);
  };

} // ns rosflight_unity
