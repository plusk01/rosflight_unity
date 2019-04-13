/**
 * @file unity_bridge.cpp
 * @brief Communication bridge between Unity and ROSflight SIL
 * @author Parker Lusk <parkerclusk@gmail.com>
 * @date 13 April 2019
 */

#include "rosflight_unity/unity_bridge.h"

namespace rosflight_unity
{

using boost::asio::ip::udp;

UnityBridge::UnityBridge()
: io_service_(), socket_(io_service_)
{}

// ----------------------------------------------------------------------------

UnityBridge::~UnityBridge()
{
  io_service_.stop();
  socket_.close();

  if (io_thread_.joinable())
    io_thread_.join();
}

// ----------------------------------------------------------------------------

void UnityBridge::onPhysicsUpdate(std::function<void(void)> fn)
{
  cbPhysics_ = fn;
}

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

void UnityBridge::init(std::string bindHost, uint16_t bindPort,
                       std::string remoteHost, uint16_t remotePort)
{
  udp::resolver resolver(io_service_);

  bind_endpoint_ = *resolver.resolve({udp::v4(), bindHost, ""});
  bind_endpoint_.port(bindPort);

  remote_endpoint_ = *resolver.resolve({udp::v4(), remoteHost, ""});
  remote_endpoint_.port(remotePort);

  //
  // Socket
  //

  socket_.open(udp::v4());
  socket_.bind(bind_endpoint_);

  socket_.set_option(udp::socket::reuse_address(true));
  socket_.set_option(udp::socket::send_buffer_size(1000*Buffer::MAX_PKT_LEN));
  socket_.set_option(udp::socket::receive_buffer_size(1000*Buffer::MAX_PKT_LEN));

  async_read();
  // io_thread_ = std::thread([&] { io_service_.run(); });
  io_thread_ = boost::thread(boost::bind(&boost::asio::io_service::run, &io_service_));
}

// ----------------------------------------------------------------------------

void UnityBridge::async_read()
{
  if (!socket_.is_open()) return;

  socket_.async_receive_from(boost::asio::buffer(read_buffer_, Buffer::MAX_PKT_LEN),
                             remote_endpoint_,
                             boost::bind(&UnityBridge::async_read_end,
                                         this,
                                         boost::asio::placeholders::error,
                                         boost::asio::placeholders::bytes_transferred));
}

// ----------------------------------------------------------------------------

void UnityBridge::async_read_end(const boost::system::error_code &error, size_t bytes_transferred)
{
  if (!error) {
    // std::cout << "Recvd " << bytes_transferred << " bytes." << std::endl;

    uint8_t *ptr = read_buffer_;


    double timestamp;
    memcpy(&timestamp, ptr, sizeof(double)); ptr += sizeof(double);

    float accel_x, accel_y, accel_z;
    memcpy(&accel_x, ptr, sizeof(float)); ptr += sizeof(float);
    memcpy(&accel_y, ptr, sizeof(float)); ptr += sizeof(float);
    memcpy(&accel_z, ptr, sizeof(float)); ptr += sizeof(float);

    float gyro_x, gyro_y, gyro_z;
    memcpy(&gyro_x, ptr, sizeof(float)); ptr += sizeof(float);
    memcpy(&gyro_y, ptr, sizeof(float)); ptr += sizeof(float);
    memcpy(&gyro_z, ptr, sizeof(float)); ptr += sizeof(float);

    newImuData_ = true;

    // notify the delegate
    if (cbPhysics_) cbPhysics_();

  }
  async_read();
}

// ----------------------------------------------------------------------------

} // ns rosflight_unity
