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
  MutexLock read_lock(read_mutex_);
  MutexLock write_lock(write_mutex_);

  io_service_.stop();
  socket_.close();

  if (io_thread_.joinable())
    io_thread_.join();
}

// ----------------------------------------------------------------------------

void UnityBridge::onPhysicsUpdate(std::function<void(int32_t,int32_t)> fn)
{
  cbPhysics_ = fn;
}

// ----------------------------------------------------------------------------

void UnityBridge::getNewImuData(float accel[3], float gyro[3])
{
  MutexLock lock(read_mutex_);

  // Unity (left-handed) to FRD (right-handed)
  accel[0] =  accel_[0];
  accel[1] = -accel_[2];
  accel[2] = -accel_[1];

  // Unity (left-handed) to FRD (right-handed)
  gyro[0] = -gyro_[0];
  gyro[1] =  gyro_[2];
  gyro[2] =  gyro_[1];

  // reset new data flag
  newImuData_ = false;
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
  socket_.set_option(udp::socket::send_buffer_size(1000*MAX_PKT_LEN));
  socket_.set_option(udp::socket::receive_buffer_size(1000*MAX_PKT_LEN));

  async_read();
  io_thread_ = boost::thread(boost::bind(&boost::asio::io_service::run, &io_service_));
}

// ----------------------------------------------------------------------------

void UnityBridge::async_read()
{
  if (!socket_.is_open()) return;

  MutexLock lock(read_mutex_);
  socket_.async_receive_from(boost::asio::buffer(read_buffer_, MAX_PKT_LEN),
                             remote_endpoint_,
                             boost::bind(&UnityBridge::async_read_end,
                                         this,
                                         boost::asio::placeholders::error,
                                         boost::asio::placeholders::bytes_transferred));
}

// ----------------------------------------------------------------------------

void UnityBridge::async_read_end(const boost::system::error_code &error,
                                 size_t bytes_transferred)
{
  if (!error) {
    MutexLock lock(read_mutex_);
    parse_imu_msg(read_buffer_, bytes_transferred, accel_, gyro_);
  }

  async_read();
}

// ----------------------------------------------------------------------------

void UnityBridge::parse_imu_msg(uint8_t const * buf, size_t len,
                                float accel[3], float gyro[3])
{
  int32_t timestamp_secs, timestamp_nsecs;
  memcpy(&timestamp_secs, buf, sizeof(int32_t)); buf += sizeof(int32_t);
  memcpy(&timestamp_nsecs, buf, sizeof(int32_t)); buf += sizeof(int32_t);

  // accelerometer measurement
  memcpy(&accel[0], buf, sizeof(float)); buf += sizeof(float);
  memcpy(&accel[1], buf, sizeof(float)); buf += sizeof(float);
  memcpy(&accel[2], buf, sizeof(float)); buf += sizeof(float);

  // gyro measurement
  memcpy(&gyro[0], buf, sizeof(float)); buf += sizeof(float);
  memcpy(&gyro[1], buf, sizeof(float)); buf += sizeof(float);
  memcpy(&gyro[2], buf, sizeof(float)); buf += sizeof(float);

  // indicate that we have new data to read
  newImuData_ = true;

  // Notify the delegate.
  // NOTE: The assumption here is that we receive an IMU message on every
  // physics timestep in Unity. This corresponds to an IMU message being
  // sent on every FixedUpdate() call.
  if (cbPhysics_) cbPhysics_(timestamp_secs, timestamp_nsecs);
}

// ----------------------------------------------------------------------------

} // ns rosflight_unity
