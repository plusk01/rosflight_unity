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

UnityBridge::TruthMsg UnityBridge::getTruthData()
{
  MutexLock lock(read_mutex_);

  TruthMsg truth;

  truth.timestamp_secs = truth_.timestamp_secs;
  truth.timestamp_nsecs = truth_.timestamp_nsecs;

  // Unity (left-handed) to NED (right-handed)
  truth.x[0] =  truth_.x[0];
  truth.x[1] = -truth_.x[2];
  truth.x[2] = -truth_.x[1];

  // Unity (left-handed) to NED (right-handed)
  truth.v[0] =  truth_.v[0];
  truth.v[1] = -truth_.v[2];
  truth.v[2] = -truth_.v[1];

  // Unity (left-handed) to NED (right-handed)
  // see: https://gamedev.stackexchange.com/a/157954
  truth.q[0] =  truth_.q[0];
  truth.q[1] = -truth_.q[1];
  truth.q[2] =  truth_.q[3];
  truth.q[3] =  truth_.q[2];

  return truth;
}

// ----------------------------------------------------------------------------

void UnityBridge::doConfigSim()
{
  uint8_t bytes[MAX_PKT_LEN];
  size_t len = pack_simconfig_msg(bytes);
  write(bytes, len);
}

// ----------------------------------------------------------------------------

void UnityBridge::doConfigVehicle()
{
  uint8_t bytes[MAX_PKT_LEN];
  size_t len = pack_vehconfig_msg(bytes);
  write(bytes, len);
}

// ----------------------------------------------------------------------------

void UnityBridge::doMotorCmd(uint8_t channel, float value)
{
  uint8_t bytes[MAX_PKT_LEN];
  size_t len = pack_motorcmd_msg(channel, value, bytes);
  write(bytes, len);
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

void UnityBridge::async_read_end(const boost::system::error_code& error,
                                 size_t bytes_transferred)
{
  if (!error) {
    MutexLock lock(read_mutex_);

    // Extract just the message data (i.e., seek past bytes indicated msg type)
    uint8_t * data = &read_buffer_[SIMCOM_MSG_TYPE_LEN];

    switch (read_buffer_[0]) {
      case static_cast<uint8_t>(SimComMsg::IMU):
        parse_imu_msg(data, bytes_transferred, accel_, gyro_);
        break;

      case static_cast<uint8_t>(SimComMsg::TRUTH):
        parse_truth_msg(data, bytes_transferred, truth_);
        break;

      default: // unrecognized msg type
        break;
    }

  }

  async_read();
}

// ----------------------------------------------------------------------------

void UnityBridge::async_write(bool check_write_state)
{
  if (check_write_state && write_in_progress_) return;

  MutexLock lock(write_mutex_);
  if (write_queue_.empty()) return;

  write_in_progress_ = true;
  Buffer * buffer = write_queue_.front();
  socket_.async_send_to(boost::asio::buffer(buffer->dpos(), buffer->nbytes()),
                        remote_endpoint_,
                        boost::bind(&UnityBridge::async_write_end,
                                    this,
                                    boost::asio::placeholders::error,
                                    boost::asio::placeholders::bytes_transferred));
}

// ----------------------------------------------------------------------------

void UnityBridge::async_write_end(const boost::system::error_code& error,
                                  size_t bytes_transferred)
{
  if (!error) {
    MutexLock lock(write_mutex_);

    Buffer * buffer = write_queue_.front();
    buffer->pos += bytes_transferred;
    if (buffer->empty()) {
      write_queue_.pop_front();
      delete buffer;
    }

    if (write_queue_.empty())
      write_in_progress_ = false;
    else
      async_write(false);
  }
}

// ----------------------------------------------------------------------------

void UnityBridge::write(uint8_t const * buf, size_t len)
{
  Buffer * buffer = new Buffer(buf, len);

  {
    MutexLock lock(write_mutex_);
    write_queue_.push_back(buffer);
  }

  async_write(true);
}

// ----------------------------------------------------------------------------
// Parsers for Incoming Messages
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

void UnityBridge::parse_truth_msg(uint8_t const * buf, size_t len, TruthMsg& truth)
{
  memcpy(&truth.timestamp_secs, buf, sizeof(int32_t)); buf += sizeof(int32_t);
  memcpy(&truth.timestamp_nsecs, buf, sizeof(int32_t)); buf += sizeof(int32_t);

  // position
  memcpy(&truth.x[0], buf, sizeof(float)); buf += sizeof(float);
  memcpy(&truth.x[1], buf, sizeof(float)); buf += sizeof(float);
  memcpy(&truth.x[2], buf, sizeof(float)); buf += sizeof(float);

  // velocity
  memcpy(&truth.v[0], buf, sizeof(float)); buf += sizeof(float);
  memcpy(&truth.v[1], buf, sizeof(float)); buf += sizeof(float);
  memcpy(&truth.v[2], buf, sizeof(float)); buf += sizeof(float);

  // quaternion (w, x, y, z)
  memcpy(&truth.q[0], buf, sizeof(float)); buf += sizeof(float);
  memcpy(&truth.q[1], buf, sizeof(float)); buf += sizeof(float);
  memcpy(&truth.q[2], buf, sizeof(float)); buf += sizeof(float);
  memcpy(&truth.q[3], buf, sizeof(float)); buf += sizeof(float);
}

// ----------------------------------------------------------------------------
// Packers for Outgoing Messages
// ----------------------------------------------------------------------------

size_t UnityBridge::pack_simconfig_msg(uint8_t * buf)
{
  size_t len = 0;

  // Set msg id
  buf[0] = static_cast<uint8_t>(SimComMsg::SIMCONFIG); len += 1;

  return len;
}

// ----------------------------------------------------------------------------

size_t UnityBridge::pack_vehconfig_msg(uint8_t * buf)
{
  size_t len = 0;

  // Set msg id
  buf[0] = static_cast<uint8_t>(SimComMsg::VEHCONFIG); len += 1;

  return len;
}

// ----------------------------------------------------------------------------

size_t UnityBridge::pack_motorcmd_msg(uint8_t channel, float value,
                                      uint8_t * buf)
{
  size_t len = 0;

  // Set msg id
  buf[0] = static_cast<uint8_t>(SimComMsg::MOTORCMD); len += 1;

  // which motor does this value belong to?
  buf[1] = channel; len += 1;

  // commanded motor value
  memcpy(&buf[len], &value, sizeof(float)); len += sizeof(float);

  return len;
}

} // ns rosflight_unity
