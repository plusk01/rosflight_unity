/**
 * @file unity_board.cpp
 * @brief ROSflight board specialization for Unity SIL
 * @author Parker Lusk <parkerclusk@gmail.com>
 * @date 13 April 2019
 */

#include "rosflight_unity/unity_board.h"

namespace rosflight_unity
{

UnityBoard::UnityBoard(UnityBridge& unity)
: rosflight_firmware::UDPBoard(),
  unity_(unity)
{}

// ----------------------------------------------------------------------------

void UnityBoard::setVehicleName(std::string name)
{
  // clean name---only keep alphanumeric chars
  name.erase(std::remove_if(name.begin(), name.end(),
        [](auto const& c) -> bool { return !std::isalnum(c); }
      ), name.end());

  // Leave as default if there is nothing left over
  if (!name.empty()) vehicleName_ = name;
}

// ----------------------------------------------------------------------------

void UnityBoard::setTime(uint32_t secs, uint64_t nsecs)
{
  double t = secs + nsecs*1e-9;

  // capture "power on" time so that ROSflight clock is secs from start up
  if (time_init_ == 0) time_init_ = t;

  time_ = t - time_init_;
}

void UnityBoard::setRC(const uint16_t rc[MAX_RC_CHANNELS])
{
  std::memcpy(latestRC_, rc, MAX_RC_CHANNELS*sizeof(uint16_t));
  latestRCTime_ = std::chrono::high_resolution_clock::now();
}

// ----------------------------------------------------------------------------
// Overrides (required by rosflight firmware)
// ----------------------------------------------------------------------------

uint32_t UnityBoard::clock_millis()
{
  return static_cast<uint32_t>(time_ * 1e3);
}

// ----------------------------------------------------------------------------

uint64_t UnityBoard::clock_micros()
{
  return static_cast<uint64_t>(time_ * 1e6);
}

// ----------------------------------------------------------------------------

bool UnityBoard::new_imu_data()
{
  return unity_.hasNewImuData();
}

// ----------------------------------------------------------------------------

bool UnityBoard::imu_read(float accel[3], float* temperature, float gyro[3], uint64_t* time_us)
{
  unity_.getNewImuData(accel, gyro);

  *temperature = 27.0f;
  *time_us = clock_micros();
  return true;
}

// ----------------------------------------------------------------------------

void UnityBoard::pwm_write(uint8_t channel, float value)
{
  unity_.doMotorCmd(channel, value);
}

// ----------------------------------------------------------------------------

void UnityBoard::pwm_disable()
{
  static constexpr int MAX_NUM_MOTORS = 8;
  for (uint8_t i=0; i<MAX_NUM_MOTORS; ++i) {
    pwm_write(i, 0.0f);
  }
}

// ----------------------------------------------------------------------------

void UnityBoard::rc_init(rc_type_t rc_type)
{
  latestRC_[0] = 1500; // A
  latestRC_[1] = 1500; // E
  latestRC_[3] = 1000; // T
  latestRC_[2] = 1500; // R
  latestRC_[4] = 1000; // attitude override
  latestRC_[5] = 1000; // arm
  latestRC_[6] = 1000;
  latestRC_[7] = 1000;
}

// ----------------------------------------------------------------------------

float UnityBoard::rc_read(uint8_t channel)
{
  // Map pwm [1000, 2000] usec to normalized [0, 1]
  return static_cast<float>(latestRC_[channel] - 1000) / 1000.0;
}

// ----------------------------------------------------------------------------

bool UnityBoard::rc_lost()
{
  constexpr double thresh = 200; // [ms]
  auto now = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::milli> elapsed = now - latestRCTime_;
  return (elapsed.count() > thresh);
}

// ----------------------------------------------------------------------------

bool UnityBoard::memory_read(void *dest, size_t len)
{
  std::string file = "rosflight_" + vehicleName_ + ".bin";
  std::ifstream memory_file;
  memory_file.open(file, std::ios::binary);

  if (!memory_file.is_open()) return false;

  memory_file.read(static_cast<char *>(dest), len);
  memory_file.close();
  return true;
}

// ----------------------------------------------------------------------------

bool UnityBoard::memory_write(const void *src, size_t len)
{
  std::string file = "rosflight_" + vehicleName_ + ".bin";
  std::ofstream memory_file;
  memory_file.open(file, std::ios::binary);

  if (!memory_file.is_open()) return false;

  memory_file.write((char *)src, len);
  memory_file.close();
  return true;
}

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

} // ns rosflight_unity
