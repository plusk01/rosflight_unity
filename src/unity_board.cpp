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
// Overrides (required by rosflight firmware)
// ----------------------------------------------------------------------------

uint32_t UnityBoard::clock_millis()
{

}

// ----------------------------------------------------------------------------

uint64_t UnityBoard::clock_micros()
{

}

// ----------------------------------------------------------------------------

bool UnityBoard::new_imu_data()
{
  return unity_.hasNewImuData();
}

// ----------------------------------------------------------------------------

bool UnityBoard::imu_read(float accel[3], float* temperature, float gyro[3], uint64_t* time_us)
{

}

// ----------------------------------------------------------------------------

void UnityBoard::pwm_init(bool cppm, uint32_t refresh_rate, uint16_t idle_pwm)
{

}

// ----------------------------------------------------------------------------

bool UnityBoard::pwm_lost()
{

}

// ----------------------------------------------------------------------------

uint16_t UnityBoard::pwm_read(uint8_t channel)
{

}

// ----------------------------------------------------------------------------

void UnityBoard::pwm_write(uint8_t channel, uint16_t value)
{

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

}