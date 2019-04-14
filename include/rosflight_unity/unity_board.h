/**
 * @file unity_board.h
 * @brief ROSflight board specialization for Unity SIL
 * @author Parker Lusk <parkerclusk@gmail.com>
 * @date 13 April 2019
 */

#pragma once

#include <iostream>
#include <fstream>
#include <string>
#include <algorithm>

#include <rosflight_firmware/udp_board.h>

#include "rosflight_unity/unity_bridge.h"

namespace rosflight_unity
{

  class UnityBoard : public rosflight_firmware::UDPBoard
  {
  public:
    UnityBoard(UnityBridge& unity);
    ~UnityBoard() = default;
    
    /**
     * @brief      Sets the vehicle name.
     *
     * @param[in]  name  Vehicle name
     */
    void setVehicleName(std::string name);

    /**
     * @brief      Sets the time based on an external clock
     *
     * @param[in]  secs   The seconds only
     * @param[in]  nsecs  The remainder in nanoseconds
     */
    void setTime(uint32_t secs, uint64_t nsecs);

    //
    // ROSflight overrides
    //

    // setup
    void init_board() override {}
    void board_reset(bool bootloader) override {}

    // clock
    uint32_t clock_millis() override;
    uint64_t clock_micros() override;
    void clock_delay(uint32_t milliseconds) override {}

    // sensors
    void sensors_init() override {}
    uint16_t num_sensor_errors() override { return 0; }

    bool new_imu_data() override;
    bool imu_read(float accel[3], float* temperature, float gyro[3], uint64_t* time_us) override;
    void imu_not_responding_error() override {}

    bool mag_check() override { return false; }
    void mag_read(float mag[3]) override {}

    bool baro_check() override { return false; }
    void baro_read(float *pressure, float *temperature) override {}

    bool diff_pressure_check() override { return false; }
    void diff_pressure_read(float *diff_pressure, float *temperature) override {}

    bool sonar_check() override { return false; }
    float sonar_read() override { return 0.0f; }

    // PWM
    // TODO make these deal in normalized (-1 to 1 or 0 to 1) values (not pwm-specific)
    void pwm_init(bool cppm, uint32_t refresh_rate, uint16_t idle_pwm) override;
    bool pwm_lost() override;
    uint16_t pwm_read(uint8_t channel) override;
    void pwm_write(uint8_t channel, uint16_t value) override;

    // non-volatile memory
    void memory_init() override {}
    bool memory_read(void * dest, size_t len) override;
    bool memory_write(const void * src, size_t len) override;

    // LEDs
    void led0_on() override {}
    void led0_off() override {}
    void led0_toggle() override {}

    void led1_on() override {}
    void led1_off() override {}
    void led1_toggle() override {}

  private:
    std::string vehicleName_ = "default"; ///< name of associated simulated vehicle

    double time_init_ = 0; ///< initial time from external source
    double time_ = 0; ///< time calculated from external source

    UnityBridge& unity_; ///< ref to an instantiated unity bridge

  };

} // ns rosflight_unity
