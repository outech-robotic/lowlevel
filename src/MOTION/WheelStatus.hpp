/**
 * Structure containing useful data related to a wheel :
 *    - Positions of its encoder, speed
 *    - Setpoints
 *    - PWM applied to it
 */

#ifndef MOTION_WHEELSTATUS_HPP_
#define MOTION_WHEELSTATUS_HPP_

#include "UTILITY/Average.hpp"

template<bool IS_32BITS_COUNTER>
struct WheelStatus {

  volatile int32_t ticks_current;
  volatile int32_t speed_current;
  volatile int32_t speed_average;
  volatile int32_t speed_setpoint;
  volatile int32_t speed_setpoint_wanted;
  volatile int16_t pwm;


  WheelStatus(const uint16_t update_freq) : C_UPDATE_FREQ(update_freq){
    ticks_current = 0;
    ticks_last = 0;
    speed_current = 0;
    speed_average = 0;
    speed_setpoint = 0;
    speed_setpoint_wanted = 0;
    speed_setpoint_last = 0;
    overflows = 0;
    ticks_raw_last = 0;
    pwm = 0;
  }

  /**
   * Updates the current position and speed of the wheel.
   */
  void update(int32_t raw_ticks){
    // Ticks update
    if(IS_32BITS_COUNTER){
      ticks_current = raw_ticks;
    }
    else{
      // Manage over/under-flows
      // Underflow : went from a low value to a high value
      if(raw_ticks - ticks_raw_last > 32767){
        overflows--;
      }
      // Overflow : went from a low value to a high value
      else if(ticks_raw_last - raw_ticks > 32767){
        overflows++;
      }

      ticks_raw_last = raw_ticks;
      ticks_current = (int32_t)raw_ticks + (int32_t)overflows*(int32_t)65536;
    }

    // Speed update
    speed_current = (ticks_current - ticks_last)*C_UPDATE_FREQ;
    ticks_last = ticks_current;
    speed_averager.add(speed_current);
    speed_average = speed_averager.value();
  }


  void cap_speed(const int32_t speed_max){
    if(speed_setpoint > speed_max){
      speed_setpoint = speed_max;
    }
    else if(speed_setpoint < -speed_max){
      speed_setpoint = -speed_max;
    }
  }


  void cap_accel(const int32_t accel_max){
    if(speed_setpoint - speed_setpoint_last > accel_max){
      speed_setpoint = speed_setpoint_last + accel_max;
    }
    else if(speed_setpoint_last - speed_setpoint > accel_max){
      speed_setpoint = speed_setpoint_last - accel_max;
    }
  }

private:

  Average<volatile int32_t, 8> speed_averager;

  volatile int16_t overflows;
  volatile int32_t ticks_last;
  volatile int32_t speed_setpoint_last;
  volatile int16_t ticks_raw_last;
  const uint16_t C_UPDATE_FREQ;

};

#endif /* MOTION_WHEELSTATUS_HPP_ */
