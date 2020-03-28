/**
 * Structure containing useful data related to the robot :
 *    - Rotation/Translation data
 *    - Setpoints
 *    - Tolerances
 *    - Caps
 *    - Movement/blocking statuses
 */

#ifndef MOTION_ROBOTSTATUS_HPP_
#define MOTION_ROBOTSTATUS_HPP_

#include "UTILITY/Average.hpp"

struct RobotStatus {

  // Translation data
  volatile int32_t translation_total;
  volatile int32_t translation_last;
  volatile int32_t translation_speed;
  volatile int32_t translation_setpoint;
  volatile int32_t translation_speed_setpoint;

  // Rotation data
  volatile int32_t rotation_total;
  volatile int32_t rotation_last;
  volatile int32_t rotation_speed;
  volatile int32_t rotation_setpoint;
  volatile int32_t rotation_speed_setpoint;

  // Tolerances
  int32_t translation_tolerance;
  int32_t rotation_tolerance;
  int32_t derivative_tolerance;
  int32_t differential_tolerance;
  int32_t pwm_tolerance;

  // Caps
  int32_t accel_max;
  int32_t speed_max_translation;
  int32_t speed_max_rotation;
  int32_t speed_max_wheel;

  // Statuses
  volatile bool blocked;
  volatile bool moving;
  volatile bool forced_movement;
  volatile bool movement_stopped;

  //Control modes
  bool controlled_speed;
  bool controlled_rotation;
  bool controlled_position;

  // Movement done detection
  volatile bool movement_done;


  RobotStatus(const uint16_t update_freq) : C_UPDATE_FREQ(update_freq){
    translation_total = 0;
    translation_last = 0;
    translation_speed = 0;
    translation_setpoint = 0;
    translation_speed_setpoint = 0;
    translation_tolerance = 0;

    rotation_total = 0;
    rotation_last = 0;
    rotation_speed = 0;
    rotation_setpoint = 0;
    rotation_speed_setpoint = 0;
    rotation_tolerance = 0;

    translation_tolerance = 0;
    rotation_tolerance = 0;
    derivative_tolerance = 0;
    differential_tolerance = 0;
    pwm_tolerance = 0;

    blocked = false;
    moving = false;
    forced_movement = false;
    movement_stopped = false;
    movement_done = false;
    count_done = 0;

    controlled_speed = false;
    controlled_position = false;
    controlled_rotation = false;
}

  void update_position(int32_t left_ticks, int32_t right_ticks){
    translation_total = (left_ticks + right_ticks) >> 1;
    rotation_total = ((right_ticks - translation_total) - (left_ticks - translation_total)) >> 1;

    translation_speed = (translation_total - translation_last)*C_UPDATE_FREQ;
    rotation_speed = (rotation_total - rotation_last)*C_UPDATE_FREQ;

    translation_last = translation_total;
    rotation_last = rotation_total;
  }


  void update_movement_done(int32_t error_translation, int32_t error_rotation){
    if(error_translation < translation_tolerance && error_rotation < rotation_tolerance){
      // Approximately at destination
      if(!movement_done){ // Was not done, init a counter
        movement_done = true;
        count_done = INIT_COUNT;
      }
      else if(count_done > 0){
        count_done--;
      }
    }
    else{
      movement_done = false;
    }
  }


  /**
   * If the robot was close to the target position for long enough
   */
  bool is_movement_done(){
    return movement_done && count_done == 0;
  }

private:
  const uint8_t INIT_COUNT = 20;
  const float LIMIT = 0.3;
  volatile uint8_t count_done;
  const uint16_t C_UPDATE_FREQ;

};

#endif /* MOTION_WHEELSTATUS_HPP_ */
