#include "MOTION/MotionController.h"
#include "config.h"
#include "UTILITY/timing.h"

MotionController::MotionController() :
  robot_status(MOTION_CONTROL_FREQ),
  motor_left(Motor::Side::LEFT), motor_right(Motor::Side::RIGHT),
  cod_left(MOTION_CONTROL_FREQ), cod_right(MOTION_CONTROL_FREQ)
{}

#define PI ((float)3.14159265359)
#define WHEEL_DIAMETER ((float)73.6)
#define TICKS_PER_TURN ((float)2400)
#define MM_TO_TICK (TICKS_PER_TURN/(PI*WHEEL_DIAMETER))

#define MAX_SPEED_TRANSLATION_MM (800)
#define MAX_SPEED_ROTATION_MM (400)
#define MAX_SPEED_TRANSLATION_TICK (MAX_SPEED_TRANSLATION_MM*MM_TO_TICK)
#define MAX_SPEED_ROTATION_TICK (MAX_SPEED_ROTATION_MM*MM_TO_TICK)
#define MAX_ACCEL_MM (1200/MOTION_CONTROL_FREQ)
#define MAX_ACCEL_TICK (MAX_ACCEL_MM*MM_TO_TICK)

void MotionController::init() {
  pid_speed_left.reset();
  pid_speed_right.reset();
  pid_translation.reset();
  pid_rotation.reset();
  pid_speed_left.set_coefficients(0.275, 0.24, 0.0002, MOTION_CONTROL_FREQ);
  pid_speed_left.set_output_limit(CONST_PWM_MAX);
  pid_speed_left.set_anti_windup(CONST_PWM_MAX);
  pid_speed_left.set_derivative_limit(CONST_PWM_MAX);
  pid_speed_right.set_coefficients(0.27, 0.22, 0.0002, MOTION_CONTROL_FREQ);
  pid_speed_right.set_output_limit(CONST_PWM_MAX);
  pid_speed_right.set_anti_windup(CONST_PWM_MAX);
  pid_speed_right.set_derivative_limit(CONST_PWM_MAX);

  pid_translation.set_coefficients(3.0, 0.0, 0.2, MOTION_CONTROL_FREQ);
  pid_translation.set_output_limit(MAX_SPEED_TRANSLATION_TICK);
  pid_translation.set_anti_windup(MAX_SPEED_TRANSLATION_TICK);
  pid_translation.set_derivative_limit(MAX_SPEED_TRANSLATION_TICK);
  pid_rotation.set_coefficients(4.9, 0.0, 0.8, MOTION_CONTROL_FREQ);
  pid_rotation.set_output_limit(MAX_SPEED_ROTATION_TICK);
  pid_rotation.set_anti_windup(MAX_SPEED_ROTATION_TICK);
  pid_rotation.set_derivative_limit(MAX_SPEED_ROTATION_TICK);

  motor_left.init();
  motor_right.init();

  robot_status.controlled_position = false;
  robot_status.controlled_rotation = false;
  robot_status.controlled_speed = false;

  robot_status.accel_max = MAX_ACCEL_TICK;
  robot_status.speed_max_translation = MAX_SPEED_TRANSLATION_TICK;
  robot_status.speed_max_rotation = MAX_SPEED_ROTATION_TICK;
  robot_status.speed_max_wheel = MAX_SPEED_TRANSLATION_TICK;

  robot_status.differential_tolerance = 800;
  robot_status.rotation_tolerance = 15;
  robot_status.translation_tolerance = 15;
  robot_status.pwm_tolerance = 0.9*CONST_PWM_MAX;

  block_status_left.reset();
  block_status_right.reset();

  MX_TIM14_Init();
}


void MotionController::update_position() {
  // Update encoder positions and speeds
  cod_right.update(COD_get_right()); // Right wheel is positive (trigo rotations)
  cod_left.update(-COD_get_left());  // Left wheel is the opposite

  // Update robot translation and rotation
  robot_status.update_position(cod_left.ticks_current, cod_right.ticks_current);
}


void MotionController::control_motion() {
  if(robot_status.controlled_position || robot_status.controlled_rotation || robot_status.controlled_speed){

    if(robot_status.controlled_position || robot_status.controlled_rotation){
      robot_status.translation_speed_setpoint = pid_translation.compute(robot_status.translation_total, robot_status.translation_setpoint);
      robot_status.rotation_speed_setpoint    = pid_rotation.compute(robot_status.rotation_total, robot_status.rotation_setpoint);

      //CAP ROTATION/TRANSLATION SPEED
      if(robot_status.translation_speed_setpoint > robot_status.speed_max_translation){
        robot_status.translation_speed_setpoint = robot_status.speed_max_translation;
      }
      else if(robot_status.translation_speed_setpoint< -robot_status.speed_max_translation){
        robot_status.translation_speed_setpoint = -robot_status.speed_max_translation;
      }

      if(robot_status.rotation_speed_setpoint > robot_status.speed_max_rotation){
        robot_status.rotation_speed_setpoint = robot_status.speed_max_rotation;
      }
      else if(robot_status.rotation_speed_setpoint < -robot_status.speed_max_rotation){
        robot_status.rotation_speed_setpoint = -robot_status.speed_max_rotation;
      }

      //Update wheel speed setpoints
      // Trigonometric rotation makes the left wheel go backwards
      cod_left.speed_setpoint  = robot_status.translation_speed_setpoint
                               - robot_status.rotation_speed_setpoint;
      // But makes the right wheel go forwards
      cod_right.speed_setpoint = robot_status.translation_speed_setpoint
                               + robot_status.rotation_speed_setpoint;
    }
    else{
      //Only use speed, for tuning
      cod_left.speed_setpoint  = cod_left.speed_setpoint_wanted;
      cod_right.speed_setpoint = cod_right.speed_setpoint_wanted;
    }

    //Wheel Acceleration limits
    cod_left.cap_accel(robot_status.accel_max);
    cod_right.cap_accel(robot_status.accel_max);

    //Wheel speed limits
    cod_left.cap_speed(robot_status.speed_max_wheel);
    cod_right.cap_speed(robot_status.speed_max_wheel);
  }

  if(robot_status.controlled_speed){
    cod_left.pwm = pid_speed_left.compute(cod_left.speed_average, cod_left.speed_setpoint);
    cod_right.pwm = pid_speed_right.compute(cod_right.speed_average, cod_right.speed_setpoint);
  }

  if(robot_status.controlled_speed){
    motor_left.set_pwm(cod_left.pwm);
    motor_right.set_pwm(cod_right.pwm);
  }
}


void MotionController::set_target_speed(Motor::Side side, int32_t speed){
  switch(side){
    case Motor::Side::LEFT:
      cod_left.speed_setpoint_wanted = speed;
      break;
    case Motor::Side::RIGHT:
      cod_right.speed_setpoint_wanted = speed;
      break;
  }
  robot_status.moving = true;
  robot_status.forced_movement = true;
}


void MotionController::translate_ticks(int32_t distance_ticks){
  robot_status.translation_setpoint += distance_ticks;
  robot_status.moving = true;
}

void MotionController::rotate_ticks(int32_t distance_ticks){
  robot_status.rotation_setpoint += distance_ticks;
  robot_status.moving = true;
}


int32_t MotionController::get_COD_left(){
  return cod_left.ticks_current;
}


int32_t MotionController::get_COD_right(){
  return cod_right.ticks_current;
}

/*
bool MotionController::is_wheel_blocked(wheel_block_status& wheel_status, const encoder_status& cod_status, PID_FP& pid_status){
  // Each time the wheel speed starts to be far from its setpoint AND the PWM is high (the wheel PID is saturated) AND the robot is farm from its
  // a count is initialized and goes down to 0 and stays there until the wheel isn't blocked
  if(robot_status.moving && (ABS(cod_status.speed_current) < ABS(cod_status.speed_setpoint)*wheel_status.LIMIT)
      &&
      (ABS(pid_translation.get_error())>5*robot_status.translation_tolerance)
      &&
      (ABS(pid_rotation.get_error())>5*robot_status.rotation_tolerance)
      &&
      (ABS(cod_status.pwm) > robot_status.pwm_tolerance)
  ){
    // Init count if first block detection
    if(!wheel_status.blocked){
      wheel_status.blocked = true; //Start block detection
      wheel_status.count_blocks = wheel_status.INIT_COUNT; //Countdown init
    }
    // Downcount until 0 is reached : enough block cycles were detected
    if(wheel_status.count_blocks > 0){
      wheel_status.count_blocks--;
    }
  }
  else{
    wheel_status.blocked = false;
  }
  return (wheel_status.blocked && (wheel_status.count_blocks == 0));
}*/

// Detects if the robot is physically stopped (blocked)
bool MotionController::is_robot_blocked(){

  bool left_wheel_blocked  = block_status_left.is_blocked();
  bool right_wheel_blocked = block_status_right.is_blocked();

  return left_wheel_blocked || right_wheel_blocked;
}


void MotionController::detect_stop(){
  bool status_block; // is the robot physically blocked and should stop?
  bool status_end;   // is the robot at the end of a movement, as expected?
  volatile uint32_t t = micros();

  block_status_left.update();
  block_status_right.update();
  robot_status.update_movement_done(pid_translation.get_error(), pid_rotation.get_error());

  status_block = is_robot_blocked();
  status_end = robot_status.is_movement_done();

  t = micros()-t;

  // if done or blocked, stop the robot
  if(robot_status.moving && !robot_status.forced_movement && (status_block || status_end)){
    stop(status_block && !status_end); //Only count as blocked if the movement is not at its end
  }

  //TODO: wheel block checker (blocking means pwm high)
  //TODO: if blocked or stop order : go into stopping state, and block wheels until speed is zero long enough.
  //TODO: graph of PWM and errors to check for block tolerances
  /*
  static const uint8_t INIT_BLOCK = 20;
  static const uint8_t INIT_STOP  = 5;

  static uint8_t time_block = INIT_BLOCK;
  static uint8_t time_stop  = INIT_STOP;
  int32_t err_trans = ABS(pid_translation.get_error());
  int32_t err_rot   = ABS(pid_rotation.get_error());

  if(robot_status.moving && !robot_status.forced_movement
      && (err_trans <= robot_status.translation_tolerance && err_rot <= robot_status.rotation_tolerance))
  {
    if(time_stop > 0){
      time_stop--;
    }
    else{
      stop(false);
    }
  }
  else if(robot_status.moving && !robot_status.forced_movement &&
      (((pid_translation.get_derivative() == 0) && (pid_rotation.get_derivative()==0))
        || (ABS(ABS(pid_speed_left.get_error())-ABS(pid_speed_right.get_error()))>robot_status.differential_tolerance))
    && (err_trans > 5*robot_status.translation_tolerance || err_rot > 5*robot_status.rotation_tolerance))
  {
    if(time_block > 0){
      time_block--;
    }
    else{
      stop(true);
    }
  }

  else{
    time_block = INIT_BLOCK;
    time_stop  = INIT_STOP;
  }*/
}

// Stops the robot at its current position
void MotionController::stop(bool wheels_blocked){
  //Disable Motion control interrupt, to modify its variables safely
  LL_TIM_DisableCounter(TIM14);

  // Order a full stop of the robot:
  // // Position wanted: current position
  robot_status.translation_setpoint = robot_status.translation_total;
  robot_status.rotation_setpoint = robot_status.rotation_total;
  // // Order a stop of the motors
  motor_left.set_pwm(0);
  motor_right.set_pwm(0);
  // // Reset PID errors, to react faster to the instant change of setpoint
  pid_translation.reset();
  pid_rotation.reset();
  pid_speed_left.reset();
  pid_speed_right.reset();

  // // stop wheels, for tuning of speed PIDs
  cod_left.speed_setpoint_wanted = 0;
  cod_right.speed_setpoint_wanted = 0;

  robot_status.movement_stopped = true;   // Signal a stop event
  robot_status.moving = false;            // Stop current movement
  robot_status.blocked = wheels_blocked;  // Signal blocked wheels
  robot_status.forced_movement = false;   // If there was a forced movement, stop it

  LL_TIM_EnableCounter(TIM14);
}

MotionController::STOP_STATUS MotionController::has_stopped(){
  STOP_STATUS return_val=NONE; //default : not stopped, not blocked

  if(robot_status.blocked){
    return_val = BLOCKED;   // stopped and/or blocked
  }else if(robot_status.movement_stopped){
    return_val = STOPPED;    // stopped, not blocked
  }

  //acknowledge stop
  robot_status.blocked = false;
  robot_status.movement_stopped = false;

  return return_val;
}

int32_t MotionController::get_data_from_pid(PID_FP& pid, uint8_t requested_data){
  int32_t ret_data;
  switch(requested_data){
    case PID_DATA_ID::PROPORTIONAL:
      ret_data = pid.get_proportional();
      break;
    case PID_DATA_ID::INTEGRAL_MSB:
      ret_data = (pid.get_integral()>>32)&0xFFFFFFFF;
      break;
    case PID_DATA_ID::INTEGRAL_LSB:
      ret_data = pid.get_integral()&0xFFFFFFFF;
      break;
    case PID_DATA_ID::DERIVATIVE:
      ret_data = pid.get_derivative();
      break;
    default:
      ret_data = 0;
      break;
  }
  return ret_data;
}


/**
 * Gets the PID data associated with the PID_DATA_ID requested_data, for a given PID_ID id
 */
int32_t MotionController::get_pid_data(uint8_t id, uint8_t requested_data){
  int32_t ret_val;
  switch(id){
    case PID_ID::PID_LEFT_SPEED:
      ret_val = get_data_from_pid(pid_speed_left, requested_data);
      break;
    case PID_ID::PID_RIGHT_SPEED:
      ret_val = get_data_from_pid(pid_speed_right, requested_data);
      break;
    case PID_ID::PID_TRANSLATION:
      ret_val = get_data_from_pid(pid_translation, requested_data);
      break;
    case PID_ID::PID_ROTATION:
      ret_val = get_data_from_pid(pid_rotation, requested_data);
      break;
    default:
      while(1);
  }
  return ret_val;
}



void MotionController::set_raw_pwm(Motor::Side side, int16_t pwm){
  switch(side){
    case Motor::Side::LEFT :
      motor_left.set_pwm(pwm);
      break;
    case Motor::Side::RIGHT:
      motor_right.set_pwm(pwm);
      break;
  }
}


void MotionController::set_kp(uint8_t id, uint32_t k){
  switch(id){
  case PID_ID::PID_LEFT_SPEED :
    pid_speed_left.set_kp(k);
    break;
  case PID_ID::PID_RIGHT_SPEED :
    pid_speed_right.set_kp(k);
    break;
  case PID_ID::PID_TRANSLATION :
    pid_translation.set_kp(k);
    break;
  case PID_ID::PID_ROTATION :
    pid_rotation.set_kp(k);
    break;
  default:
    while(true);
  }
}


void MotionController::set_ki(uint8_t id, uint32_t k){
  switch(id){
  case PID_ID::PID_LEFT_SPEED :
    pid_speed_left.set_ki(k, MOTION_CONTROL_FREQ);
    break;
  case PID_ID::PID_RIGHT_SPEED :
    pid_speed_right.set_ki(k, MOTION_CONTROL_FREQ);
    break;
  case PID_ID::PID_TRANSLATION :
    pid_translation.set_ki(k, MOTION_CONTROL_FREQ);
    break;
  case PID_ID::PID_ROTATION :
    pid_rotation.set_ki(k, MOTION_CONTROL_FREQ);
    break;
  default:
    while(true);
  }
}


void MotionController::set_kd(uint8_t id, uint32_t k){
  switch(id){
    case PID_ID::PID_LEFT_SPEED :
      pid_speed_left.set_kd(k, MOTION_CONTROL_FREQ);
      break;
    case PID_ID::PID_RIGHT_SPEED :
      pid_speed_right.set_kd(k, MOTION_CONTROL_FREQ);
      break;
    case PID_ID::PID_TRANSLATION :
      pid_translation.set_kd(k, MOTION_CONTROL_FREQ);
      break;
    case PID_ID::PID_ROTATION :
      pid_rotation.set_kd(k, MOTION_CONTROL_FREQ);
      break;
  default:
    while(true);
  }
}


void MotionController::set_control(bool speed, bool translation, bool rotation){
  robot_status.controlled_speed    = speed;
  robot_status.controlled_position = translation;
  robot_status.controlled_rotation = rotation;
}

void MotionController::set_limits(uint16_t speed_translation, uint16_t speed_rotation, uint16_t speed_wheel, uint16_t accel_wheel){
  robot_status.speed_max_translation = speed_translation;
  pid_translation.set_output_limit(speed_translation);
  pid_translation.set_anti_windup(speed_translation);
  pid_translation.set_derivative_limit(speed_translation);

  robot_status.speed_max_rotation = speed_rotation;
  pid_rotation.set_output_limit(speed_rotation);
  pid_rotation.set_anti_windup(speed_rotation);
  pid_rotation.set_derivative_limit(speed_rotation);

  robot_status.speed_max_wheel= speed_wheel;
  robot_status.accel_max = accel_wheel/MOTION_CONTROL_FREQ; // Acceleration is in tick/s each mcs update iteration
}
