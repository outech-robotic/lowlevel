/*
 * WheelStatusChecker.h
 *
 *  Created on: 28 mars 2020
 *      Author: Tic-Tac
 */

#ifndef MOTION_WHEELSTATUSCHECKER_HPP_
#define MOTION_WHEELSTATUSCHECKER_HPP_

#include "MOTION/PIDFP.h"
#include "MOTION/WheelStatus.hpp"
#include "MOTION/RobotStatus.hpp"

class WheelBlockChecker {

  const uint8_t INIT_COUNT = 20;
  const float LIMIT = 0.3;
  volatile uint8_t count_blocks;
  volatile bool blocked;

public:
  WheelBlockChecker(){

  }

  void update(){

  }

  bool is_blocked(){
    return false;
  }

  void reset(){

  }

};

#endif /* MOTION_WHEELSTATUSCHECKER_HPP_ */
