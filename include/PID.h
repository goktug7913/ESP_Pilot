#include "Config.h"
#include "definitions.h"
#include <array>
#pragma once

/**
 * @brief PID Controller class
 * @details This class implements a PID controller
 * @version 0.1
*/
class PID {
  public:
  
  /**
   * @brief Construct a new PID object with default gains
  */
  PID();

  /**
   * @brief Construct a new PID object with given gains
  */
  PID(float p_gain, float i_gain, float d_gain);

  // Public Functions of the PID instance

  float               Calculate(float pv, float target, float dt);
  void                SetGains(float p_gain, float i_gain, float d_gain);
  std::array<float,3> GetGains();


  // TODO: move to private
  float Kp{}, Ki{}, Kd{};   //PID Gains
  float p{}, i{}, d{};      //Error values
  float p_prev{};       //Previous P Term
  float dT{};       //Delta time in millisecs
  
  float iScalar = ITERMSCALAR;
  float iMax =  ITERMLIMIT;
  float iMin = -ITERMLIMIT;
};