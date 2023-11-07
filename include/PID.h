#include "Config.h"
#include "definitions.h"
#include <array>
#pragma once

/**
 * @brief PID Controller class
 * @details This class implements a PID controller
 * @version 0.1
 */
class PID
{
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

  /**
   * @brief Calculate the PID output
   * @param pv Process Variable
   * @param target Target value
   * @param dt Delta time in milliseconds
   * @return PID output
   */
  float Calculate(float pv, float target, float dt);

  /**
   * @brief Set the PID gains
   * @param p_gain Proportional gain
   * @param i_gain Integral gain
   * @param d_gain Derivative gain
   */
  void SetGains(float p_gain, float i_gain, float d_gain);

  /**
   * @brief Get the PID gains
   * @return Array of gains
   */
  std::array<float, 3> GetGains();

  /**
   * @brief Clamp a value between a minimum and maximum
   * @param value Value to clamp
   * @param min Minimum value
   * @param max Maximum value
   * @return Clamped value
   */
  // Todo: move to a separate math library
  float clamp(float value, float min, float max);

  // TODO: move to private
  float proportionalGain{}, integralGain{}, derivativeGain{}; // PID Gains
  float p{}, integral{}, d{};                                 // Error values
  float previousError{};                                      // Previous P Term
  float deltaTime{};                                          // Delta time in millisecs

  float integralScalar = ITERMSCALAR;
  float iMax = ITERMLIMIT;
  float iMin = -ITERMLIMIT;
};