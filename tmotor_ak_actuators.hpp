/// @file tmotor_ak_actuators.hpp
/// @brief T-Motor/CubeMars AK series actuators library header file.
/// @author ZiTe (honmonoh@gmail.com)

#ifndef TMOTOR_AK_ACTUATORS_HPP
#define TMOTOR_AK_ACTUATORS_HPP

#include <stdint.h>

/// @brief T-Motor/CubMars actuators.
namespace TMotorActuators
{
  /// @brief AK series actuators/motor.
  class AkActuators
  {
  public:
    typedef void (*SendCanDataFunction)(uint32_t id, uint8_t dlc, uint8_t *data);

    typedef struct
    {
      float positionMax;
      float positionMin;

      float velocityMax;
      float velocityMin;

      float torqueMax;
      float torqueMin;

      float kpMax;
      float kpMin;

      float kdMax;
      float kdMin;
    } MotorParameters;

    MotorParameters ak10_9_v1_1 = {-12.5,
                                   12.5,
                                   -50,
                                   50,
                                   -65,
                                   65,
                                   0,
                                   500,
                                   0,
                                   5};

    typedef struct
    {
      uint16_t id;
      float position;
      float velocity;
      float torque;
    } MotorState;

  private:
    /// @brief Motor ID, same as CAN ID.
    uint8_t mId;
    MotorParameters mMotorParameters;

    /// @brief Send CAN Bus data function.
    SendCanDataFunction sendCanData;
    uint32_t convFloatToUint(float val, float min, float max, uint8_t bits);
    float convUintToFloat(uint32_t val, float min, float max, uint8_t bits);

  public:
    /// @brief Construct a new AK actuators object.
    ///
    /// @param motorId Motor ID, same as CAN ID.
    /// @param motorParas Motor parameters.
    /// @param canSendFunc CAN Bus send data function.
    AkActuators(uint8_t motorId, MotorParameters motorParas, SendCanDataFunction canSendFunc);

    /// @brief Enter motor control mode.
    void enable(void);

    /// @brief Exit motor control mode.
    void disable(void);

    /// @brief Set the current position of the motor to 0.
    void setZeroPosition(void);

    /// @brief Send command to contol the motor.
    ///
    /// @param position Goal position in rad.
    /// @param velocity Goal speed in rad/s.
    /// @param torque Goal torque in Nm.
    /// @param kp PID kp.
    /// @param kd PID kd.
    void move(float position, float velocity, float torque, float kp, float kd = 0);

    MotorState parseMotorState(uint8_t *canData);

    /// @brief Get the motor ID.
    ///
    /// @return Motor ID
    uint8_t getId(void);
  };
}

#endif // TMOTOR_AK_ACTUATORS_HPP
