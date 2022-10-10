#ifndef TMOTOR_AK_ACTUATORS_H_
#define TMOTOR_AK_ACTUATORS_H_

#include <stdint.h>

namespace TMotorAkActuators
{
  /**
   * @brief T-Motor/CubeMars AK series actuators.
   */
  class AkActuators
  {
public:
    typedef void (*canSendFunc_t)(uint32_t id, uint8_t dlc, uint8_t *data);

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
    } motorParameters_t;

    motorParameters_t AK10_9_v1_1 = {
        -12.5,
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
    } motorState_t;

private:
    uint16_t id;

    canSendFunc_t canSend;
    motorParameters_t motorParameters;

    uint32_t floatToUint(float val, float min, float max, uint8_t bits);
    float uintToFloat(uint32_t val, float min, float max, uint8_t bits);

public:
    /**
     * @brief Construct a new AK actuators object.
     *
     * @param motorId Motor ID, same as CAN id.
     * @param motorParas Motor parameters.
     * @param canSendFunc CAN Bus send message function.
     */
    AkActuators(uint16_t motorId, motorParameters_t motorParas, canSendFunc_t canSendFunc);
    ~AkActuators();

    /**
     * @brief Enter motor control mode.
     */
    void enable(void);

    /**
     * @brief Exit motor control mode.
     */
    void disable(void);

    /**
     * @brief Set the current position of the motor to 0.
     */
    void setZeroPosition(void);

    /**
     * @brief Send command to contol the motor.
     *
     * @param position Goal position in rad.
     * @param velocity Goal speed in rad/s.
     * @param torque Goal torque in Nm.
     * @param kp PID kp.
     * @param kd PID kd.
     */
    void move(float position, float velocity, float torque, float kp, float kd = 0);

    motorState_t parseMotorState(uint8_t *canData);

    /**
     * @brief Get the motor ID.
     *
     * @return uint16_t Motor ID
     */
    uint16_t getId(void);
  };

}

#endif /* TMOTOR_AK_ACTUATORS_H_ */