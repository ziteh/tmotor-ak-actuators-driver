#include "tmotor_ak_actuators.h"

namespace TMotorAkActuators
{
  AkActuators::AkActuators(uint16_t motorId, motorParameters_t motorParas, canSendFunc_t canSendFunc)
  {
    id = motorId;
    motorParameters = motorParas;
    canSend = canSendFunc;

    disable();
  }

  AkActuators::~AkActuators()
  {
    disable();
  }

  void AkActuators::enable(void)
  {
    uint8_t data[] = {
        0xFF,
        0xFF,
        0xFF,
        0xFF,
        0xFF,
        0xFF,
        0xFF,
        0xFC};
    canSend(id, 8, data);
  }

  void AkActuators::disable(void)
  {
    uint8_t data[] = {
        0xFF,
        0xFF,
        0xFF,
        0xFF,
        0xFF,
        0xFF,
        0xFF,
        0xFD};
    canSend(id, 8, data);
  }

  void AkActuators::setZeroPosition(void)
  {
    uint8_t data[] = {
        0xFF,
        0xFF,
        0xFF,
        0xFF,
        0xFF,
        0xFF,
        0xFF,
        0xFE};
    canSend(id, 8, data);
  }

  void AkActuators::move(float position, float velocity, float torque, float kp, float kd)
  {
    uint32_t p_i = floatToUint(position, motorParameters.positionMin, motorParameters.positionMax, 16);
    uint32_t v_i = floatToUint(velocity, motorParameters.velocityMin, motorParameters.velocityMax, 12);
    uint32_t t_i = floatToUint(torque, motorParameters.torqueMin, motorParameters.torqueMax, 12);
    uint32_t kp_i = floatToUint(kp, motorParameters.kpMin, motorParameters.kpMax, 12);
    uint32_t kd_i = floatToUint(kd, motorParameters.kdMin, motorParameters.kdMax, 12);

    uint8_t data[8];
    data[0] = p_i >> 8;
    data[1] = p_i & 0xFF;
    data[2] = v_i >> 4;
    data[3] = ((v_i & 0xF) << 4 | (kp_i >> 8));
    data[4] = kp_i & 0xFF;
    data[5] = kd_i >> 4;
    data[6] = ((kd_i & 0xF) << 4 | (t_i >> 8));
    data[7] = t_i & 0xFF;

    canSend(id, 8, data);
  }

  AkActuators::motorState_t AkActuators::parseMotorState(uint8_t *canData)
  {
    motorState_t state;

    uint16_t p_i = (canData[1] << 8) | canData[2];
    uint16_t v_i = (canData[3] << 4) | (canData[4] >> 4);
    uint16_t t_i = ((canData[4] & 0xF) << 8) | canData[5];

    state.id = canData[0];
    state.position = uintToFloat(p_i, motorParameters.positionMin, motorParameters.positionMax, 16);
    state.velocity = uintToFloat(v_i, motorParameters.velocityMin, motorParameters.velocityMax, 12);
    state.torque = uintToFloat(t_i, -motorParameters.torqueMax, motorParameters.torqueMax, 12); // XXX

    return state;
  }

  uint32_t AkActuators::floatToUint(float val, float min, float max, uint8_t bits)
  {
    /* Limits. */
    if (val > max)
    {
      val = max;
    }
    else if (val < min)
    {
      val = min;
    }

    float span = max - min;
    return (uint32_t)((val - min) * ((float)((1 << bits) - 1)) / span);
  }

  float AkActuators::uintToFloat(uint32_t val, float min, float max, uint8_t bits)
  {
    float span = max - min;
    return ((float)val) * span / ((float)((1 << bits) - 1)) + min;
  }

  uint16_t AkActuators::getId(void)
  {
    return this->id;
  }
}
