/// @file tmotor_ak_actuators.cpp
/// @brief T-Motor/CubeMars AK series actuators library.
/// @author ZiTe (honmonoh@gmail.com)

#include "tmotor_ak_actuators.hpp"

namespace TMotorActuators
{
  AkActuators::AkActuators(uint8_t motorId, MotorParameters motorParas, SendCanDataFunction canSendFunc)
  {
    mId = motorId;
    mMotorParameters = motorParas;
    sendCanData = canSendFunc;

    disable();
  }

  void AkActuators::enable(void)
  {
    uint8_t data[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    sendCanData(mId, 8, data);
  }

  void AkActuators::disable(void)
  {
    uint8_t data[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    sendCanData(mId, 8, data);
  }

  void AkActuators::setZeroPosition(void)
  {
    uint8_t data[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
    sendCanData(mId, 8, data);
  }

  void AkActuators::move(float position, float velocity, float torque, float kp, float kd)
  {
    int32_t pInt = convFloatToUint(position, mMotorParameters.positionMin, mMotorParameters.positionMax, 16);
    int32_t vInt = convFloatToUint(velocity, mMotorParameters.velocityMin, mMotorParameters.velocityMax, 12);
    int32_t tInt = convFloatToUint(torque, mMotorParameters.torqueMin, mMotorParameters.torqueMax, 12);
    int32_t kpInt = convFloatToUint(kp, mMotorParameters.kpMin, mMotorParameters.kpMax, 12);
    int32_t kdInt = convFloatToUint(kd, mMotorParameters.kdMin, mMotorParameters.kdMax, 12);

    uint8_t data[8];
    data[0] = pInt >> 8;
    data[1] = pInt & 0xFF;
    data[2] = vInt >> 4;
    data[3] = ((vInt & 0xF) << 4 | (kpInt >> 8));
    data[4] = kpInt & 0xFF;
    data[5] = kdInt >> 4;
    data[6] = ((kdInt & 0xF) << 4 | (tInt >> 8));
    data[7] = tInt & 0xFF;

    sendCanData(mId, 8, data);
  }

  AkActuators::MotorState AkActuators::parseMotorState(uint8_t *canData)
  {
    uint16_t pInt = (canData[1] << 8) | canData[2];
    uint16_t vInt = (canData[3] << 4) | (canData[4] >> 4);
    uint16_t tInt = ((canData[4] & 0xF) << 8) | canData[5];

    MotorState state;
    state.id = canData[0];
    state.position = convUintToFloat(pInt, mMotorParameters.positionMin, mMotorParameters.positionMax, 16);
    state.velocity = convUintToFloat(vInt, mMotorParameters.velocityMin, mMotorParameters.velocityMax, 12);
    state.torque = convUintToFloat(tInt, -mMotorParameters.torqueMax, mMotorParameters.torqueMax, 12); // XXX

    return state;
  }

  int32_t AkActuators::convFloatToUint(float val, float min, float max, uint8_t bits)
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
    return (int32_t)((val - min) * ((float)((1 << bits) - 1)) / span);
  }

  float AkActuators::convUintToFloat(int32_t val, float min, float max, uint8_t bits)
  {
    float span = max - min;
    return ((float)val) * span / ((float)((1 << bits) - 1)) + min;
  }

  uint8_t AkActuators::getId(void)
  {
    return this->mId;
  }
}
