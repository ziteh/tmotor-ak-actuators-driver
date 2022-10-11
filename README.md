# T-Motor/CubeMars AK Actuators Driver

T-Motor/CubeMars AK series actuators (MIT Mini-Cheetah type) library

## Usage
```cpp
void sendCanData(uint32_t id, uint8_t dlc, uint8_t *data)
{
  can_frame frame;
  frame.can_id = id;
  frame.can_dlc = dlc;
  for (int i = 0; i < dlc; i++)
  {
    frame.data[i] = data[i];
  }

  mcp2515->sendMessage(&frame);
}

int main(void)
{
  /* Some code here. */

  uint8_t motorId = 1;
  TMotorActuators::AkActuators ak10 = TMotorActuators::AkActuators(motorId,
                                                                   TMotorActuators::ak10_9_v1_1,
                                                                   sendCanData);

  ak10.enable();
  delay(1000);

  ak10.move(0.0, -50, 0, 2, 0.2); // Move to 0 rad.
  delay(1000);

  ak10.move(3.1415, -50, 0, 2, 0.2); // Move to 3.1415 rad.
  delay(1000);

  ak10.disable();
  delay(1000);
}
```