#ifndef ODRIVE_CAN_H
#define ODRIVE_CAN_H

#include <Arduino.h>
#include "driver/twai.h"

#define ODRIVE_NODE_ID 0

#define CMD_HEARTBEAT 0x01 // NEW: Heartbeat
#define CMD_SET_AXIS_STATE 0x07
#define CMD_GET_ENCODER_ESTIMATES 0x09
#define CMD_SET_CONTROLLER_MODE 0x0B
#define CMD_SET_INPUT_VEL 0x0D
#define CMD_SET_INPUT_TORQUE 0x0E
#define CMD_GET_IQC 0x14
#define CMD_GET_VBUS_VOLTAGE 0x17
#define CMD_CLEAR_ERRORS 0x18 // NEW: Clear Errors

class ODriveCAN
{
private:
  uint8_t node_id;
  float odrv_vel;
  float odrv_current;
  float odrv_vbus;
  float odrv_ibus;
  uint8_t odrv_state;  // NEW: Tracks if the ODrive is Armed or Idle
  uint32_t odrv_error; // NEW: Tracks the exact error code

  void twai_send(uint32_t cmd_id, uint8_t *data, uint8_t len);

public:
  ODriveCAN(uint8_t node_id);
  bool begin(gpio_num_t tx_pin, gpio_num_t rx_pin);
  void poll();
  void requestData(uint8_t cmd_id);

  void setMode(int32_t control_mode, int32_t input_mode);
  void setState(uint32_t state);
  void setTorque(float torque_amps);
  void setVelocity(float velocity);
  void clearErrors(); // NEW

  float getVelocity() const;
  float getCurrent() const;
  float getVoltage() const;
  float getBusCurrent() const;
  uint8_t getState() const; // NEW
};

#endif