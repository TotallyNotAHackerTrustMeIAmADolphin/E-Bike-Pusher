#ifndef ODRIVE_CAN_H
#define ODRIVE_CAN_H

#include <Arduino.h>
#include "driver/twai.h"

#define ODRIVE_NODE_ID 0

#define CMD_HEARTBEAT 0x01
#define CMD_SET_AXIS_STATE 0x07
#define CMD_GET_ENCODER_ESTIMATES 0x09
#define CMD_SET_CONTROLLER_MODE 0x0B
#define CMD_SET_INPUT_VEL 0x0D
#define CMD_SET_INPUT_TORQUE 0x0E
#define CMD_GET_IQC 0x14
#define CMD_GET_VBUS_VOLTAGE 0x17
#define CMD_CLEAR_ERRORS 0x18

// ODrive v0.5.1 Axis Error Bits
#define ODRV_ERROR_NONE 0x00
#define ODRV_ERROR_INVALID_STATE 0x01
#define ODRV_ERROR_DC_BUS_OVER_VOLTAGE 0x02
#define ODRV_ERROR_DC_BUS_UNDER_VOLTAGE 0x04
#define ODRV_ERROR_BOOST_MARGIN_VIOLATED 0x08
#define ODRV_ERROR_PHASE_RESISTANCE_OUT_OF_RANGE 0x10
#define ODRV_ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE 0x20
#define ODRV_ERROR_MOTOR_FAILED 0x40
#define ODRV_ERROR_ENCODER_FAILED 0x80
#define ODRV_ERROR_CONTROLLER_FAILED 0x100
#define ODRV_ERROR_POS_CTRL_DURING_SENSORLESS 0x200
#define ODRV_ERROR_WATCHDOG_TIMER_EXPIRED 0x800
#define ODRV_ERROR_MIN_ENDSTOP_PRESSED 0x1000
#define ODRV_ERROR_MAX_ENDSTOP_PRESSED 0x2000
#define ODRV_ERROR_ESTOP_REQUESTED 0x4000
#define ODRV_ERROR_DC_BUS_OVER_CURRENT 0x8000
#define ODRV_ERROR_DC_BUS_MAX_REGEN_CURRENT_VIOLATED 0x10000
#define ODRV_ERROR_DC_BUS_MAX_POSITIVE_CURRENT_VIOLATED 0x20000

class ODriveCAN
{
private:
  uint8_t node_id;
  float odrv_vel;
  float odrv_current;
  float odrv_vbus;
  float odrv_ibus;
  uint8_t odrv_state;
  uint32_t odrv_error;
  unsigned long last_heartbeat; // NEW: Timestamp of last heartbeat

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
  void clearErrors();

  float getVelocity() const;
  float getCurrent() const;
  float getVoltage() const;
  float getBusCurrent() const;
  uint8_t getState() const;
  uint32_t getError() const; // NEW
  bool isDataFresh() const;  // NEW: Check if heartbeat is recent
};

#endif