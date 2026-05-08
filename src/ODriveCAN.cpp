#include "ODriveCAN.h"

ODriveCAN::ODriveCAN(uint8_t node) : node_id(node), odrv_vel(0.0), odrv_current(0.0), odrv_vbus(0.0), odrv_ibus(0.0) {}

bool ODriveCAN::begin(gpio_num_t tx_pin, gpio_num_t rx_pin)
{
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(tx_pin, rx_pin, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK)
  {
    if (twai_start() == ESP_OK)
    {
      return true;
    }
  }
  return false;
}

void ODriveCAN::twai_send(uint32_t cmd_id, uint8_t *data, uint8_t len)
{
  // THE FIX: Zero initialize first, then set properties safely!
  twai_message_t msg = {};
  msg.identifier = (node_id << 5) | cmd_id;
  msg.extd = 0;
  msg.rtr = 0;
  msg.data_length_code = len;

  memcpy(msg.data, data, len);
  twai_transmit(&msg, 0);
}

void ODriveCAN::requestData(uint8_t cmd_id)
{
  // THE FIX: Zero initialize first!
  twai_message_t msg = {};
  msg.identifier = (node_id << 5) | cmd_id;
  msg.extd = 0;
  msg.rtr = 1;
  msg.data_length_code = 0;

  twai_transmit(&msg, 0);
}

void ODriveCAN::poll()
{
  twai_message_t msg;
  while (twai_receive(&msg, 0) == ESP_OK)
  {
    if (msg.rtr)
      continue;

    uint8_t received_node = msg.identifier >> 5;
    uint8_t cmd_id = msg.identifier & 0x1F;

    if (received_node == node_id)
    {
      if (cmd_id == CMD_GET_ENCODER_ESTIMATES)
      {
        memcpy(&odrv_vel, &msg.data[4], 4);
      }
      else if (cmd_id == CMD_GET_IQC)
      {
        memcpy(&odrv_current, &msg.data[4], 4);
      }
      else if (cmd_id == CMD_GET_VBUS_VOLTAGE)
      {
        memcpy(&odrv_vbus, &msg.data[0], 4);
        memcpy(&odrv_ibus, &msg.data[4], 4);
      }
    }
  }
}

void ODriveCAN::setMode(int32_t control_mode, int32_t input_mode)
{
  uint8_t data[8] = {0};
  memcpy(&data[0], &control_mode, 4);
  memcpy(&data[4], &input_mode, 4);
  twai_send(CMD_SET_CONTROLLER_MODE, data, 8);
}

void ODriveCAN::setState(uint32_t state)
{
  uint8_t data[4] = {0};
  memcpy(&data[0], &state, 4);
  twai_send(CMD_SET_AXIS_STATE, data, 4);
}

void ODriveCAN::setTorque(float torque_amps)
{
  uint8_t data[8] = {0};
  memcpy(&data[0], &torque_amps, 4);
  twai_send(CMD_SET_INPUT_TORQUE, data, 8);
}

void ODriveCAN::setVelocity(float velocity)
{
  uint8_t data[8] = {0};
  memcpy(&data[0], &velocity, 4);
  twai_send(CMD_SET_INPUT_VEL, data, 8);
}

float ODriveCAN::getVelocity() const { return odrv_vel; }
float ODriveCAN::getCurrent() const { return odrv_current; }
float ODriveCAN::getVoltage() const { return odrv_vbus; }
float ODriveCAN::getBusCurrent() const { return odrv_ibus; }