#include "crc.h"
#include "serial_device.h"
#include "wust_base.h"


// SerialDevice serial("/dev/ttyACM0");


void protocol_transmit(uint16_t cmd_id, uint8_t* data, uint16_t len);
void protocol_receive(uint16_t cmd_id, uint8_t* data, uint16_t len);