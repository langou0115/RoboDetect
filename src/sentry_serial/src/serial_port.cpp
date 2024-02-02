#include "serial_port.h"

SerialDevice serial("/dev/ttyACM0");
uint8_t txBuf[USB_FRAME_MAX_SIZE];
uint8_t seq = 0;

void protocol_transmit(uint16_t cmd_id, uint8_t* data, uint16_t len)
{
    
    /* Set frame header */
    auto *pHeader = (frame_header_t *)txBuf;
    pHeader->sof = 0xA5;
    pHeader->data_length = len;
    pHeader->seq = seq++;

    /* Calculate data size */
    uint16_t headSize = USB_FRAME_HEADER_SIZE;
    uint16_t frameSize = len + USB_FRAME_HEADER_CRC_CMDID_LEN;

    /* Apend CRC */
    memcpy(txBuf + headSize, &cmd_id, sizeof(cmd_id));
    append_crc8(txBuf, headSize);
    memcpy(txBuf + headSize + sizeof(cmd_id), data, len);
    append_crc16(txBuf, frameSize);
    
    serial.write(txBuf, frameSize);
    
}