/**
 * Communication header
 */

#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <Arduino.h>

struct CommandData {
    uint8_t type;
    uint16_t flywheel_rpm;
    uint8_t ir_state;     // 0 = Detected, 1 = Not Detected
    int16_t drive_left;   // 16-bit: supports full -400 to +400 motor range
    int16_t drive_right;  // 16-bit: supports full -400 to +400 motor range
};

class Communication {
public:
    Communication();
    void begin();
    void update();

    // Packet parsing
    bool hasPacket();
    void parsePacket(CommandData& cmd);

    // Send telemetry (only 2 IR sensors)
    // Send telemetry (only 2 IR sensors)
    void sendSensorData(uint8_t ir_left, uint8_t ir_right, uint16_t distance_cm);
    void sendLoaderStatus(bool is_ready);
    void sendDebugMessage(const char* msg);

private:
    uint8_t rx_buffer[32];  // MAX_PACKET_SIZE
    uint8_t rx_buffer_index;
    bool packet_ready;

    void sendPacket(uint8_t msg_type, uint8_t* data, uint8_t data_len);
    uint8_t calculateChecksum(uint8_t* data, uint8_t len);
};

extern Communication comm;

#endif // COMMUNICATION_H
