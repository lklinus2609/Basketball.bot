/**
 * Non-blocking serial communication for A-Star
 * Packet-based protocol with checksum
 */

#include "communication.h"
#include "config.h"

Communication comm;

Communication::Communication() {
    rx_buffer_index = 0;
    packet_ready = false;
}

void Communication::begin() {
    Serial.begin(SERIAL_BAUD);
}

uint8_t Communication::calculateChecksum(uint8_t* data, uint8_t len) {
    uint16_t sum = 0;
    for (uint8_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return (uint8_t)(sum & 0xFF);
}

void Communication::sendPacket(uint8_t msg_type, uint8_t* data, uint8_t data_len) {
    uint8_t packet[MAX_PACKET_SIZE];
    uint8_t idx = 0;

    packet[idx++] = START_BYTE;
    packet[idx++] = msg_type;
    packet[idx++] = data_len;

    // Copy data
    for (uint8_t i = 0; i < data_len && idx < MAX_PACKET_SIZE - 1; i++) {
        packet[idx++] = data[i];
    }

    // Calculate checksum (exclude START_BYTE)
    uint8_t checksum = calculateChecksum(&packet[1], idx - 1);
    packet[idx++] = checksum;

    // Send packet
    Serial.write(packet, idx);
}

void Communication::sendSensorData(uint8_t ir_left, uint8_t ir_right, uint16_t distance_cm) {
    uint8_t data[4];
    data[0] = ir_left;
    data[1] = ir_right;
    data[2] = (uint8_t)(distance_cm & 0xFF);        // Low byte
    data[3] = (uint8_t)((distance_cm >> 8) & 0xFF); // High byte

    sendPacket(TEL_SENSOR_DATA, data, 4);
}

void Communication::sendFlywheelStatus(uint16_t rpm_left, uint16_t rpm_right) {
    uint8_t data[4];
    data[0] = (uint8_t)(rpm_left & 0xFF);
    data[1] = (uint8_t)((rpm_left >> 8) & 0xFF);
    data[2] = (uint8_t)(rpm_right & 0xFF);
    data[3] = (uint8_t)((rpm_right >> 8) & 0xFF);

    sendPacket(TEL_FLYWHEEL_STATUS, data, 4);
}

void Communication::sendPositionStatus(float angle_degrees) {
    // Encode angle with 0.1° resolution
    int16_t angle_encoded = (int16_t)(angle_degrees * 10);

    uint8_t data[2];
    data[0] = (uint8_t)(angle_encoded & 0xFF);
    data[1] = (uint8_t)((angle_encoded >> 8) & 0xFF);

    sendPacket(TEL_POSITION_STATUS, data, 2);
}

void Communication::sendLoaderStatus(bool is_ready) {
    uint8_t data[1];
    data[0] = is_ready ? 1 : 0;

    sendPacket(TEL_LOADER_STATUS, data, 1);
}

void Communication::sendDebugMessage(const char* msg) {
    uint8_t len = strlen(msg);
    if (len > MAX_PACKET_SIZE - 4) {
        len = MAX_PACKET_SIZE - 4;
    }

    sendPacket(TEL_DEBUG_MESSAGE, (uint8_t*)msg, len);
}

void Communication::update() {
    // Read available bytes (non-blocking)
    while (Serial.available()) {
        uint8_t byte = Serial.read();

        // Look for start byte
        if (rx_buffer_index == 0 && byte != START_BYTE) {
            continue; // Skip until we find start byte
        }

        rx_buffer[rx_buffer_index++] = byte;

        // Check if we have complete header (START + CMD + LEN)
        if (rx_buffer_index >= 3) {
            uint8_t data_len = rx_buffer[2];
            uint8_t packet_len = 3 + data_len + 1; // Header + data + checksum

            // Check for complete packet
            if (rx_buffer_index >= packet_len) {
                // Verify checksum
                uint8_t received_checksum = rx_buffer[packet_len - 1];
                uint8_t calculated_checksum = calculateChecksum(&rx_buffer[1], packet_len - 2);

                if (received_checksum == calculated_checksum) {
                    // Valid packet!
                    packet_ready = true;
                }

                // Reset buffer
                rx_buffer_index = 0;
                break;
            }
        }

        // Prevent buffer overflow
        if (rx_buffer_index >= MAX_PACKET_SIZE) {
            rx_buffer_index = 0;
        }
    }
}

bool Communication::hasPacket() {
    return packet_ready;
}

void Communication::parsePacket(CommandData& cmd) {
    if (!packet_ready) {
        return;
    }

    cmd.type = rx_buffer[1];
    uint8_t data_len = rx_buffer[2];

    // Parse based on command type
    switch (cmd.type) {
        case CMD_SET_FLYWHEEL_RPM:
            if (data_len >= 2) {
                cmd.flywheel_rpm = ((uint16_t)rx_buffer[4] << 8) | rx_buffer[3];
            }
            break;

        case CMD_ROTATE_LAZY_SUSAN:
            if (data_len >= 2) {
                int16_t angle_encoded = ((int16_t)rx_buffer[4] << 8) | rx_buffer[3];
                cmd.lazy_susan_angle = angle_encoded / 10.0; // Decode 0.1° resolution
            }
            break;

        case CMD_SHOOT:
            // No data needed
            break;

        case CMD_SET_DRIVE:
            if (data_len >= 2) {
                cmd.drive_left = (int8_t)rx_buffer[3];
                cmd.drive_right = (int8_t)rx_buffer[4];
            }
            break;

        case CMD_RESET:
            // No data needed
            break;
    }

    packet_ready = false; // Clear flag
}
