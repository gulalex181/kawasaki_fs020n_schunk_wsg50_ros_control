// $ g++ gripper_server_emulator.cpp -o gripper_server_emulator

#include "gripper_server_emulator.h"

int main() {
    signal(SIGINT, CtrlC_handler);

    if ((server_socket_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        perror(("[" + std::string(LOG_PREFIX) + "]: socket failed").c_str());
        exit(EXIT_FAILURE);
    }

    serverAddr.sin_addr.s_addr = INADDR_ANY;
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(PORT);

    if (bind(server_socket_fd, (struct sockaddr*) &serverAddr, sizeof(serverAddr)) < 0) {
        perror(("[" + std::string(LOG_PREFIX) + "]: bind failed").c_str());
        exit(EXIT_FAILURE);
    }

    if (listen(server_socket_fd, 0) < 0) {
        perror(("[" + std::string(LOG_PREFIX) + "]: listen failed").c_str());
        exit(EXIT_FAILURE);
    }

    while (1) {
        printf("[%s]: Listening for incoming connections\n", LOG_PREFIX);

        client_socket_fd = accept(server_socket_fd, NULL, NULL);
        if (client_socket_fd == -1) {
            perror(("[" + std::string(LOG_PREFIX) + "]: error when accepting connection").c_str());
            exit(EXIT_FAILURE);
        } else {
            printf("[%s]: Client is connected, fd - %d\n", LOG_PREFIX, client_socket_fd);

            while (1) {
                memset(request, 0, BUFFER_SIZE);

                int request_size = read(client_socket_fd, request, sizeof(request) - 1);

                if (request_size <= 0) continue;

                std::string request_prefix = "(request) ";
                
                print_packet_data(request_prefix.c_str(), request_size, request, REQUEST, "", "");

                // Request handling
                bool need_to_free;
                std::string response_status;
                std::string command_name;
                uint16_t response_size;
                uint8_t* response = request_handling(request, &response_size, &response_status, &command_name, &need_to_free);
                
                std::string response_prefix = "(response) ";
                print_packet_data(response_prefix.c_str(), response_size, response, RESPONSE, command_name, response_status);
                                
                if (write(client_socket_fd, response, response_size) < 0) {
                    perror(("[" + std::string(LOG_PREFIX) + "]: writing to socker error").c_str());
                    exit(EXIT_FAILURE);
                }

                printf("[%s]: Response is sent\n\n", LOG_PREFIX);
                
                // Clear the memory allocated for response packet
                if (need_to_free) {
                    free(response);
                }

                if (command_name == "DisconnectAnnouncement") {
                    break;
                }
            }
        }
    }
}

void CtrlC_handler(int sig) {
    close(server_socket_fd);
    exit(EXIT_SUCCESS);
}

uint8_t* request_handling(uint8_t* request_packet, uint16_t* response_size,
    std::string* response_status, std::string* command_name, bool* need_to_free) {

    // Request parsing

    TREQUEST request;
    
    request.command_id      = request_packet[3];
    request.size_of_payload = (request_packet[5] << 8) | request_packet[4];

    uint8_t _payload[request.size_of_payload];
    request.payload = _payload;
    
    for (int i = 0; i < request.size_of_payload; ++i) {
        request.payload[i] = request_packet[6 + i];
    }

    uint16_t checksum = (request_packet[6 + request.size_of_payload + 1] << 8) | request_packet[6 + request.size_of_payload];

    // Checking CRC checksum

    // PREAMLE(3 bytes) + COMMAND_ID(1) + SIZE_OF_PAYLOAD(2) + PAYLOAD + CHECKSUM(2)
    uint16_t request_checksum = checksum_update_crc16(request_packet, 3 + 1 + 2 + request.size_of_payload + 2, 0xFFFF);
    
    if (request_checksum == 0) {
        printf("[%s]: CRC Checksum is %d, OK\n\n", LOG_PREFIX, request_checksum);
    } else {
        printf("[%s]: CRC Checksum is not 0, ERROR\n\n", LOG_PREFIX);
    }

    // Command execution

    TRESPONSE response;

    switch (request.command_id) {
        // Command ID 0x07
        case DisconnectAnnouncement:
            response = disconnect_announcement(&request);
            *command_name = "DisconnectAnnouncement";
            break;
        // Command ID 0x20
        case Homing:
            response = homing(&request);
            *command_name = "Homing";
            break;
        // Command ID 0x50
        case GetSystemInformation:
            response = get_system_information(&request);
            *command_name = "GetSystemInformation";
            break;
        // Command ID 0xB0
        case Measure:
            response = measure(&request);
            *command_name = "Measure";
            break;
        default:
            // error, no such command
            *command_name = "";
            break;
    }
    
    // Response packet building

    uint8_t* response_packet = response_packet_build(&response, response_size);

    if (!response_packet) {
        response_packet = insufficient_resourses_response_packet_build(request.command_id);
        *response_size = 10;
        *response_status = "E_INSUFFICIENT_RESOURCES";
        *need_to_free = false;
    }

    *response_status = response.status;
    *need_to_free = true;

    return response_packet;
}

/*********************************************************************/
/*! 
Calculates the CRC checksum of an array by using a table.
The start value for calculating the CRC should be set to 0xFFFF.

@param *data points to the byte array from which checksum should be calculated 
@param size size of the byte array
@param crc value calculated over another array and start value of the crc16 calculation
@return CRC16 checksum 

*/
/*********************************************************************/ 
uint16_t checksum_update_crc16(uint8_t* data, u_int16_t size, uint16_t crc) {
    u_int16_t c;

    /* process each byte prior to checksum field */
    for (c = 0; c < size; c++ ) { 
        crc = CRC_TABLE[ ( crc ^ *(data++)) & 0x00FF ] ^ ( crc >> 8 );
    }

    return crc;
}

/*********************************************************************/
/*! 
Builds a data packet from the given message.
You have to free the returned buffer, if you do not use it anymore.

@param *msg Pointer to the source message
@param *size Returns the size of the created buffer

@return buffer containing the bytewise packet data or NULL in case of an error.

*/ 
/*********************************************************************/ 
uint8_t* response_packet_build(TRESPONSE *response, uint16_t* response_size) {
    uint8_t* response_packet;
    uint16_t checksum;
    uint16_t response_size_packet;
    uint8_t preamble_size = sizeof(response->preamble);

    // PREAMLE(3 bytes) + COMMAND_ID(1) + SIZE_OF_PAYLOAD(2) + STATUS_CODE(2) + PAYLOAD + CHECKSUM(2)
    response_size_packet = preamble_size + 1 + 2 + 2 + response->size_of_payload + 2;

    response_packet = (uint8_t*)malloc(response_size_packet);
    if (!response_packet) {
        *response_size = 0;
        
        // Free memory allocated for payload
        if (response->size_of_payload) {
            free(response->payload);
        }
        
        return NULL;
    }

    // Assemble the packet header
    for (int i = 0; i < preamble_size; ++i) {
        response_packet[i] = response->preamble[i];
    }

    response_packet[preamble_size]     = response->command_id;          // Command ID

    // packet size = payload size + status code size
    response_packet[preamble_size + 1] = lo(response->size_of_payload + 0x02); // Packet size low byte
    response_packet[preamble_size + 2] = hi(response->size_of_payload + 0x02); // Packet size high byte

    response_packet[preamble_size + 3] = lo(response->status_code);     // Status code low byte
    response_packet[preamble_size + 4] = hi(response->status_code);     // Status code high byte

    // Calculate the checksum
    // PREAMLE(3 bytes) + COMMAND_ID(1) + SIZE_OF_PAYLOAD(2) + STATUS_CODE(2)
    checksum = checksum_update_crc16(response_packet, preamble_size + 5, 0xFFFF);

    // Copy payload to buffer
    if (response->size_of_payload) {
        memcpy(&response_packet[preamble_size + 5], response->payload, response->size_of_payload);

        // Calculate the checksum
        // add PAYLOAD
        checksum = checksum_update_crc16(response->payload, response->size_of_payload, checksum);
        
        // Free memory allocated for payload
        free(response->payload);
    }

    // Add checksum to packet 
    response_packet[preamble_size + 5 + response->size_of_payload] = lo(checksum); // Checksum low byte
    response_packet[preamble_size + 6 + response->size_of_payload] = hi(checksum); // Checksum high byte

    *response_size = response_size_packet;
    
    return response_packet;
} 

void print_packet_data(const char* prefix, int count_of_bytes, uint8_t* packet, NET_DIR dir,
    std::string command, std::string status) {

    printf("[%s]: %sPacket size: %d bytes\n", LOG_PREFIX, prefix, count_of_bytes);
    printf("[%s]: %sPacket: %02X %02X %02X %02X %02X %02X ",
        LOG_PREFIX,
        prefix,
        packet[0], packet[1], packet[2], // PREAMBLE
        packet[3],                       // COMMAND_ID
        packet[4], packet[5]             // SIZE_OF_PAYLOAD
    );

    if (dir == RESPONSE) {
        printf("%02X %02X ", packet[6], packet[7]); // STATUS_CODE
    }

    uint16_t payload_size = (packet[5] << 8) | packet[4];

    if (dir == RESPONSE) {
        payload_size = payload_size - 0x02; // subtract size of status code
    }

    for (int i = 0; i < payload_size; ++i) {
        if (dir == REQUEST) {
            printf("%02X ", packet[6 + i]); // PAYLOAD
        } else if (dir == RESPONSE) {
            printf("%02X ", packet[8 + i]); // PAYLOAD
        }
    }

    if (dir == REQUEST) {
        printf("%02X %02X\n", packet[6 + payload_size], packet[6 + payload_size + 1]); // CHECKSUM
    } else if (dir == RESPONSE) {
        printf("%02X %02X\n", packet[8 + payload_size], packet[8 + payload_size + 1]); // CHECKSUM
    }

    printf("[%s]: %sParsed packet: \n\n", LOG_PREFIX, prefix);

    printf("PREAMBLE: %02X %02X %02X\n", packet[0], packet[1], packet[2]);
    if (dir == REQUEST) {
        printf("COMMAND ID: %02X\n", packet[3]);
    } else if (dir == RESPONSE) {
        printf("COMMAND ID: %02X (%s)\n", packet[3], command.c_str());
    }
    printf("SIZE OF PAYLOAD: %02X %02X\n", packet[4], packet[5]);
    
    if (dir == RESPONSE) {
        printf("STATUS CODE: %02X %02X (%s)\n", packet[6], packet[7], status.c_str());
    }
    
    printf("PAYLOAD: ");
    
    for (int i = 0; i < payload_size; ++i) {
        if (dir == REQUEST) {
            printf("%02X ", packet[6 + i]);
        } else if (dir == RESPONSE) {
            printf("%02X ", packet[8 + i]);
        }
    }

    if (dir == REQUEST) {
        printf("\nCHECKSUM: %02X %02X\n\n", packet[6 + payload_size], packet[6 + payload_size + 1]);
    } else if (dir == RESPONSE) {
        printf("\nCHECKSUM: %02X %02X\n\n", packet[8 + payload_size], packet[8 + payload_size + 1]);
    }
}

uint8_t* insufficient_resourses_response_packet_build(uint8_t command_id) {
    // Command ID
    insufficient_resourses_response_packet[3] = command_id;
    
    // Calculate the checksum
    uint16_t checksum = checksum_update_crc16(insufficient_resourses_response_packet, 8, 0xFFFF);

    printf("checksum: %X\n", checksum);
    // Add checksum to packet
    insufficient_resourses_response_packet[8] = lo(checksum); // Checksum low byte
    insufficient_resourses_response_packet[9] = hi(checksum); // Checksum high byte

    return insufficient_resourses_response_packet;
}

/*********************************************************************/ 
/*! 
Command 0x07
Announce the disconnection of the current interface.
Any finger movement that is executed when the disconnect announcement
arrives is aborted immediately. By sending this command before
closing the connection, the gripper will not enter FAST STOP on disconnect. 
(page 14 of WSG command set reference manual v4.0.x)

@param *request Pointer to request structure

@return response structure
*/ 
/*********************************************************************/ 
TRESPONSE disconnect_announcement(TREQUEST* request) {
    // E_NOT_AVAILABLE is not implemented
    // as TCP/IP interface is supposed to be used

    // Response

    TRESPONSE response;

    response.command_id = DisconnectAnnouncement;
    response.size_of_payload = 0;

    if (request->size_of_payload != 0) {
        response.status_code = E_NO_PARAM_EXPECTED;
        response.status = "E_NO_PARAM_EXPECTED";
    } else {
        response.status_code = E_SUCCESS;
        response.status = "E_SUCCESS";
    }

    return response;
}

/*********************************************************************/ 
/*! 
Command 0x20
Execute a homing sequence to reference the gripper fingers.
(page 15-16 of WSG command set reference manual v4.0.x)

@param *request Pointer to request structure

@return response structure
*/ 
/*********************************************************************/ 
TRESPONSE homing(TREQUEST* request) {
    
    // Request

    /** Homing direction
     * 0 - use default value from system configuration
     * 1 - homing in positive movement direction (external)
     * 2 - homing in negative movement direction (internal)
     **/
    uint8_t dir;
    if (request->size_of_payload == 1) {
        dir = request->payload[0];
    }

    // Response

    TRESPONSE response;

    response.command_id = Homing;
    response.size_of_payload = 0;

    if (SF_FAST_STOP) {
        response.status_code = E_ACCESS_DENIED;
        response.status = "E_ACCESS_DENIED";
    } else if (SF_MOVING) {
        response.status_code = E_ALREADY_RUNNING;
        response.status = "E_ALREADY_RUNNING";
    } else if (request->size_of_payload != 1) {
        response.status_code = E_CMD_FORMAT_ERROR;
        response.status = "E_CMD_FORMAT_ERROR";
    } else if (dir != 0x00 && dir != 0x01 && dir != 0x02) {
        response.status_code = E_INVALID_PARAMETER;
        response.status = "E_INVALID_PARAMETER";
    } else {
        response.status_code = E_CMD_PENDING;
        response.status = "E_CMD_PENDING";
    }

    return response;
}

/*********************************************************************/ 
/*! 
Command 0x50
Get information about the connected gripping module.
(page 41 of WSG command set reference manual v4.0.x)

@param *request Pointer to request structure

@return response structure
*/ 
/*********************************************************************/ 
TRESPONSE get_system_information(TREQUEST* request) {

    // Data
    
    uint8_t  TYPE       = SYSTEM_TYPE;
    uint8_t  HWREV      = HARDWARE_RIVISION;
    uint16_t FW_VERSION =
        FIRMWARE_VERSION_MAJOR << 12 |
        FIRMWARE_VERSION_MINOR << 8  |
        FIRMWARE_VERSION_PATH  << 4  | 0b0000;
    uint32_t SN         = SERIAL_NUMBER;
    
    // Response

    TRESPONSE response;

    response.command_id = GetSystemInformation;
    response.size_of_payload = 8;
    response.payload = (uint8_t *)malloc(response.size_of_payload);

    if (!response.payload) {
        response.size_of_payload = 0;
        response.status_code = E_INSUFFICIENT_RESOURCES;
        response.status = "E_INSUFFICIENT_RESOURCES";
        return response;
    }

    response.payload[0] = TYPE;
    response.payload[1] = HWREV;
    response.payload[2] = FW_VERSION >> 8;
    response.payload[3] = FW_VERSION;
    response.payload[4] = SN >> 3 * 8;
    response.payload[5] = SN >> 2 * 8;
    response.payload[6] = SN >> 8;
    response.payload[7] = SN;

    if (request->size_of_payload != 0) {
        response.status_code = E_NO_PARAM_EXPECTED;
        response.status = "E_NO_PARAM_EXPECTED";
    } else {
        response.status_code = E_SUCCESS;
        response.status = "E_SUCCESS";
    }

    return response;
}

/*********************************************************************/ 
/*! 
Command 0xB0
Custom command defined in Lua script (`cmd_mesure_speed_pos.lua`)

@param *request Pointer to request structure

@return response structure
*/ 
/*********************************************************************/ 
TRESPONSE measure(TREQUEST* request) {

    // Data
    
    uint32_t gripper_state =
        SF_SCRIPT_FAILURE     << 20 |
        SF_SCRIPT_RUNNING     << 19 |
        SF_CMD_FAILURE        << 18 |
        SF_FINGER_FAULT       << 17 |
        SF_CURR_FAULT         << 16 |
        SF_POWER_FAULT        << 15 |
        SF_TEMP_FAULT         << 14 |
        SF_TEMP_WARNING       << 13 |
        SF_FAST_STOP          << 12 |
        SF_FORCECNTL_MODE     << 9  |
        SF_OVERDRIVE_MODE     << 8  |
        SF_TARGET_POS_REACHED << 7  |
        SF_AXIS_STOPPED       << 6  |
        SF_SOFT_LIMIT_PLUS    << 5  |
        SF_SOFT_LIMIT_MINUS   << 4  |
        SF_BLOCKED_PLUS       << 3  |
        SF_BLOCKED_MINUS      << 2  |
        SF_MOVING             << 1  |
        SF_REFERENCED         << 0;

    bool mc_busy = SF_MOVING ? true : false;
    bool mc_blocked = (SF_BLOCKED_MINUS || SF_BLOCKED_PLUS) ? true : false;

    // float tmp;
    //   unsigned int src = 0;
    // src = b[3] * 16777216 + b[2] * 65536 + b[1] * 256 + b[0];

    // memcpy(&tmp, &src, sizeof tmp);

    uint32_t mc_position =
        request->payload[1] << 24 | request->payload[2] << 16 |
        request->payload[3] << 8  | request->payload[4];

    uint32_t mc_speed =
        request->payload[5] << 24 | request->payload[6] << 16 |
        request->payload[7] << 8  | request->payload[8];

    uint32_t mc_force_motor   = 0x00000000;
    uint32_t mc_force_finger0 = 0x00000000;
    uint32_t mc_force_finger1 = 0x00000000;

    // Response

    TRESPONSE response;

    response.command_id = Measure;
    // State (8) + Position (32) + Speed (32) + Force (32) + Force0 (32) + Force1 (32)
    response.size_of_payload = 8 + 4 * 8 + 4 * 8 + 4 * 8 + 4 * 8 + 4 * 8;
    response.payload = (uint8_t *)malloc(response.size_of_payload);

    if (!response.payload) {
        response.size_of_payload = 0;
        response.status_code = E_INSUFFICIENT_RESOURCES;
        response.status = "E_INSUFFICIENT_RESOURCES";
        return response;
    }

    response.payload[0] = (uint8_t)gripper_state;
    
    response.payload[1]  = mc_position;
    response.payload[2]  = mc_position >> 8;
    response.payload[3]  = mc_position >> 16;
    response.payload[4]  = mc_position >> 24;

    response.payload[5]  = mc_speed;
    response.payload[6]  = mc_speed >> 8;
    response.payload[7]  = mc_speed >> 16;
    response.payload[8]  = mc_speed >> 24;

    response.payload[9]  = mc_force_motor;
    response.payload[10] = mc_force_motor >> 8;
    response.payload[11] = mc_force_motor >> 16;
    response.payload[12] = mc_force_motor >> 24;

    response.payload[13] = mc_force_finger0;
    response.payload[14] = mc_force_finger0 >> 8;
    response.payload[15] = mc_force_finger0 >> 16;
    response.payload[16] = mc_force_finger0 >> 24;

    response.payload[17] = mc_force_finger1;
    response.payload[18] = mc_force_finger1 >> 8;
    response.payload[19] = mc_force_finger1 >> 16;
    response.payload[20] = mc_force_finger1 >> 24;

    response.status_code = E_SUCCESS;
    response.status = "E_SUCCESS";

    return response;
}