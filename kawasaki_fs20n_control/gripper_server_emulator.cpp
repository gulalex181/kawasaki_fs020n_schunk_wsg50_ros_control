// $ g++ gripper_server_emulator.cpp -o gripper_server_emulator

#include "gripper_server_emulator.h"

int main() {
    signal(SIGINT, SIGINT_handler);

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
                std::string command_name;
                PacketSequenceType packets_sequence = request_handling(request, &command_name);
                
                std::string response_prefix = "(response) ";

                for (int i = 0; i < packets_sequence.sequence_size; ++i) {
                    // Print packet
                    print_packet_data(
                        response_prefix.c_str(),
                        packets_sequence.packets_sizes[i],
                        packets_sequence.packets[i],
                        RESPONSE,
                        command_name,
                        packets_sequence.packets_statuses[i]
                    );

                    // Send packet to the client
                    if (write(client_socket_fd, packets_sequence.packets[i], packets_sequence.packets_sizes[i]) < 0) {
                        perror(("[" + std::string(LOG_PREFIX) + "]: writing to socker error").c_str());
                        exit(EXIT_FAILURE);
                    }

                    printf("[%s]: Response is sent\n", LOG_PREFIX);
                    printf("--------------------------------------------------\n");

                    // Free allocated memory for this packet
                    // response_packet = (uint8_t*)malloc(response_packet_size * sizeof(uint8_t));
                    if (packets_sequence.packets_free_need[i]) {
                        // free(packets_sequence.packets[i]);
                    }

                    // Sleep before next packet in the sequence
                    if (i < (packets_sequence.sequence_size - 1) && packets_sequence.delays[i]) {

                        sleep(packets_sequence.delays[i] / 1000); // s
                    }
                }

                // Free allocated memory
                // packets_sequence.packets = (uint8_t**)malloc(packets_sequence.sequence_size * sizeof(uint8_t*));
                // packets_sequence.packets_sizes = (uint16_t*)malloc(packets_sequence.sequence_size * sizeof(uint16_t));
                // packets_sequence.delays = (int*)malloc((packets_sequence.sequence_size - 1) * sizeof(int));
                // packets_sequence.packets_statuses = (std::string*)malloc(packets_sequence.sequence_size * sizeof(std::string));
                // packets_sequence.packets_free_need = (bool*)malloc(packets_sequence.sequence_size * sizeof(bool));
                // free(packets_sequence.packets);
                // free(packets_sequence.packets_sizes);
                // free(packets_sequence.delays);
                // free(packets_sequence.packets_statuses);
                // free(packets_sequence.packets_free_need);

                if (command_name == "DisconnectAnnouncement") {
                    break;
                }
            }
        }
    }
}

void SIGINT_handler(int sig) {
    close(server_socket_fd);
    exit(EXIT_SUCCESS);
}

PacketSequenceType request_handling(uint8_t* request_packet, std::string* command_name) {

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

    ResponseSequenceType response_sequence;

    switch (request.command_id) {
        // Command ID 0x07
        case DisconnectAnnouncement:
            response_sequence = disconnect_announcement(&request);
            *command_name = "DisconnectAnnouncement";
            break;
        // Command ID 0x20
        case Homing:
            response_sequence = homing(&request);
            *command_name = "Homing";
            break;
        // Command ID 0x50
        case GetSystemInformation:
            response_sequence = get_system_information(&request);
            *command_name = "GetSystemInformation";
            break;
        // Command ID 0x53
        case GetSystemLimits:
            response_sequence = get_system_limits(&request);
            *command_name = "GetSystemLimits";
            break;
        // Command ID 0xB0
        case Measure:
            response_sequence = measure(&request);
            *command_name = "Measure";
            break;
        // Command ID 0xB1
        case PositionControl:
            response_sequence = position_control(&request);
            *command_name = "PositionControl";
            break;
        // Command ID 0xB2
        case SpeedControl:
            response_sequence = speed_control(&request);
            *command_name = "SpeedControl";
            break;
        default:
            // error, no such command
            *command_name = "";
            break;
    }
    
    // Response packets building

    PacketSequenceType packets_sequence;

    packets_sequence.sequence_size = response_sequence.sequence_size;

    // Create array for packets
    packets_sequence.packets = (uint8_t**)malloc(packets_sequence.sequence_size * sizeof(uint8_t*));

    // Create array for packets sizes
    packets_sequence.packets_sizes = (uint16_t*)malloc(packets_sequence.sequence_size * sizeof(uint16_t));

    // Create array for delays between packets
    packets_sequence.delays = (int*)malloc((packets_sequence.sequence_size - 1) * sizeof(int));

    for (int i = 0; i < response_sequence.sequence_size - 1; ++i) {
        packets_sequence.delays[i] = response_sequence.delays[i];
    }

    // Create array for packets statuses
    packets_sequence.packets_statuses = (std::string*)malloc(packets_sequence.sequence_size * sizeof(std::string));

    // Create array for flags of need to free memory for packets
    packets_sequence.packets_free_need = (bool*)malloc(packets_sequence.sequence_size * sizeof(bool));
    
    // Build packets
    for (int i = 0; i < response_sequence.sequence_size; ++i) {
        uint16_t response_size;

        packets_sequence.packets[i] = response_packet_build(&response_sequence.responses[i], &response_size);
        packets_sequence.packets_sizes[i] = response_size;

        if (!packets_sequence.packets[i]) {
            packets_sequence.packets[i] =
                insufficient_resourses_response_packet_build(response_sequence.responses[i].command_id);

            packets_sequence.packets_sizes[i] = 10;
            packets_sequence.packets_statuses[i] = "E_INSUFFICIENT_RESOURCES";
            packets_sequence.packets_free_need[i] = false;

            continue;
        }

        packets_sequence.packets_statuses[i] = response_sequence.responses[i].status;
        packets_sequence.packets_free_need[i] = true;
    }

    // Free allocated memory
    // response_sequence.responses = (TRESPONSE*)malloc(response_sequence.sequence_size * sizeof(TRESPONSE));
    // response_sequence.delays    = (int*)malloc((response_sequence.sequence_size - 1) * sizeof(int));
    // free(response_sequence.responses);

    if (packets_sequence.sequence_size - 1) {
        // free(response_sequence.delays);
    }

    return packets_sequence;
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
uint8_t* response_packet_build(TRESPONSE* response, uint16_t* response_size) {
    uint8_t* response_packet;
    uint16_t checksum;
    uint16_t response_packet_size;
    uint8_t preamble_size = sizeof(response->preamble);

    // PREAMLE(3 bytes) + COMMAND_ID(1) + SIZE_OF_PAYLOAD(2) + STATUS_CODE(2) + PAYLOAD + CHECKSUM(2)
    response_packet_size = preamble_size + 1 + 2 + 2 + response->size_of_payload + 2;

    response_packet = (uint8_t*)malloc(response_packet_size * sizeof(uint8_t));

    if (!response_packet) {
        *response_size = 0;

        // Free memory allocated for payload
        // response.payload = (uint8_t*)malloc(response.size_of_payload * sizeof(uint8_t));
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
        // response.payload = (uint8_t*)malloc(response.size_of_payload * sizeof(uint8_t));
        free(response->payload);
    }

    // Add checksum to packet 
    response_packet[preamble_size + 5 + response->size_of_payload] = lo(checksum); // Checksum low byte
    response_packet[preamble_size + 6 + response->size_of_payload] = hi(checksum); // Checksum high byte

    *response_size = response_packet_size;

    return response_packet;
}

void print_packet_data(const char* prefix, uint16_t packet_size, uint8_t* packet, NET_DIR dir,
    std::string command, std::string status) {

    printf("[%s]: %sPacket size: %d bytes\n", LOG_PREFIX, prefix, packet_size);
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

    if (dir == REQUEST) {
        printf("SIZE OF PAYLOAD: %02X %02X [bytes: %d (payload), %d (payload + status code)]\n",
            packet[4],
            packet[5],
            (packet[5] << 8) | packet[4],
            (packet[5] << 8) | packet[4] + 2
        );
    } else if (dir == RESPONSE) {
        printf("SIZE OF PAYLOAD: %02X %02X [bytes: %d (payload), %d (payload + status code)]\n",
            packet[4],
            packet[5],
            (packet[5] << 8) | packet[4] - 2,
            (packet[5] << 8) | packet[4]
        );
    }

    if (dir == RESPONSE) {
        printf("STATUS CODE: %02X %02X (%s)\n", packet[6], packet[7], status.c_str());
    }
    
    printf("PAYLOAD: ");
    
    uint8_t payload[payload_size];

    for (int i = 0; i < payload_size; ++i) {
        if (dir == REQUEST) {
            payload[i] = packet[6 + i];
            printf("%02X ", packet[6 + i]);
        } else if (dir == RESPONSE) {
            payload[i] = packet[8 + i];
            printf("%02X ", packet[8 + i]);
        }
    }

    if (packet[3] == 0xB0 || packet[3] == 0xB1 || packet[3] == 0xB2) {
        if (dir == REQUEST) {
            uint32_t _request_position = payload[4] << 24 | payload[3] << 16 | payload[2] << 8 | payload[1];
            uint32_t _request_speed    = payload[8] << 24 | payload[7] << 16 | payload[6] << 8 | payload[5];

            float request_position, request_speed;

            memcpy(&request_position, &_request_position, sizeof(float));
            memcpy(&request_speed,    &_request_speed,    sizeof(float));
            
            printf("\n(pos: %5.1f mm, spd: %5.1f mm/s)", request_position, request_speed);
        } else if (dir == RESPONSE) {
            printf("\n(pos: %5.1f mm, spd: %5.1f mm/s, frc: %5.1f N, frc0: %5.1f N, frc1: %5.1f N)",
                gripper_current_position,
                gripper_current_speed,
                gripper_current_force_motor,
                gripper_current_force_finger0,
                gripper_current_force_finger1
            );
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

@return response sequence structure
*/ 
/*********************************************************************/ 
ResponseSequenceType disconnect_announcement(TREQUEST* request) {
    // E_NOT_AVAILABLE is not implemented
    // as TCP/IP interface is supposed to be used

    // Responses
    
    ResponseSequenceType response_sequence;

    response_sequence.sequence_size = 1;
    response_sequence.responses = (TRESPONSE*)malloc(response_sequence.sequence_size * sizeof(TRESPONSE));

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

    response_sequence.responses[0] = response;

    return response_sequence;
}

/*********************************************************************/ 
/*! 
Command 0x20
Execute a homing sequence to reference the gripper fingers.
(page 15-16 of WSG command set reference manual v4.0.x)

@param *request Pointer to request structure

@return response sequence structure
*/ 
/*********************************************************************/ 
ResponseSequenceType homing(TREQUEST* request) {
    
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

    // Responses

    ResponseSequenceType response_sequence;

    response_sequence.sequence_size  = 2;
    response_sequence.responses = (TRESPONSE*)malloc(response_sequence.sequence_size * sizeof(TRESPONSE));
    response_sequence.delays    = (int*)malloc((response_sequence.sequence_size - 1) * sizeof(int));

    // Immediate response

    TRESPONSE response_immediate;

    response_immediate.command_id = Homing;
    response_immediate.size_of_payload = 0;

    if (SF_FAST_STOP) {
        response_immediate.status_code = E_ACCESS_DENIED;
        response_immediate.status = "E_ACCESS_DENIED";
    } else if (SF_MOVING) {
        response_immediate.status_code = E_ALREADY_RUNNING;
        response_immediate.status = "E_ALREADY_RUNNING";
    } else if (request->size_of_payload != 1) {
        response_immediate.status_code = E_CMD_FORMAT_ERROR;
        response_immediate.status = "E_CMD_FORMAT_ERROR";
    } else if (dir != 0x00 && dir != 0x01 && dir != 0x02) {
        response_immediate.status_code = E_INVALID_PARAMETER;
        response_immediate.status = "E_INVALID_PARAMETER";
    } else {
        response_immediate.status_code = E_CMD_PENDING;
        response_immediate.status = "E_CMD_PENDING";
    }

    // Response on completion

    TRESPONSE response_completion;

    response_completion.command_id = Homing;
    response_completion.size_of_payload = 0;

    // E_TIMEOUT is not implemented as the emulator doesn't have this timeout
    if (SF_AXIS_STOPPED) {
        response_completion.status_code = E_CMD_ABORTED;
        response_completion.status = "E_CMD_ABORTED";
    } else if (SF_BLOCKED_MINUS || SF_BLOCKED_PLUS) {
        response_completion.status_code = E_AXIS_BLOCKED;
        response_completion.status = "E_AXIS_BLOCKED";
    } else {
        response_completion.status_code = E_SUCCESS;
        response_completion.status = "E_SUCCESS";
    }

    response_sequence.responses[0] = response_immediate;
    response_sequence.responses[1] = response_completion;

    response_sequence.delays[0] = HOMING_DELAY; // ms

    // Set system flag that fingers are referenced
    SF_REFERENCED = 1;

    return response_sequence;
}

/*********************************************************************/ 
/*! 
Command 0x50
Get information about the connected gripping module.
(page 41 of WSG command set reference manual v4.0.x)

@param *request Pointer to request structure

@return response sequence structure
*/ 
/*********************************************************************/ 
ResponseSequenceType get_system_information(TREQUEST* request) {

    // Data
    
    uint8_t  TYPE       = SYSTEM_TYPE;
    uint8_t  HWREV      = HARDWARE_RIVISION;
    uint16_t FW_VERSION =
        FIRMWARE_VERSION_MAJOR << 12 |
        FIRMWARE_VERSION_MINOR << 8  |
        FIRMWARE_VERSION_PATH  << 4  | 0b0000;
    uint32_t SN         = SERIAL_NUMBER;
    
    // Responses

    ResponseSequenceType response_sequence;

    response_sequence.sequence_size = 1;
    response_sequence.responses = (TRESPONSE*)malloc(response_sequence.sequence_size * sizeof(TRESPONSE));

    // Response

    TRESPONSE response;

    response.command_id = GetSystemInformation;
    response.size_of_payload = 8;
    response.payload = (uint8_t*)malloc(response.size_of_payload * sizeof(uint8_t));

    if (!response.payload) {
        response.size_of_payload = 0;
        response.status_code = E_INSUFFICIENT_RESOURCES;
        response.status = "E_INSUFFICIENT_RESOURCES";
        
        response_sequence.responses[0] = response;

        return response_sequence;
    }

    response.payload[0] = TYPE;
    response.payload[1] = HWREV;
    response.payload[2] = (uint8_t)(FW_VERSION >> 8);
    response.payload[3] = (uint8_t)(FW_VERSION);
    response.payload[4] = (uint8_t)(SN >> 3 * 8);
    response.payload[5] = (uint8_t)(SN >> 2 * 8);
    response.payload[6] = (uint8_t)(SN >> 8);
    response.payload[7] = (uint8_t)(SN);

    if (request->size_of_payload != 0) {
        response.status_code = E_NO_PARAM_EXPECTED;
        response.status = "E_NO_PARAM_EXPECTED";
    } else {
        response.status_code = E_SUCCESS;
        response.status = "E_SUCCESS";
    }

    response_sequence.responses[0] = response;

    return response_sequence;
}

/*********************************************************************/ 
/*! 
Command 0x53
Get the gripper's physical limits for stroke, speed,
acceleration and force.
(page 43 of WSG command set reference manual v4.0.x)

@param *request Pointer to request structure

@return response sequence structure
*/ 
/*********************************************************************/ 
ResponseSequenceType get_system_limits(TREQUEST* request) {

    // Data
    
    uint32_t stroke    = STROKE;
    uint32_t min_speed = MIN_SPEED;
    uint32_t max_speed = MAX_SPEED;
    uint32_t min_acc   = MIN_ACC;
    uint32_t max_acc   = MAX_ACC;
    uint32_t min_force = MIN_FORCE;
    uint32_t nom_force = NOM_FORCE;
    uint32_t ovr_force = OVR_FORCE;
    
    // Responses

    ResponseSequenceType response_sequence;

    response_sequence.sequence_size = 1;
    response_sequence.responses = (TRESPONSE*)malloc(response_sequence.sequence_size * sizeof(TRESPONSE));

    // Response

    TRESPONSE response;

    response.command_id = GetSystemLimits;
    response.size_of_payload = 32;
    response.payload = (uint8_t*)malloc(response.size_of_payload * sizeof(uint8_t));

    if (!response.payload) {
        response.size_of_payload = 0;
        response.status_code = E_INSUFFICIENT_RESOURCES;
        response.status = "E_INSUFFICIENT_RESOURCES";
        
        response_sequence.responses[0] = response;

        return response_sequence;
    }

    response.payload[0]  = (uint8_t)(stroke >> 24);
    response.payload[1]  = (uint8_t)(stroke >> 16);
    response.payload[2]  = (uint8_t)(stroke >> 8);
    response.payload[3]  = (uint8_t)(stroke);

    response.payload[4]  = (uint8_t)(min_speed >> 24);
    response.payload[5]  = (uint8_t)(min_speed >> 16);
    response.payload[6]  = (uint8_t)(min_speed >> 8);
    response.payload[7]  = (uint8_t)(min_speed);

    response.payload[8]  = (uint8_t)(max_speed >> 24);
    response.payload[9]  = (uint8_t)(max_speed >> 16);
    response.payload[10] = (uint8_t)(max_speed >> 8);
    response.payload[11] = (uint8_t)(max_speed);

    response.payload[12] = (uint8_t)(min_acc >> 24);
    response.payload[13] = (uint8_t)(min_acc >> 16);
    response.payload[14] = (uint8_t)(min_acc >> 8);
    response.payload[15] = (uint8_t)(min_acc);

    response.payload[16] = (uint8_t)(max_acc >> 24);
    response.payload[17] = (uint8_t)(max_acc >> 16);
    response.payload[18] = (uint8_t)(max_acc >> 8);
    response.payload[19] = (uint8_t)(max_acc);

    response.payload[20] = (uint8_t)(min_force >> 24);
    response.payload[21] = (uint8_t)(min_force >> 16);
    response.payload[22] = (uint8_t)(min_force >> 8);
    response.payload[23] = (uint8_t)(min_force);

    response.payload[24] = (uint8_t)(nom_force >> 24);
    response.payload[25] = (uint8_t)(nom_force >> 16);
    response.payload[26] = (uint8_t)(nom_force >> 8);
    response.payload[27] = (uint8_t)(nom_force);

    response.payload[28] = (uint8_t)(ovr_force >> 24);
    response.payload[29] = (uint8_t)(ovr_force >> 16);
    response.payload[30] = (uint8_t)(ovr_force >> 8);
    response.payload[31] = (uint8_t)(ovr_force);

    if (request->size_of_payload != 0) {
        response.status_code = E_NO_PARAM_EXPECTED;
        response.status = "E_NO_PARAM_EXPECTED";
    } else {
        response.status_code = E_SUCCESS;
        response.status = "E_SUCCESS";
    }
printf("1\n");
    response_sequence.responses[0] = response;
printf("2\n");
    return response_sequence;
}

/*********************************************************************/ 
/*! 
Command 0xB0
Custom command defined in Lua script (`cmd_mesure_speed_pos.lua`)

@param *request Pointer to request structure

@return response sequence structure
*/ 
/*********************************************************************/ 
ResponseSequenceType measure(TREQUEST* request) {

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

    // Responses

    ResponseSequenceType response_sequence;

    response_sequence.sequence_size = 1;
    response_sequence.responses = (TRESPONSE*)malloc(response_sequence.sequence_size * sizeof(TRESPONSE));

    // Response

    TRESPONSE response;

    response.command_id = Measure;
    // State (1 byte) + Position (4) + Speed (4) + Force (4) + Force0 (4) + Force1 (4)
    response.size_of_payload = 1 + 4 + 4 + 4 + 4 + 4;
    response.payload = (uint8_t*)malloc(response.size_of_payload * sizeof(uint8_t));

    if (!response.payload) {
        response.size_of_payload = 0;
        response.status_code = E_INSUFFICIENT_RESOURCES;
        response.status = "E_INSUFFICIENT_RESOURCES";
        
        response_sequence.responses[0] = response;

        return response_sequence;
    }

    response.payload[0]  = (uint8_t)gripper_state;

    memcpy(&response.payload[1],  &gripper_current_position,      sizeof(float));
    memcpy(&response.payload[5],  &gripper_current_speed,         sizeof(float));
    memcpy(&response.payload[9],  &gripper_current_force_motor,   sizeof(float));
    memcpy(&response.payload[17], &gripper_current_force_finger0, sizeof(float));
    memcpy(&response.payload[17], &gripper_current_force_finger1, sizeof(float));

    response.status_code = E_SUCCESS;
    response.status = "E_SUCCESS";

    response_sequence.responses[0] = response;

    return response_sequence;
}

/*********************************************************************/ 
/*! 
Command 0xB1
Custom command defined in Lua script (`cmd_mesure_speed_pos.lua`)

@param *request Pointer to request structure

@return response sequence structure
*/ 
/*********************************************************************/ 
ResponseSequenceType position_control(TREQUEST* request) {

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

    // little-endian format
    uint32_t _gripper_current_position = 
        request->payload[4] << 24 | request->payload[3] << 16 |
        request->payload[2] << 8  | request->payload[1];

    // little-endian format
    uint32_t _gripper_current_speed = 
        request->payload[8] << 24 | request->payload[7] << 16 |
        request->payload[6] << 8  | request->payload[5];

    // Save updated current state
    memcpy(&gripper_current_position, &_gripper_current_position, sizeof(float));
    memcpy(&gripper_current_speed,    &_gripper_current_speed,    sizeof(float));

    // Responses

    ResponseSequenceType response_sequence;

    response_sequence.sequence_size = 1;
    response_sequence.responses = (TRESPONSE*)malloc(response_sequence.sequence_size * sizeof(TRESPONSE));

    // Response

    TRESPONSE response;

    response.command_id = PositionControl;
    // State (1 byte) + Position (4) + Speed (4) + Force (4) + Force0 (4) + Force1 (4)
    response.size_of_payload = 1 + 4 + 4 + 4 + 4 + 4;
    response.payload = (uint8_t*)malloc(response.size_of_payload * sizeof(uint8_t));

    if (!response.payload) {
        response.size_of_payload = 0;
        response.status_code = E_INSUFFICIENT_RESOURCES;
        response.status = "E_INSUFFICIENT_RESOURCES";
        
        response_sequence.responses[0] = response;

        return response_sequence;
    }

    response.payload[0]  = (uint8_t)gripper_state;
    
    memcpy(&response.payload[1],  &gripper_current_position,      sizeof(float));
    memcpy(&response.payload[5],  &gripper_current_speed,         sizeof(float));
    memcpy(&response.payload[9],  &gripper_current_force_motor,   sizeof(float));
    memcpy(&response.payload[17], &gripper_current_force_finger0, sizeof(float));
    memcpy(&response.payload[17], &gripper_current_force_finger1, sizeof(float));

    response.status_code = E_SUCCESS;
    response.status = "E_SUCCESS";

    response_sequence.responses[0] = response;

    return response_sequence;
}

/*********************************************************************/ 
/*! 
Command 0xB2
Custom command defined in Lua script (`cmd_mesure_speed_pos.lua`)

@param *request Pointer to request structure

@return response sequence structure
*/ 
/*********************************************************************/ 
ResponseSequenceType speed_control(TREQUEST* request) {

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

    uint32_t mc_position = gripper_current_position;

    // little-endian format
    uint32_t _gripper_current_speed = 
        request->payload[8] << 24 | request->payload[7] << 16 |
        request->payload[6] << 8  | request->payload[5];

    memcpy(&gripper_current_speed, &_gripper_current_speed, sizeof(float));

    // Responses

    ResponseSequenceType response_sequence;

    response_sequence.sequence_size = 1;
    response_sequence.responses = (TRESPONSE*)malloc(response_sequence.sequence_size * sizeof(TRESPONSE));

    // Response

    TRESPONSE response;

    response.command_id = SpeedControl;
    // State (1 byte) + Position (4) + Speed (4) + Force (4) + Force0 (4) + Force1 (4)
    response.size_of_payload = 1 + 4 + 4 + 4 + 4 + 4;
    response.payload = (uint8_t*)malloc(response.size_of_payload * sizeof(uint8_t));

    if (!response.payload) {
        response.size_of_payload = 0;
        response.status_code = E_INSUFFICIENT_RESOURCES;
        response.status = "E_INSUFFICIENT_RESOURCES";
        
        response_sequence.responses[0] = response;

        return response_sequence;
    }

    response.payload[0]  = (uint8_t)gripper_state;
    
    memcpy(&response.payload[1],  &gripper_current_position,      sizeof(float));
    memcpy(&response.payload[5],  &gripper_current_speed,         sizeof(float));
    memcpy(&response.payload[9],  &gripper_current_force_motor,   sizeof(float));
    memcpy(&response.payload[17], &gripper_current_force_finger0, sizeof(float));
    memcpy(&response.payload[17], &gripper_current_force_finger1, sizeof(float));

    response.status_code = E_SUCCESS;
    response.status = "E_SUCCESS";

    response_sequence.responses[0] = response;

    return response_sequence;
}