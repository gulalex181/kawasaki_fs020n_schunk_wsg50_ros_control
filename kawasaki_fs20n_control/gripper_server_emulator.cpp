// $ g++ gripper_server_emulator.cpp -o gripper_server_emulator

#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <errno.h>
#include <string.h>
#include <sstream>
#include <vector>
#include <signal.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <byteswap.h>

typedef struct {
    uint8_t preamble[3] = {0xAA, 0xAA, 0xAA};
    uint8_t command_id;
    uint16_t size_of_payload;
    uint16_t status_code;
    uint8_t *payload;
} TRESPONSE;

//! Status codes
typedef enum {
    E_SUCCESS = 0,            //!< No error
    E_NOT_AVAILABLE,          //!< Device, service or data is not available
    E_NO_SENSOR,              //!< No sensor connected
    E_NOT_INITIALIZED,        //!< The device is not initialized
    E_ALREADY_RUNNING,        //!< Service is already running
    E_FEATURE_NOT_SUPPORTED,  //!< The asked feature is not supported
    E_INCONSISTENT_DATA,      //!< One or more dependent parameters mismatch
    E_TIMEOUT,                //!< Timeout error
    E_READ_ERROR,             //!< Error while reading from a device
    E_WRITE_ERROR,            //!< Error while writing to a device
    E_INSUFFICIENT_RESOURCES, //!< No memory available
    E_CHECKSUM_ERROR,         //!< Checksum error
    E_NO_PARAM_EXPECTED,      //!< No parameters expected
    E_NOT_ENOUGH_PARAMS,      //!< Not enough parameters
    E_CMD_UNKNOWN,            //!< Unknown command
    E_CMD_FORMAT_ERROR,       //!< Command format error
    E_ACCESS_DENIED,          //!< Access denied
    E_ALREADY_OPEN,           //!< The interface is already open
    E_CMD_FAILED,             //!< Command failed
    E_CMD_ABORTED,            //!< Command aborted
    E_INVALID_HANDLE,         //!< invalid handle
    E_NOT_FOUND,              //!< device not found
    E_NOT_OPEN,               //!< device not open
    E_IO_ERROR,               //!< I/O error
    E_INVALID_PARAMETER,      //!< invalid parameter
    E_INDEX_OUT_OF_BOUNDS,    //!< index out of bounds
    E_CMD_PENDING,            //!< Command execution needs more time
    E_OVERRUN,                //!< Data overrun
    E_RANGE_ERROR,            //!< Range error
    E_AXIS_BLOCKED,           //!< Axis is blocked
    E_FILE_EXISTS             //!< File already exists
} TStat;

typedef enum {
    UNKNOWN = 0, // unknown
    WSG_50,      // WSG 50
    WSG_32,      // WSG 32
    KMS_40,      // Force-Torque Sensor KMS 40
    WTS,         // Tactile Sensing Module WTS
    WSG_25,      // WSG 25
    WSG_70       // WSG 70
} TYPE;

typedef enum {
    REQUEST = 0,
    RESPONSE
} NET_DIR;

#define LOG_PREFIX             "GRIPPER EMULATOR SERVER"
#define BUFFER_SIZE            1024     //!< buffer size
#define PORT                   5222     //!< port
#define SYSTEM_TYPE            WSG_50   //!< hardware rivision
#define HARDWARE_RIVISION      5        //!< hardware rivision
#define SERIAL_NUMBER          00003713 //!< serial number
#define FIRMWARE_VERSION_MAJOR 3        //!< firmware major version
#define FIRMWARE_VERSION_MINOR 0        //!< firmware minor version
#define FIRMWARE_VERSION_PATH  0        //!< firmware path version

const unsigned short CRC_TABLE[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
    0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
    0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
    0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
    0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
    0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
    0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
    0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
    0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
    0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
    0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
    0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
    0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
    0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
    0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
    0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
    0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
    0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
    0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
    0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
    0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
    0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
    0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
};

// Byte access
#define hi(x)    (unsigned char) ( ((x) >> 8) & 0xff ) // Returns the upper byte of the passed short
#define lo(x)    (unsigned char) ( (x) & 0xff )        // Returns the lower byte of the passed short

void CtrlC_handler(int sig);
void print_packet_data(const char* prefix, int count_of_bytes, uint8_t* packet, NET_DIR dir);
uint8_t* request_handling(uint8_t* request, uint16_t* response_size);
uint16_t checksum_update_crc16(uint8_t* data, u_int16_t size, uint16_t crc);
uint8_t* msg_build(TRESPONSE *response, uint16_t* size);
// TStat msg_send(FILE *file, TMESSAGE *msg);

// Commands
TRESPONSE get_system_information(uint8_t* request_payload, uint16_t request_payload_size);

uint8_t buffer[BUFFER_SIZE] = {0};

int server_socket_fd = 0;
int client_socket_fd = 0;

struct sockaddr_in serverAddr;

int main() {
    signal(SIGINT, CtrlC_handler);

    if ((server_socket_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        perror("[GRIPPER EMULATOR SERVER]: socket failed");
        exit(EXIT_FAILURE);
    }

    serverAddr.sin_addr.s_addr = INADDR_ANY;
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(PORT);

    if (bind(server_socket_fd, (struct sockaddr*) &serverAddr, sizeof(serverAddr)) < 0) {
        perror("[GRIPPER EMULATOR SERVER]: bind failed");
        exit(EXIT_FAILURE);
    }

    if (listen(server_socket_fd, 0) < 0) {
        perror("[GRIPPER EMULATOR SERVER]: listen failed");
        exit(EXIT_FAILURE);
    }

    while (1) {
        std::cout << "[GRIPPER EMULATOR SERVER]: Listening for incoming connections" << std::endl;

        client_socket_fd = accept(server_socket_fd, NULL, NULL);
        if (client_socket_fd == -1) {
            perror("[GRIPPER EMULATOR SERVER]: error when accepting connection");
            exit(EXIT_FAILURE);
        } else {
            std::cout << "[GRIPPER EMULATOR SERVER]: Client is connected, fd " << client_socket_fd << std::endl;

            while (1) {
                memset(buffer, 0, BUFFER_SIZE);

                int count_of_bytes = read(client_socket_fd, buffer, sizeof(buffer) - 1);

                // if (count_of_bytes < 0) {
                //     perror("[GRIPPER EMULATOR SERVER]: reading from socker error");
                //     exit(EXIT_FAILURE);
                // }

                if (count_of_bytes <= 0) continue;

                std::string request_prefix = "(request) ";
                print_packet_data(request_prefix.c_str(), count_of_bytes, buffer, REQUEST);

                // Request handling
                // std::string response = request_handling(buffer);
                uint16_t response_size;
                uint8_t* response = request_handling(buffer, &response_size);
                
                if (!response) {
                    
                }

                std::string response_prefix = "(response) ";
                print_packet_data(response_prefix.c_str(), response_size, response, RESPONSE);

                // // Close connection
                // if (response == "client is disconnected") {
                //     break;
                // }

                // std::cout << "[GRIPPER EMULATOR SERVER]: Response: '" << response << "'" << std::endl;
                
                if (write(client_socket_fd, response, sizeof(response)) < 0) {
                    perror("[GRIPPER EMULATOR SERVER]: writing to socker error");
                    exit(EXIT_FAILURE);
                }

                std::cout << "[GRIPPER EMULATOR SERVER]: Response is sent" << std::endl << std::endl;

                // Clear the memory allocated for response packet
                free(response);
            }
        }
    }
}

void CtrlC_handler(int sig) {
    close(server_socket_fd);
    exit(EXIT_SUCCESS);
}

uint8_t* request_handling(uint8_t* request, uint16_t* response_size) {

    uint8_t  command_id   = request[3];
    uint16_t payload_size = (request[5] << 8) | request[4];
    uint8_t  payload[payload_size];
    
    for (int i = 0; i < payload_size; ++i) {
        payload[i] = request[i];
    }

    uint16_t checksum = (request[payload_size + 1] << 8) | request[payload_size];

    // PREAMLE(3 bytes) + COMMAND_ID(1) + SIZE_OF_PAYLOAD(2) + PAYLOAD + CHECKSUM(2)
    uint16_t request_checksum = checksum_update_crc16(request, 3 + 1 + 2 + payload_size + 2, 0xFFFF);
    
    if (request_checksum == 0) {
        std::cout << "[GRIPPER EMULATOR SERVER]: CRC Checksum is " << request_checksum << ", OK" << std::endl << std::endl;
    } else {
        std::cout << "[GRIPPER EMULATOR SERVER]: CRC Checksum is not 0, ERROR" << std::endl << std::endl;
    }

    TRESPONSE response = get_system_information(payload, payload_size);
 
    // for (int i = 0; i < response.size_of_payload; ++i) {
    //     printf("%X ", response.payload[i]);
    // }

    response.command_id = command_id;

    if (response.status_code != E_SUCCESS) {
        // ...
    }

    return msg_build(&response, response_size);
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
uint8_t* msg_build(TRESPONSE *response, uint16_t* response_size) {
    uint8_t* response_packet;
    uint16_t chksum;
    uint16_t response_size_packet;
    uint8_t preamble_size = sizeof(response->preamble);

    // PREAMLE(3 bytes) + COMMAND_ID(1) + SIZE_OF_PAYLOAD(2) + STATUS_CODE(2) + PAYLOAD + CHECKSUM(2)
    response_size_packet = preamble_size + 1 + 2 + 2 + response->size_of_payload + 2;

    response_packet = (uint8_t*)malloc(response_size_packet);
    if (!response_packet) {
        *response_size = 0;
        free(response->payload);
        return NULL;
    }

    // Assemble the message header:
    for (int i = 0; i < preamble_size; ++i) {
        response_packet[i] = response->preamble[i];
    }

    response_packet[preamble_size]     = response->command_id;          // Command ID

    response_packet[preamble_size + 1] = lo(response->size_of_payload); // Packet size low byte
    response_packet[preamble_size + 2] = hi(response->size_of_payload); // Packet size high byte

    response_packet[preamble_size + 3] = lo(response->status_code);     // Status code low byte
    response_packet[preamble_size + 4] = hi(response->status_code);     // Status code high byte

    // Copy payload to buffer:
    if (response->size_of_payload) {
        memcpy(&response_packet[preamble_size + 5], response->payload, response->size_of_payload);
        free(response->payload);
    }

    // Calculate the checksum over the header, include the preamble: 
    chksum = checksum_update_crc16(response_packet, preamble_size + 5 + response->size_of_payload, 0xFFFF);

    // Add checksum to message: 
    response_packet[preamble_size + 5 + response->size_of_payload] = lo(chksum); // Checksum low byte
    response_packet[preamble_size + 6 + response->size_of_payload] = hi(chksum); // Checksum high byte

    *response_size = response_size_packet;
    
    return response_packet;
} 


/*********************************************************************/ 
/*! 
Send a message to an open file handle 

@param *file Handle of an open file to which the message should be sent 
@param *msg Pointer to the message that should be sent 

@return E_SUCCESS, if successful, otherwise error code 
*/ 
/*********************************************************************/ 
// TStat msg_send(FILE *file, TMESSAGE *msg) {
//     unsigned int c;
//     uint16_t size;

//     // Convert message into byte sequence:
//     unsigned char *buf = msg_build(msg, &size);
//     if (!buf) {
//         return E_INSUFFICIENT_RESOURCES;
//     }

//     // Transmit buffer:
//     c = fwrite(buf, size, 1, file);

//     // Free allocated memory:
//     free(buf);
//     if (c != 1) {
//         return E_WRITE_ERROR;
//     }

//     return E_SUCCESS;
// }

void print_packet_data(const char* prefix, int count_of_bytes, uint8_t* packet, NET_DIR dir) {
    printf("[%s]: %sPacket size: %d bytes\n", LOG_PREFIX, prefix, count_of_bytes);
    printf("[%s]: %sPacket: %02X %02X %02X %02X %02X %02X ",
        LOG_PREFIX,
        prefix,
        packet[0], packet[1], packet[2], packet[3], packet[4], packet[5]
    );

    uint16_t payload_size = (packet[5] << 8) | packet[4];

    for (int i = 0; i < payload_size; ++i) {
        printf("%02X ", packet[i]);
    }

    printf("%02X %02X\n", packet[payload_size], packet[payload_size + 1]);

    printf("[%s]: %sParsed packet: \n\n", LOG_PREFIX, prefix);

    printf("PREAMBLE: %02X %02X %02X\n", packet[0], packet[1], packet[2]);
    printf("COMMAND ID: %02X\n", packet[3]);
    printf("SIZE OF PAYLOAD: %02X %02X\n", packet[4], packet[5]);
    
    if (dir == RESPONSE) {
        printf("STATUS CODE: %02X %02X\n", packet[6], packet[7]);
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

/*********************************************************************/ 
/*! 
Get information about the connected gripping module
(page 41 of WSG command set reference manual v4.0.x)

@param *file Handle of an open file to which the message should be sent 
@param *msg Pointer to the message that should be sent 

@return E_SUCCESS, if successful, otherwise error code 
*/ 
/*********************************************************************/ 
TRESPONSE get_system_information(uint8_t* request_payload, uint16_t request_payload_size) {
    // Data
    uint8_t  TYPE       = SYSTEM_TYPE;
    uint8_t  HWREV      = HARDWARE_RIVISION;
    uint16_t FW_VERSION =
        FIRMWARE_VERSION_MAJOR << 12 |
        FIRMWARE_VERSION_MINOR << 8  |
        FIRMWARE_VERSION_PATH  << 4  | 0b000;
    uint32_t SN         = SERIAL_NUMBER;
    
    // Response
    TRESPONSE response;

    response.size_of_payload = 8;
    response.payload = (uint8_t *)malloc(response.size_of_payload);

    if (!response.payload) {
        response.size_of_payload = 0;
        response.status_code = E_INSUFFICIENT_RESOURCES;
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

    if (request_payload_size != 0) {
        response.status_code = E_NO_PARAM_EXPECTED;
    } else {
        response.status_code = E_SUCCESS;
    }

    return response;
}
