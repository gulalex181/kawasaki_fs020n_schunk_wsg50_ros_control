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

// More details in command set reference manual v4.0.x)
typedef enum {
    // Connection Managment
    Loop                                  = 0x06,
    DisconnectAnnouncement                = 0x07,
    // Motion Control
    Homing                                = 0x20,
    PrepositionFingers                    = 0x21,
    Stop                                  = 0x22,
    IssueFastStop                         = 0x23,
    AcknowledgingFastStopOrFaultCondition = 0x24,
    GripPart                              = 0x25,
    ReleasePart                           = 0x26,
    // Motion Configuration
    SetAcceleration                       = 0x30,
    GetAcceleration                       = 0x31,
    SetForceLimit                         = 0x32,
    GetForceLimit                         = 0x33,
    SetSoftLimits                         = 0x34,
    GetSoftLimits                         = 0x35,
    ClearSoftLimits                       = 0x36,
    OverdriveMode                         = 0x37,
    TareForceSensor                       = 0x38,
    // System State Commands
    GetSystemState                        = 0x40,
    GetGripperState                       = 0x41,
    GetGrippingStatistics                 = 0x42,
    GetOpeningWidth                       = 0x43,
    GetSpeed                              = 0x44,
    GetForce                              = 0x45,
    GetTemperature                        = 0x46,
    // System Configuration
    GetSystemInformation                  = 0x50,
    SetDeviceTag                          = 0x51,
    GetDeviceTag                          = 0x52,
    GetSystemLimits                       = 0x53,
    // Finger Interface
    GetFinger1Info                        = 0x60,
    GetFinger1Flags                       = 0x61,
    Finger1PowerControl                   = 0x62,
    GetFinger1Data                        = 0x63,
    GetFinger2Info                        = 0x70,
    GetFinger2Flags                       = 0x71,
    Finger2PowerControl                   = 0x72,
    GetFinger2Data                        = 0x73,
    // Custrom commands
    Measure                               = 0xB0,
    PositionControl                       = 0xB1,
    SpeedControl                          = 0xB2
} TCommand;

typedef struct {
    uint8_t      preamble[3] = {0xAA, 0xAA, 0xAA};
    uint8_t      command_id;
    uint16_t     size_of_payload;
    uint8_t*     payload;
} TREQUEST;

typedef struct {
    uint8_t      preamble[3] = {0xAA, 0xAA, 0xAA};
    uint8_t      command_id;
    uint16_t     size_of_payload;
    uint16_t     status_code;
    std::string  status;
    uint8_t*     payload;
} TRESPONSE;

typedef struct {
    TRESPONSE*   responses;
    int*         delays;
    size_t       sequence_size;
} ResponseSequenceType;

typedef struct {
    uint8_t**    packets;
    uint16_t*    packets_sizes;
    int*         delays;
    std::string* packets_statuses;
    bool*        packets_free_need;
    size_t       sequence_size;
} PacketSequenceType;

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
// Network settings
#define BUFFER_SIZE            1024     //!< buffer size
#define PORT                   5222     //!< port
// System information
#define SYSTEM_TYPE            WSG_50   //!< hardware rivision
#define HARDWARE_RIVISION      5        //!< hardware rivision
#define SERIAL_NUMBER          00003713 //!< serial number
#define FIRMWARE_VERSION_MAJOR 3        //!< firmware major version
#define FIRMWARE_VERSION_MINOR 0        //!< firmware minor version
#define FIRMWARE_VERSION_PATH  0        //!< firmware path version
// Gripper information
#define OVR_FORCE              80       //!< gripper maximum overdrive grasping force (N)
#define NOM_FORCE              80       //!< gripper nominal grasping force (N)
#define MIN_FORCE              5        //!< gripper minimum grasping force (N)
#define MAX_ACC                5000     //!< gripper maximum acceleration (mm/s^2)
#define MIN_ACC                100      //!< gripper minimum acceleration (mm/s^2)
#define MAX_SPEED              420      //!< gripper maximum speed (mm/s)
#define MIN_SPEED              5        //!< gripper minimum speed (mm/s)
#define STROKE                 110      //!< gripper stroke (mm)
// Delays
#define HOMING_DELAY           1000     //!< homing delay (ms)

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

void SIGINT_handler(int sig);
void print_packet_data(const char* prefix, uint16_t packet_size, uint8_t* packet, NET_DIR dir,
    std::string command, std::string status);
PacketSequenceType request_handling(uint8_t* request, std::string* command_name);
uint16_t checksum_update_crc16(uint8_t* data, u_int16_t size, uint16_t crc);
uint8_t* response_packet_build(TRESPONSE *response, uint16_t* size);
uint8_t* insufficient_resourses_response_packet_build(uint8_t command_id);
// TStat msg_send(FILE *file, TMESSAGE *msg);

// Built-in commands
ResponseSequenceType disconnect_announcement(TREQUEST* request);
ResponseSequenceType homing(TREQUEST* request);
ResponseSequenceType get_system_information(TREQUEST* request);
ResponseSequenceType get_system_limits(TREQUEST* request);
// Custom commands (in Lua script)
ResponseSequenceType measure(TREQUEST* request);
ResponseSequenceType position_control(TREQUEST* request);
ResponseSequenceType speed_control(TREQUEST* request);

uint8_t request[BUFFER_SIZE] = { 0 };

int server_socket_fd = 0;
int client_socket_fd = 0;

struct sockaddr_in serverAddr;

uint8_t insufficient_resourses_response_packet[10] = {
    0xAA, 0xAA, 0xAA,                                           // PREAMBLE
    0x00,                                                       // COMMAND_ID
    0x02, 0x00,                                                 // SIZE_OF_PAYLOAD
                                                                // (status code included in size of payload)
    lo(E_INSUFFICIENT_RESOURCES), hi(E_INSUFFICIENT_RESOURCES), // STATUS_CODE
    0x00, 0x00                                                  // CHECKSUM
};

// System state flags
bool SF_REFERENCED         = 0; // Fingers referenced (if set, the gripper is referenced and
                                // accepts movement commands)
bool SF_MOVING             = 0; // The fingers are currently moving
bool SF_BLOCKED_MINUS      = 0; // Axis is blocked in negative moving direction
bool SF_BLOCKED_PLUS       = 0; // Axis is blocked in positive moving direction (the flag is reset
                                // if either the blocking condition has been resolved or a stop command
                                // has been issued)
bool SF_SOFT_LIMIT_MINUS   = 0; // Negative direction soft limit reached
bool SF_SOFT_LIMIT_PLUS    = 0; // Positive direction soft limit reached (a further movement into this
                                // direction is not allowed anymore; the flag is reset when the fingers
                                // have been moved away from the soft limit position)
bool SF_AXIS_STOPPED       = 0; // Axis stopped (a previous command was aborted using the stop command;
                                // the flag is reset on the next motion command)
bool SF_TARGET_POS_REACHED = 0; // Target position reached
bool SF_OVERDRIVE_MODE     = 0; // Overdrive mode
bool SF_FORCECNTL_MODE     = 0; // Force control mode (enabled if FMF is installed;
                                // if flag is not set, force is controlled by approximation based on the motor current)
bool SF_FAST_STOP          = 0; // Fast stop (the gripper has been stopped due to an error condition)
bool SF_TEMP_WARNING       = 0; // Temperature warning
bool SF_TEMP_FAULT         = 0; // Temperature error
bool SF_POWER_FAULT        = 0; // Power error
bool SF_CURR_FAULT         = 0; // Engine current error
bool SF_FINGER_FAULT       = 0; // Finger fault
bool SF_CMD_FAILURE        = 0; // Command error (last command returned an error)
bool SF_SCRIPT_RUNNING     = 0; // A script is currently running
bool SF_SCRIPT_FAILURE     = 0; // Script error

float gripper_current_position      = 0;
float gripper_current_speed         = 0;
float gripper_current_force_motor   = 0;
float gripper_current_force_finger0 = 0;
float gripper_current_force_finger1 = 0;