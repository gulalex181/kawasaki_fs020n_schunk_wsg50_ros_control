#ifndef KAWASAKI_INTERFACE_H
#define KAWASAKI_INTERFACE_H

#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>
#include <arpa/inet.h>

#include <string.h>
#include <sstream>
#include <iostream>
#include <vector>
#include <regex>
#include <math.h>

#include <ros/ros.h>
#include <ros_control_boilerplate/generic_hw_interface.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>

#include "wsg50_common/Status.h"
#include "wsg50_common/Cmd.h"

#define DEG_TO_RAD 0.01745329251
#define RAD_TO_DEG 57.2957795131
#define LOG_PREFIX "KawasakiHWInterface"

namespace kawasaki_ns {

/** \brief Hardware interface for a robot */
class KawasakiHWInterface: public ros_control_boilerplate::GenericHWInterface {
public:
    /**
     * \brief Constructor
     * \param nh - Node handle for topics.
     */
    KawasakiHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model = NULL);

    /**
     * \brief Destructor
     */
    ~KawasakiHWInterface();

    /** \brief Initialize the robot hardware interface */
    virtual void init();

    /** \brief Read the state from the robot hardware. */
    virtual void read(ros::Duration& elapsed_time);

    /** \brief Write the command to the robot hardware. */
    virtual void write(ros::Duration& elapsed_time);

    /** \breif Enforce limits for all values before writing */
    virtual void enforceLimits(ros::Duration& period);

    void disconnectFromRobot();
    void disconnectFromGripper();

protected:
    int socket_fd = 0;
    struct sockaddr_in serv_addr;
    char buffer[100] = {0};

    int gripper_socket_fd = 0;
    struct sockaddr_in gripper_serv_addr;
    char gripper_buffer[100] = {0};

    bool output_to_console;
    std::size_t joints_number;

    std::vector<double> joint_position_last = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    float palm_revolute_upper = 2 * M_PI;                                         // rad
    float palm_revolute_lower = 0;                                                // rad
    float palm_joint_gripper_upper = 55;                                          // mm
    float palm_joint_gripper_lower = 5;                                           // mm
    float offset = palm_joint_gripper_lower;                                      // mm
    float multiplier = (palm_joint_gripper_upper - offset) / palm_revolute_upper;

    ros::Subscriber gripper_subscriber;
    ros::Publisher gripper_publisher;

    void connectToRobot();
    void sendCmdToRobot(std::string cmd, bool is_disconnect = false);
    void receiveAnsFromRobot();

    void connectToGripper();
    void sendCmdToGripper();
    void receiveAnsFromGripper(const wsg50_common::Status::ConstPtr& msg);

    void connectionError(std::string msg);

    std::string getStartMovementProgramCmd();
    std::string getStopMovementProgramCmd();
    std::string getStopCurrentMovementCmd();
    std::string getGoToPoseCmd();
    std::string getStopMovementAndCloseConnectionCmd();
    
    std::string getVersionFromParamServer();
    std::vector<std::string> splitStringToVector(const std::string &s, char delim);
};  // class

}  // namespace ros_control_boilerplate

#endif
