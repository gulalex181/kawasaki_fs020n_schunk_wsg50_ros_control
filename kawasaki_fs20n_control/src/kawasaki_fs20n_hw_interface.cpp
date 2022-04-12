#include <kawasaki_fs20n_control/kawasaki_fs20n_hw_interface.h>
// #include <kawasaki_fs20n_control/schunk_wsg50_hw_interface.h>

namespace kawasaki_ns {

KawasakiHWInterface::KawasakiHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model)
    : ros_control_boilerplate::GenericHWInterface(nh, urdf_model) {
    name_ = "kawasaki_fs020n_hw_interface";
    nh_ = nh;
}

KawasakiHWInterface::~KawasakiHWInterface() {
    disconnectFromRobot();
    disconnectFromGripper();
}

void KawasakiHWInterface::init() {
    // Call parent class version of this function
    ros_control_boilerplate::GenericHWInterface::init();

    ros::NodeHandle rpnh(nh_, name_);
    std::size_t error = 0;
    error += !rosparam_shortcuts::get(name_, rpnh, "robot_joints_number", joints_number);
    rosparam_shortcuts::shutdownIfError(name_, error);

    ROS_INFO_STREAM_NAMED(name_, "[" << LOG_PREFIX << "]: " << "ready");

    connectToRobot();
    connectToGripper();

    gripper_subscriber = nh_.subscribe("/wsg50_driver/status", 5,
        &KawasakiHWInterface::receiveAnsFromGripper, this);
    gripper_publisher = nh_.advertise<wsg50_common::Cmd>("/wsg50_driver/goal_position", 1000);
}

void KawasakiHWInterface::read(ros::Duration& elapsed_time) {
    // std::string pos = "";
    for (int i = 0; i < joints_number; ++i) {
        joint_position_[i] = joint_position_last[i];
    //     pos += i + ": '" + std::to_string(joint_position_[i]) + "' ";
    }

    // joint_position_[num_joints_ - 1] = joint_position_command_[num_joints_ - 1];
    // joint_position_[num_joints_] = joint_position_command_[num_joints_];
    // ROS_INFO_STREAM_NAMED(name_, "[" << LOG_PREFIX << "]: " << pos);
}

void KawasakiHWInterface::write(ros::Duration& elapsed_time) {
    // Safety
    //enforceLimits(elapsed_time);

    // ROS_INFO_STREAM_NAMED(name_, "[" << LOG_PREFIX << "]: " << "WRITE TO ROBOT");
    std::string pos = "";
    for (int i = 0; i < num_joints_; ++i) {
        // joint_position_[i] = joint_position_command_[i];
        pos += i + ": '" + std::to_string(joint_position_[i]) + "' ";
    }
    // ROS_INFO_STREAM_NAMED(name_, "[" << LOG_PREFIX << "]: " << pos);

    sendCmdToRobot(getGoToPoseCmd());
    sendCmdToGripper();
}

void KawasakiHWInterface::enforceLimits(ros::Duration& period) {
    // Enforces position and velocity
    //pos_jnt_sat_interface_.enforceLimits(period);
}

void KawasakiHWInterface::connectToRobot() {
    // Create client socket
    socket_fd = socket(AF_INET, SOCK_STREAM, 0);

    if (socket_fd < 0) {
        connectionError("creating socket error");
    }

    // Get connection params from parameter server
    std::string ip_address;
    int port;

    ros::NodeHandle rpnh(nh_, name_);
    std::size_t error = 0;
    error += !rosparam_shortcuts::get(name_, rpnh, "robot_ip", ip_address);
    error += !rosparam_shortcuts::get(name_, rpnh, "robot_port", port);
    error += !rosparam_shortcuts::get(name_, rpnh, "output_to_console", output_to_console);
    // If there is no any of the requested param then throw an error
    rosparam_shortcuts::shutdownIfError(name_, error);

    // Configure robot server address structure
    serv_addr.sin_addr.s_addr = inet_addr(ip_address.c_str());
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port);

    // Connecting to robot server
    if (connect(socket_fd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
        connectionError("connecting error");
    }

    ROS_INFO_STREAM_NAMED(name_, "[" << LOG_PREFIX << "]: " << "connected to server");

    // Launch robot movement program.
    sendCmdToRobot(getStartMovementProgramCmd());

    ROS_INFO_STREAM_NAMED(name_, "[" << LOG_PREFIX << "]: " << "robot movement program is launched");
}

void KawasakiHWInterface::connectionError(std::string msg) {
    ROS_ERROR_STREAM_NAMED(name_, "[" << LOG_PREFIX << "]: " << msg);
    ros::shutdown();
    exit(0);
}

void KawasakiHWInterface::disconnectFromRobot() {
    sendCmdToRobot(getStopMovementAndCloseConnectionCmd(), true);
    ROS_INFO_STREAM_NAMED(name_, "[" << LOG_PREFIX << "]: " << "robot movement program is stopped");
    
    int close_result = close(socket_fd);
    ROS_INFO_STREAM_NAMED(name_, "[" << LOG_PREFIX << "]: " << "disconnected from robot server with code: " << close_result);
}

void KawasakiHWInterface::sendCmdToRobot(std::string cmd, bool is_disconnect) {
    if (output_to_console) {
        ROS_INFO_STREAM_NAMED(name_, "[" << LOG_PREFIX << "]: " << "COMMAND TO ROBOT: '" << cmd << "'");
    }
    
    if (::write(socket_fd, cmd.c_str(), cmd.length()) < 0) {
        connectionError("writing to socker error");
    }

    if (is_disconnect) {
        return;
    }

    receiveAnsFromRobot();
}

void KawasakiHWInterface::receiveAnsFromRobot() {
    memset(buffer, 0, strlen(buffer));
    
    int count_of_bytes = ::read(socket_fd, buffer, sizeof(buffer) - 1);
    
    if (output_to_console) {
        ROS_INFO_STREAM_NAMED(name_, "[" << LOG_PREFIX << "]: " << "buffer: '" << buffer << "'" << std::endl);
    }
    
    if (count_of_bytes < 0) {
        connectionError("reading from socker error");
    }

    buffer[strlen(buffer)] = '\0';
    
    std::string response = std::string(buffer);
    std::vector<std::string> responseVector = splitStringToVector(response, ' ');


    if (output_to_console) {
        ROS_INFO_STREAM_NAMED(name_, "[" << LOG_PREFIX << "]: " << "STATUS OF ROBOT:" << std::endl
            << "VERSION: " << responseVector[0] << std::endl
            << "ERROR: " << responseVector[1] << std::endl
            << "SWITCH(RUN): " << responseVector[2] << std::endl
            << "SWITCH(REPEAT): " << responseVector[3] << std::endl
            << "SWITCH(TEACH_LOCK): " << responseVector[4] << std::endl
            << "SWITCH(POWER): " << responseVector[5] << std::endl
            << "TASK(1) == 1: " << responseVector[6] << std::endl
            << "TIMER(1): " << responseVector[7] << std::endl
            << "JT1: " << responseVector[8] << std::endl
            << "JT2: " << responseVector[9] << std::endl
            << "JT3: " << responseVector[10] << std::endl
            << "JT4: " << responseVector[11] << std::endl
            << "JT5: " << responseVector[12] << std::endl
            << "JT6: " << responseVector[13] << std::endl
        );
    }

    // for (int i = 0; i < joints_number; ++i) {
    //     joint_position_[i] = stod(responseVector[8 + i]) * DEG_TO_RAD;
    // }

    for (int i = 0; i < joints_number; ++i) {
        joint_position_last.at(i) = stod(responseVector[8 + i]) * DEG_TO_RAD;
    }
}

std::string KawasakiHWInterface::getStartMovementProgramCmd() {
    return getVersionFromParamServer() + " 1 0";
}

std::string KawasakiHWInterface::getStopMovementProgramCmd() {
    return getVersionFromParamServer() + " 2 0";
}

std::string KawasakiHWInterface::getStopCurrentMovementCmd() {
    return getVersionFromParamServer() + " 5 0";
}

std::string KawasakiHWInterface::getGoToPoseCmd() {
    std::string joint_positions_str = "";
    
    for(int i = 0; i < joints_number; ++i) {
        if (i == (joints_number - 1)) {
            joint_positions_str += std::to_string(joint_position_command_[i] * RAD_TO_DEG);
        } else {
            joint_positions_str += std::to_string(joint_position_command_[i] * RAD_TO_DEG) + " ";
        }
    }

    return getVersionFromParamServer() + " 6 9 0 0 0 " + joint_positions_str;
}

std::string KawasakiHWInterface::getStopMovementAndCloseConnectionCmd() {
    return getVersionFromParamServer() + " 255 0";
}

std::string KawasakiHWInterface::getVersionFromParamServer() {
    std::string version;

    ros::NodeHandle rpnh(nh_, name_);
    std::size_t error = 0;
    error += !rosparam_shortcuts::get(name_, rpnh, "robot_version", version);
    // If there is no any of the requested param then throw an error
    rosparam_shortcuts::shutdownIfError(name_, error);

    return version;
}

std::vector<std::string> KawasakiHWInterface::splitStringToVector(const std::string &str, char delim) {
    // std::vector<std::string> elems;
    // std::stringstream ss;
    // ss.str(s);
    // std::string item;
    // while (std::getline(ss, item, delim)) {
    //     elems.push_back(item);
    // }

    if (output_to_console) {
        ROS_INFO_STREAM_NAMED(name_, "[" << LOG_PREFIX << "]: " << "string1: '" << str << "'" << std::endl);
    }
    
    std::string s = std::regex_replace(str, std::regex("^\\s+|\\s+$"), "");

    if (output_to_console) {
        ROS_INFO_STREAM_NAMED(name_, "[" << LOG_PREFIX << "]: " << "string2: '" << s << "'" << std::endl);
    }

    std::regex ws_re("\\s+"); // whitespace
    std::vector<std::string> elems{
        std::sregex_token_iterator(s.begin(), s.end(), ws_re, -1), {}
    };

    return elems;
}

void KawasakiHWInterface::connectToGripper() {
    // Create client socket
    gripper_socket_fd = socket(AF_INET, SOCK_STREAM, 0);

    if (gripper_socket_fd < 0) {
        connectionError("creating socket error");
    }

    // Get connection params from parameter server
    std::string ip;
    int port;

    ros::param::get("/wsg50_driver/ip", ip);
    ros::param::get("/wsg50_driver/port", port);

    // Configure robot server address structure
    gripper_serv_addr.sin_addr.s_addr = inet_addr(ip.c_str());
    gripper_serv_addr.sin_family = AF_INET;
    gripper_serv_addr.sin_port = htons(port);

    // Connecting to robot server
    if (connect(gripper_socket_fd, (struct sockaddr *) &gripper_serv_addr, sizeof(gripper_serv_addr)) < 0) {
        connectionError("connecting error");
    }

    ROS_INFO_STREAM_NAMED(name_, "[" << LOG_PREFIX << "]: "
        << "connected to gripper server with IP: " << ip << " and port: " << port);
}

void KawasakiHWInterface::disconnectFromGripper() {
    int close_result = close(gripper_socket_fd);
    ROS_INFO_STREAM_NAMED(name_, "[" << LOG_PREFIX << "]: " << "disconnected from gripper server with code: " << close_result);
}

void KawasakiHWInterface::receiveAnsFromGripper(const wsg50_common::Status::ConstPtr& msg) {
    std::string gripper_current_status        = msg->status;
    float       gripper_current_width         = msg->width;
    float       gripper_current_speed         = msg->speed;
    float       gripper_current_acc           = msg->acc;
    float       gripper_current_force         = msg->force;
    float       gripper_current_force_finger0 = msg->force_finger0;
    float       gripper_current_force_finger1 = msg->force_finger1;

    if (output_to_console) {
        ROS_INFO_STREAM_NAMED(name_, "[" << LOG_PREFIX << "]: " << "STATUS OF GRIPPER:" << std::endl
            << "------------------------------------------------" << std::endl
            << "STATUS: "        << gripper_current_status        << std::endl
            << "WIDTH: "         << gripper_current_width         << std::endl
            << "SPEED: "         << gripper_current_speed         << std::endl
            << "ACC: "           << gripper_current_acc           << std::endl
            << "FORCE: "         << gripper_current_force         << std::endl
            << "FORCE_FINGER0: " << gripper_current_force_finger0 << std::endl
            << "FORCE_FINGER1: " << gripper_current_force_finger1 << std::endl
            << "------------------------------------------------" << std::endl
        );
    }

    // Get current positon of both fingers
    float finger_current_position = gripper_current_width / 2; // mm

    // Conversion (5 mm - 55 mm) to (0 rad 2pi rad) [the same as in the urdf file for gripper]
    finger_current_position = (finger_current_position - offset) / multiplier; // rad

    ROS_INFO_STREAM_NAMED(name_, "[" << LOG_PREFIX << "]: " << "finger position: " << finger_current_position << std::endl);

    joint_position_[num_joints_ - 1] = finger_current_position;
}

void KawasakiHWInterface::sendCmdToGripper() {

    // Conversion (0 rad 2pi rad) to (5 mm - 55 mm) [the same as in the urdf file for gripper]
    float finger_cmd_position = joint_position_command_[num_joints_ - 1] * multiplier + offset; // mm

    wsg50_common::Cmd gripper_cmd;
    gripper_cmd.pos = finger_cmd_position * 2;

    gripper_publisher.publish(gripper_cmd);
}

} // namespace ros_control_boilerplate
