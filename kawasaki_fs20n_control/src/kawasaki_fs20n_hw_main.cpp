#include <ros_control_boilerplate/generic_hw_control_loop.h>
#include <kawasaki_fs20n_control/kawasaki_fs20n_hw_interface.h>
#include <signal.h>

void CtrlC_handler(int sig);

std::shared_ptr<kawasaki_ns::KawasakiHWInterface> kawasaki_fs20n_hw_interface;

int main(int argc, char** argv) {
    ros::init(argc, argv, "kawasaki_fs020n_hw_interface");
    ros::NodeHandle nh;
    signal(SIGINT, CtrlC_handler);

    // NOTE: We run the ROS loop in a separate thread as external calls such
    // as service callbacks to load controllers can block the (main) control loop
    ros::AsyncSpinner spinner(3);
    spinner.start();

    // Create the hardware interface specific to your robot
    kawasaki_fs20n_hw_interface = std::shared_ptr<kawasaki_ns::KawasakiHWInterface>(new kawasaki_ns::KawasakiHWInterface(nh));
    kawasaki_fs20n_hw_interface->init();

    // Start the control loop
    ros_control_boilerplate::GenericHWControlLoop control_loop(nh, kawasaki_fs20n_hw_interface);
    control_loop.run();  // Blocks until shutdown signal recieved

    return 0;
}

void CtrlC_handler(int sig) {
    kawasaki_fs20n_hw_interface->disconnectFromRobot();
    exit(EXIT_SUCCESS);
}