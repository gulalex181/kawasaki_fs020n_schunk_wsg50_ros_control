// The original code here:
// https://github.com/ros-planning/moveit_tutorials/tree/master/doc/move_group_interface

#include <vector>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <tf/tf.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>

void update_param_server(std::string, std::string);
std::vector<double> read_goal_from_param_server(ros::NodeHandle);
geometry_msgs::Pose get_goal_pose_msg(std::vector<double>);
bool is_goal_changed(std::vector<double>, std::vector<double>);
void print_current_state(std::string, const moveit::planning_interface::MoveGroupInterface &);
void add_table_to_scene(std::string, ros::NodeHandle, const moveit::planning_interface::MoveGroupInterface &);

int main(int argc, char** argv) {
    ros::init(argc, argv, "kawasaki_fs020n_moveit_control");
    ros::NodeHandle nh;

    std::string settings_file_path = ros::package::getPath("kawasaki_fs20n_control") + "/config/settings.yaml";
    std::string console_output_prefix = "[KawasakiMoveItControl]: ";
    std::vector<double> last_goal(7);

    // ROS spinning must be running for the MoveGroupInterface to get information
    // about the robot's state. One way to do this is to start an AsyncSpinner
    // beforehand.
    ros::AsyncSpinner spinner(3);
    spinner.start();

    // MoveIt operates on sets of joints called "planning groups" and stores them in an object called
    // the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
    // are used interchangeably.
    static const std::string PLANNING_GROUP = "manipulator";

    // The :planning_interface:`MoveGroupInterface` class can be easily
    // setup using just the name of the planning group you would like to control and plan for.
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

    // Raw pointers are frequently used to refer to the planning group for improved performance.
    const moveit::core::JointModelGroup* joint_model_group =
        move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // We can print the name of the reference frame for this robot.
    ROS_INFO_STREAM(console_output_prefix << "Planning frame: " << move_group_interface.getPlanningFrame().c_str());

    // We can also print the name of the end-effector link for this group.
    ROS_INFO_STREAM(console_output_prefix << "End effector link: " << move_group_interface.getEndEffectorLink().c_str());

    // The package MoveItVisualTools provides many capabilities for visualizing objects, robots,
    // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script.
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
    visual_tools.deleteAllMarkers();

    // Remote control is an introspection tool that allows users to step through a high level script
    // via buttons and keyboard shortcuts in RViz
    visual_tools.loadRemoteControl();

    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.0;

    /* Add table to the scene */

    add_table_to_scene(console_output_prefix, nh, move_group_interface);

    // We can plan a motion for this group to a desired pose for the end-effector

    std::vector<double> goal(7);
    bool goal_changed = false;

    while (ros::ok()) {

        move_group_interface.setStartState(*move_group_interface.getCurrentState());

        if (!goal_changed) {
            visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to plan trajectory");
        
            update_param_server(console_output_prefix, settings_file_path);
            last_goal = goal;
            goal = read_goal_from_param_server(nh);
        }

        if (last_goal == goal) continue;

        geometry_msgs::Pose goal_pose = get_goal_pose_msg(goal);

        move_group_interface.setPoseTarget(goal_pose);
        // move_group_interface.setMaxVelocityScalingFactor(0.05);
        // move_group_interface.setMaxAccelerationScalingFactor(0.05);
        move_group_interface.setPlanningTime(10.0);

        // Now, we call the planner to compute the plan and visualize it.
        // Note that we are just planning, not asking move_group_interface
        // to actually move the robot.
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        ROS_INFO_STREAM(console_output_prefix << "Visualizing plan (pose goal) " << (success ? "" : "FAILED"));

        if (!success) {
            goal_changed = false;
            continue;
        }

        // We can also visualize the plan as a line with markers in RViz.
        ROS_INFO_STREAM(console_output_prefix << "Visualizing plan as trajectory line");
        visual_tools.publishAxisLabeled(goal_pose, "pose");
        visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
        visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
        visual_tools.trigger();
        visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to execute the trajectory \
            or to plan it if you have changed the goal");

        update_param_server(console_output_prefix, settings_file_path);
        last_goal = goal;
        goal = read_goal_from_param_server(nh);

        if (is_goal_changed(goal, last_goal)) {
            goal_changed = true;
            continue;
        } else {
            goal_changed = false;
        }

        // Finally, to execute the trajectory stored in my_plan, you could use the following method call:
        // Note that this can lead to problems if the robot moved in the meanwhile.
        move_group_interface.execute(my_plan);

        print_current_state(console_output_prefix, move_group_interface);
    }

    ros::shutdown();
    return 0;
}

void update_param_server(std::string console_output_prefix, std::string path_to_file_) {
    boost::filesystem::path path_to_file(path_to_file_);

    if(!boost::filesystem::exists(path_to_file) ||
        !(boost::filesystem::is_regular_file(path_to_file) || boost::filesystem::is_symlink(path_to_file))) {
        ROS_WARN_STREAM(console_output_prefix << "Could not open file " << path_to_file.string());
    } else {
        // load the YAML file using the external rosparam command
        std::string command = "rosparam load " + path_to_file.string();
        int result = std::system(command.c_str());
        
        if(result != 0) {
            ROS_WARN_STREAM(console_output_prefix << "Could not set file "
                << path_to_file.string()
                << " to the parameter server.");
        }
    }
}

std::vector<double> read_goal_from_param_server(ros::NodeHandle nh) {
    std::string orientation_format;
    std::string rpy_format;
    double goal_position_x;
    double goal_position_y;
    double goal_position_z;
    double goal_orientation_x;
    double goal_orientation_y;
    double goal_orientation_z;
    double goal_orientation_w;
    double goal_rpy_roll;
    double goal_rpy_pitch;
    double goal_rpy_yaw;

    ros::NodeHandle rpnh(nh, "kawasaki_fs020n_moveit_control");
    std::size_t error = 0;

    error += !rosparam_shortcuts::get("kawasaki_fs020n_moveit_control", rpnh, "goal_position_x", goal_position_x);
    error += !rosparam_shortcuts::get("kawasaki_fs020n_moveit_control", rpnh, "goal_position_y", goal_position_y);
    error += !rosparam_shortcuts::get("kawasaki_fs020n_moveit_control", rpnh, "goal_position_z", goal_position_z);

    error += !rosparam_shortcuts::get("kawasaki_fs020n_moveit_control", rpnh, "orientation_format", orientation_format);
    error += !rosparam_shortcuts::get("kawasaki_fs020n_moveit_control", rpnh, "rpy_format", rpy_format);
   
    if (orientation_format == "quaternion") {
        error += !rosparam_shortcuts::get("kawasaki_fs020n_moveit_control", rpnh, "goal_orientation_x", goal_orientation_x);
        error += !rosparam_shortcuts::get("kawasaki_fs020n_moveit_control", rpnh, "goal_orientation_y", goal_orientation_y);
        error += !rosparam_shortcuts::get("kawasaki_fs020n_moveit_control", rpnh, "goal_orientation_z", goal_orientation_z);
        error += !rosparam_shortcuts::get("kawasaki_fs020n_moveit_control", rpnh, "goal_orientation_w", goal_orientation_w);
    } else if (orientation_format == "rpy") {
        error += !rosparam_shortcuts::get("kawasaki_fs020n_moveit_control", rpnh, "goal_rpy_roll", goal_rpy_roll);
        error += !rosparam_shortcuts::get("kawasaki_fs020n_moveit_control", rpnh, "goal_rpy_pitch", goal_rpy_pitch);
        error += !rosparam_shortcuts::get("kawasaki_fs020n_moveit_control", rpnh, "goal_rpy_yaw", goal_rpy_yaw);
    }

    rosparam_shortcuts::shutdownIfError("kawasaki_fs020n_moveit_control", error);

    if (orientation_format == "rpy") {
        tf::Quaternion q;

        if (rpy_format == "deg") {
            q.setRPY(goal_rpy_roll * M_PI / 180, goal_rpy_pitch * M_PI / 180, goal_rpy_yaw * M_PI / 180);
        } else if (rpy_format == "rad") {
            q.setRPY(goal_rpy_roll, goal_rpy_pitch, goal_rpy_yaw);
        }
        
        q = q.normalize();

        goal_orientation_x = q.getX();
        goal_orientation_y = q.getY();
        goal_orientation_z = q.getZ();
        goal_orientation_w = q.getW();
    }

    std::vector<double> goal;

    goal.push_back(goal_position_x);
    goal.push_back(goal_position_y);
    goal.push_back(goal_position_z);
    goal.push_back(goal_orientation_x);
    goal.push_back(goal_orientation_y);
    goal.push_back(goal_orientation_z);
    goal.push_back(goal_orientation_w);

    return goal;
}

geometry_msgs::Pose get_goal_pose_msg(std::vector<double> goal) {
    geometry_msgs::Pose goal_pose;
    
    goal_pose.position.x    = goal[0];
    goal_pose.position.y    = goal[1];
    goal_pose.position.z    = goal[2];
    goal_pose.orientation.x = goal[3];
    goal_pose.orientation.y = goal[4];
    goal_pose.orientation.z = goal[5];
    goal_pose.orientation.w = goal[6];

    return goal_pose;
}

bool is_goal_changed(std::vector<double> goal, std::vector<double> last_goal) {
    ROS_INFO_STREAM("[KawasakiMoveItControl]: is_goal_changed: " << (last_goal == goal) << std::endl);

    if (last_goal == goal) {
        return false;
    } else {
        return true;
    }
}

void print_current_state(std::string console_output_prefix,
    const moveit::planning_interface::MoveGroupInterface &move_group_interface) {

    std::vector<double> current_joints_state = move_group_interface.getCurrentJointValues();
    const geometry_msgs::PoseStamped& current_pose_state = move_group_interface.getPoseTarget();
    
    tf::Quaternion q(
        current_pose_state.pose.orientation.x,
        current_pose_state.pose.orientation.y,
        current_pose_state.pose.orientation.z,
        current_pose_state.pose.orientation.w
    );
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    ROS_INFO_STREAM(console_output_prefix << std::endl << std::endl <<
        "JOINTS STATE:" << std::endl << std::endl <<

            "\tJT1 (rad): " << current_joints_state[0] << std::endl <<
            "\tJT2 (rad): " << current_joints_state[1] << std::endl <<
            "\tJT3 (rad): " << current_joints_state[2] << std::endl <<
            "\tJT4 (rad): " << current_joints_state[3] << std::endl <<
            "\tJT5 (rad): " << current_joints_state[4] << std::endl <<
            "\tJT6 (rad): " << current_joints_state[5] << std::endl << std::endl <<

            "\tJT1 (deg): " << current_joints_state[0] * 180 / M_PI << std::endl <<
            "\tJT2 (deg): " << current_joints_state[1] * 180 / M_PI << std::endl <<
            "\tJT3 (deg): " << current_joints_state[2] * 180 / M_PI << std::endl <<
            "\tJT4 (deg): " << current_joints_state[3] * 180 / M_PI << std::endl <<
            "\tJT5 (deg): " << current_joints_state[4] * 180 / M_PI << std::endl <<
            "\tJT6 (deg): " << current_joints_state[5] * 180 / M_PI << std::endl << std::endl <<

        "POSE STATE:" << std::endl << std::endl <<
        
            "position:" << std::endl <<
            "\tx (m): " << current_pose_state.pose.position.x << std::endl <<
            "\ty (m): " << current_pose_state.pose.position.y << std::endl <<
            "\tz (m): " << current_pose_state.pose.position.z << std::endl << std::endl <<

            "orientation (quaternion):" << std::endl <<
            "\tx: " << current_pose_state.pose.orientation.x << std::endl <<
            "\ty: " << current_pose_state.pose.orientation.y << std::endl <<
            "\tz: " << current_pose_state.pose.orientation.z << std::endl <<
            "\tw: " << current_pose_state.pose.orientation.w << std::endl << std::endl <<

            "orientation (rpy):" << std::endl <<
            "\troll (rad): "  << roll  << std::endl <<
            "\tpitch (rad): " << pitch << std::endl <<
            "\tyaw (rad): "   << yaw   << std::endl << std::endl <<

            "\troll (deg): "  << roll  * 180 / M_PI << std::endl <<
            "\tpitch (deg): " << pitch * 180 / M_PI << std::endl <<
            "\tyaw (deg): "   << yaw   * 180 / M_PI
    );
}

void add_table_to_scene(std::string console_output_prefix, ros::NodeHandle nh,
    const moveit::planning_interface::MoveGroupInterface &move_group_interface) {

    double table_size_x;
    double table_size_y;
    double table_size_z;
    double table_pos_x;
    double table_pos_y;

    ros::NodeHandle rpnh(nh, "kawasaki_fs020n_moveit_control");
    std::size_t error = 0;
    error += !rosparam_shortcuts::get("kawasaki_fs020n_moveit_control", rpnh, "table_size_x", table_size_x);
    error += !rosparam_shortcuts::get("kawasaki_fs020n_moveit_control", rpnh, "table_size_y", table_size_y);
    error += !rosparam_shortcuts::get("kawasaki_fs020n_moveit_control", rpnh, "table_size_z", table_size_z);
    error += !rosparam_shortcuts::get("kawasaki_fs020n_moveit_control", rpnh, "table_pos_x", table_pos_x);
    error += !rosparam_shortcuts::get("kawasaki_fs020n_moveit_control", rpnh, "table_pos_y", table_pos_y);
    rosparam_shortcuts::shutdownIfError("kawasaki_fs020n_moveit_control", error);

    ROS_INFO_STREAM(console_output_prefix << "kawasaki_fs020n_move_group_interface INIT");

    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group_interface.getPlanningFrame();

    // The id of the object is used to identify it.
    collision_object.id = "table";

    // Define a box to add to the world.
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = table_size_x;
    primitive.dimensions[primitive.BOX_Y] = table_size_y;
    primitive.dimensions[primitive.BOX_Z] = table_size_z;

    // Define a pose for the box (specified relative to frame_id)
    geometry_msgs::Pose box_pose;
    box_pose.orientation.z = 0.0;
    box_pose.orientation.y = 0.0;
    box_pose.orientation.z = 0.0;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = table_pos_x;
    box_pose.position.y = table_pos_y;
    box_pose.position.z = table_size_z / 2;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

    // We will use the :planning_interface:`PlanningSceneInterface`
    // class to add and remove collision objects in our "virtual world" scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Now, let's add the collision object into the world
    // (using a vector that could contain additional objects)
    ROS_INFO_STREAM(console_output_prefix << "Add a table into the world");

    planning_scene_interface.addCollisionObjects(collision_objects);
}