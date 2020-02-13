/*
 * File: bluerov_teleop/include/BluerovTeleop.h
 * Author: Antonella Wilby <antonella@explorationlabs.com>
 * Date: Feb. 2020
 * Description: This software provides remote control (teleoperation) of the BlueROV2.
 *              This software was modified from the original teleoperation code in
 *              the original bluerov_apps ROS package.
 */

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <dynamic_reconfigure/server.h>
#include <bluerov_robot/Arm.h>
#include <bluerov_teleop/bluerov_teleopConfig.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <vector>
#include <math.h>


class BluerovTeleop {

    public:
        BluerovTeleop(ros::NodeHandle* nodehandle);
        void spin();

    private:
        ros::NodeHandle nh_;        // ROS Node Handle

        // CONSTANTS: see https://pixhawk.ethz.ch/mavlink/
        enum {COMPONENT_ARM_DISARM=400};
        enum {FORCE_DISARM = 21196};
        enum {MODE_STABILIZE = 1000, MODE_DEPTH_HOLD = 2000, MODE_MANUAL=1000};  // ppm in uS
        enum {PPS_MIN = 1000, PPS_MAX = 2000};  // ppm in uS
        enum {CAM_TILT_RESET = 1500};  // ppm in uS


        // Type of joystick
        std::string joystick;
        std::string joy_topic;

        // Keep track of previous buttons pressed
        std::vector<int> previous_buttons;

        // Variables to keep track of current state
        uint16_t mode;
        uint16_t camera_tilt;
        bool initLT;
        bool initRT;


        // Dynamic reconfigure
        dynamic_reconfigure::Server<bluerov_teleop::bluerov_teleopConfig> server;
        bluerov_teleop::bluerov_teleopConfig config;

        // Subscriber for joystick input
        ros::Subscriber joy_sub;

        // Publisher for thruster msgs
        ros::Publisher rc_override_pub;

        // Service for arming robot
        ros::ServiceClient arm_client; // = nh_.serviceClient<bluerov_robot::Arm>("bluerov_arm");

        // Dynamic reconfigure server callback function
        void configCallback(bluerov_teleop::bluerov_teleopConfig &update, uint32_t level);

        // Joystick callback function
        void joy_callback(const sensor_msgs::Joy::ConstPtr& joy);

        // Get joystick button presses
        bool risingEdge(const sensor_msgs::Joy::ConstPtr& joy, int index);

        // Computes the position of joystick for publishing thrust control
        double computeAxisValue(const sensor_msgs::Joy::ConstPtr& joy, int index, double expo);

        // Maps axis values to PPM (Pulse Position Modulation) commands
        uint16_t mapToPpm(double in);

        // Set arming request to FCU via bluerov_robot
        void request_arm(bool arm_input);

        // Remaps F310 joystick values since d-pad buttons are treated as axes
        sensor_msgs::Joy::ConstPtr f310_RemapJoystick(const sensor_msgs::Joy::ConstPtr& f310);

};
