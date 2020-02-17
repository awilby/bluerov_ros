/*
 * File: bluerov_teleop/include/BluerovTeleop.h
 * Author: Antonella Wilby <antonella@explorationlabs.org>
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
#include <mavros_msgs/SetMode.h>
#include <sound_play/sound_play.h>
#include <vector>
#include <math.h>
#include<string>
#include <boost/thread/thread.hpp>


class BluerovTeleop {

    public:
        BluerovTeleop(ros::NodeHandle* nodehandle);
        void spin();

    private:
        ros::NodeHandle nh_;        // ROS Node Handle

        // CONSTANTS: see https://mavlink.io/en/messages/common.html
        enum {COMPONENT_ARM_DISARM = 400};
        enum {FORCE_DISARM = 21196};
        enum {DO_SET_SERVO = 183};
        //enum {DO_SET_MODE = 176, MODE_STABILIZE = 1000, MODE_DEPTH_HOLD = 2000, MODE_MANUAL=1000};  // ppm in uS
        //enum {DO_SET_MODE = 176, MODE_MANUAL = 0, MODE_STABILIZE = 1, MODE_DEPTH_HOLD = 2};
        enum {NAV_LAND_LOCAL = 23, NAV_TAKEOFF_LOCAL = 24};
        enum {PPS_MIN = 1000, PPS_MAX = 2000};  // ppm in uS
        enum {CAM_TILT_RESET = 1500};  // ppm in uS

        enum {LIGHTS_AUX_CHAN = 0};
        enum {CAM_AUX_CHAN = 1};

        // See supported mavros custom modes: http://wiki.ros.org/mavros/CustomModes
        const std::string MODE_MANUAL = "MANUAL";
        const std::string MODE_STABILIZE = "STABILIZE";
        const std::string MODE_DEPTH_HOLD = "ALT_HOLD";


        // Type of joystick
        std::string joystick;
        std::string joy_topic;
        std::string initial_mode;

        // Keep track of previous buttons pressed
        std::vector<int> previous_buttons;

        // Variables to keep track of current state
        std::string mode;
        uint16_t camera_tilt;
        bool initLT;
        bool initRT;

        int lights_level;  // Light level can be integer from 0 through 4

        // SoundPlay client for speaking responses for commands
        sound_play::SoundClient sc;


        // Dynamic reconfigure
        dynamic_reconfigure::Server<bluerov_teleop::bluerov_teleopConfig> server;
        bluerov_teleop::bluerov_teleopConfig config;

        // Subscriber for joystick input
        ros::Subscriber joy_sub;

        // Publisher for thruster msgs
        ros::Publisher rc_override_pub;

        // Services for mavros commands
        ros::ServiceClient cmd_client; // = nh_.serviceClient<bluerov_robot::Arm>("bluerov_arm");
        ros::ServiceClient set_mode;


        // Dynamic reconfigure server callback function
        void configCallback(bluerov_teleop::bluerov_teleopConfig &update, uint32_t level);

        // Joystick callback function
        void joy_callback(const sensor_msgs::Joy::ConstPtr& joy);

        // Get joystick button presses
        bool buttonPress(const sensor_msgs::Joy::ConstPtr& joy, int index);

        // Computes the position of joystick for publishing thrust control
        double computeAxisValue(const sensor_msgs::Joy::ConstPtr& joy, int index, double expo);

        // Maps axis values to PPM (Pulse Position Modulation) commands
        uint16_t mapToPpm(double in);

        // Set arming request to FCU
        void requestArm(bool arm_input);

        // Set mode
        void setMode(std::string mode);

        // Lights
        void lightsOnOff(bool light_input);

        // Auto descend/ascend to depth
        void autoDescendAscend(bool autodepth);

        // Remaps F310 joystick values since d-pad buttons are treated as axes
        sensor_msgs::Joy::ConstPtr f310_RemapJoystick(const sensor_msgs::Joy::ConstPtr& f310);

};
