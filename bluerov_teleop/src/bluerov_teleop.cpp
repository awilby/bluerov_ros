/*
 * File: bluerov_teleop/src/bluerov_teleop.cpp
 * Author: Antonella Wilby <antonella@explorationlabs.com>
 * Date: Feb. 2020
 * Description: This software provides remote control (teleoperation) of the BlueROV2.
 *              This software was modified from the original teleoperation code in
 *              the original bluerov_apps ROS package.
 */

#include "BluerovTeleop.h"


BluerovTeleop::BluerovTeleop(ros::NodeHandle* nodehandle):nh_(*nodehandle) {

    ROS_INFO("Starting BlueROV teleoperation...");

    // Load private namespace parameters
    ros::NodeHandle nh("~");    // new nodehandle with private namespace
    nh.param<std::string>("joystick", joystick, "");
    nh.param<std::string>("joy_topic", joy_topic, "joy");

    // Set up dynamic reconfigure server
    dynamic_reconfigure::Server<bluerov_teleop::bluerov_teleopConfig>::CallbackType f;
    f = boost::bind(&BluerovTeleop::configCallback, this, _1, _2);
    server.setCallback(f);

    // Subscribe to incoming joystick commands
    joy_sub = nh_.subscribe<sensor_msgs::Joy>(joy_topic, 1, &BluerovTeleop::joy_callback, this);

    // Publish thrust commands on mavros rc_override
    //rc_override_pub = nh_.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 1);

    // Initial state of vehicle
    mode = MODE_STABILIZE;
    camera_tilt = CAM_TILT_RESET;
    initLT = false;
    initRT = false;

}


/*
 * Callback function for dynamic reconfigure server.
 */
void BluerovTeleop::configCallback(bluerov_teleop::bluerov_teleopConfig &update, uint32_t level) {
  ROS_INFO("Reconfigure request received.");
  config = update;
}


/*
 * Callback function for incoming joystick commands.
 */
void BluerovTeleop::joy_callback(const sensor_msgs::Joy::ConstPtr& input) {

    sensor_msgs::Joy::ConstPtr joy = input;

    // If we're using an f310 joystick, remap buttons
    if(joystick == "f310") {
        joy = f310_RemapJoystick(input);

    }

    // Initialize previous buttons
    if (previous_buttons.size() != joy->buttons.size()) {
         previous_buttons = std::vector<int>(joy->buttons);
    }

    // ARMING
    if (risingEdge(joy, config.disarm_button)) {     // Disarm
       request_arm(false);

    } else if(risingEdge(joy, config.arm_button)) {  // Arm
        request_arm(true);

    }

    // MODE SWITCHING: Manual, stabilize, depth hold
    if (risingEdge(joy, config.stabilize_button)) {
        mode = MODE_STABILIZE;
        ROS_INFO("Entered stabilized flight mode.");

    } else if (risingEdge(joy, config.depth_hold_button)) {
        mode = MODE_DEPTH_HOLD;
        ROS_INFO("Entered depth hold mode.");

    } else if (risingEdge(joy, config.manual_button)) {
        mode = MODE_MANUAL;
        ROS_INFO("Entered manual flight mode.");

    }

    // CAMERA TILT
    if (risingEdge(joy, config.cam_tilt_reset)) {
         camera_tilt = CAM_TILT_RESET;

    } else if (risingEdge(joy, config.cam_tilt_up)) {
        camera_tilt = camera_tilt + config.cam_tilt_step;

        if (camera_tilt > PPS_MAX) {
            camera_tilt = PPS_MAX;
        }
    }


    // Remember current button states for future comparison
    previous_buttons = std::vector<int>(joy->buttons);


    // RC OVERRIDE MESSAGE
    mavros_msgs::OverrideRCIn msg;      // TODO: bluerov_robot

    // THRUSTER CONTROL: forward, strafe, throttle
    /*msg.channels[5] = mapToPpm(config.x_scaling  * computeAxisValue(joy, config.x_axis,  config.expo)); // forward  (x)
    msg.channels[6] = mapToPpm(config.y_scaling  * computeAxisValue(joy, config.y_axis,  config.expo)); // strafe   (y)
    msg.channels[2] = mapToPpm(config.z_scaling  * computeAxisValue(joy, config.z_axis,  config.expo)); // throttle (z)

    // THRUSTER CONTROL: roll, pitch, yaw
    msg.channels[1] = mapToPpm(config.wx_scaling * computeAxisValue(joy, config.wx_axis, config.expo)); // roll     (wx)
    msg.channels[0] = mapToPpm(config.wy_scaling * computeAxisValue(joy, config.wy_axis, config.expo)); // pitch    (wy)
    msg.channels[3] = mapToPpm(config.wz_scaling * computeAxisValue(joy, config.wz_axis, config.expo)); // yaw      (wz)

    */

    // MODE AND CAMERA CONTROL
    msg.channels[4] = mode; // mode
    msg.channels[7] = camera_tilt; // camera tilt

    rc_override_pub.publish(msg);

}


/*
 * Gets current axis value from joystick and returns exponentially scaled value.
 * Input:
 *     joy: incoming value from joystick driver
 *     index: which joystick is command issued on
 *     expo: exponential scaling factor
 * Output:
 *     exponentially-scaled value representing joystick axis position
 */
double BluerovTeleop::computeAxisValue(const sensor_msgs::Joy::ConstPtr& joy, int index, double expo) {

    // Return 0 if axis index is invalid
    if(index < 0 || index>= joy->axes.size()) {
        return 0.0;
    }

    // grab axis value
    double value;

    // The joystick driver initializes all values to 0.0, however, the triggers
    // physically spring back to 1.0 - let's account for this here
    if(index == 6) {

        double lt = joy->axes[2];
        double rt = joy->axes[5];

        if(lt < -0.01 || lt > 0.01) initLT = true;
        else if(!initLT) lt = 1.0;

        if(rt < -0.01 || rt > 0.01) initRT = true;
        else if(!initRT) rt = 1.0;

        // this is the trigger pair pseudo axis (LT-RT; pressing RT results in a positive number)
        value = (lt - rt) / 2.0;

    } else {
        value = joy->axes[index];

    }

    // apply exponential scaling
    return expo * pow(value, 5) + (1.0 - expo) * value;

}


/*
 * Maps joystick positions to PPM (Pulse Position Modulation) values in microseconds.
 * Input:
 *     in: value from -1 to 1 representing joystick position
 * Output:
 *     out: value from 1000 to 2000 (microseconds) for PPM
 */
uint16_t BluerovTeleop::mapToPpm(double in) {

    // Convert joystick value to PPM
    uint16_t out = 1000 + (in + 1.0) * 500;

    // Limit output value to correct PPM ranges
    if(out > 2000) {
        return 2000;
    } else if(out < 1000) {
        return 1000;
    } else {
        return out;
    }

}


/*
 * Checks for button press.
 */
bool BluerovTeleop::risingEdge(const sensor_msgs::Joy::ConstPtr& joy, int index) {
    return (joy->buttons[index] == 1 && previous_buttons[index] == 0);
}


/*
 * Sends a request to bluerov_robot arming service to arm the robot.
 */
void BluerovTeleop::request_arm(bool arm_input) {

    bluerov_robot::Arm srv;
    srv.request.arm = arm_input;

    // Call bluerov_robot arming service (pixhawk_bridge handles logging info)
    arm_client.call(srv);

}


/*
 * Remaps incoming joystick commands from F310 (Logitech) joystick because d-pad buttons
 * are treated as axes on F310.
 */
sensor_msgs::Joy::ConstPtr BluerovTeleop::f310_RemapJoystick(const sensor_msgs::Joy::ConstPtr& f310) {

    // remapped sensor message
    sensor_msgs::Joy *remap = new sensor_msgs::Joy;
    remap->header = f310->header;

    // translate axes
    // f310 axes (from): [left X, left Y, LT, right X, right Y, RT, pad L/R, pad U/D]
    // xbox axes (to):     [left X, left Y, LT, right X, right Y, RT]
    remap->axes = std::vector<float>(f310->axes);
    remap->axes.pop_back();
    remap->axes.pop_back();

    // translate buttons
    // f310 buttons (from): [A, B, X, Y LB, RB, BACK, START, POWER, left stick, right stick click]
    // xbox buttons (to):     [A, B, X, Y LB, RB, BACK, START, POWER, left stick, right stick click, pad L, pad R, pad U, pad D]
    remap->buttons = std::vector<int>(f310->buttons);
    remap->buttons.push_back((f310->axes[6] > 0.5) ? 1 : 0);
    remap->buttons.push_back((f310->axes[6] < -0.5) ? 1 : 0);
    remap->buttons.push_back((f310->axes[7] > 0.5) ? 1 : 0);
    remap->buttons.push_back((f310->axes[7] < -0.5) ? 1 : 0);

    sensor_msgs::Joy::ConstPtr remapped_msg_ptr(new sensor_msgs::Joy(*remap));

    return remapped_msg_ptr;

}
