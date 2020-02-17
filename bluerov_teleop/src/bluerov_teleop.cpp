/*
 * File: bluerov_teleop/src/bluerov_teleop.cpp
 * Author: Antonella Wilby <antonella@explorationlabs.org>
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
    nh.param<std::string>("initial_mode", initial_mode, "manual");
    nh.param<bool>("audible_responses", do_audible_responses, "true");

    // Set up dynamic reconfigure server
    dynamic_reconfigure::Server<bluerov_teleop::bluerov_teleopConfig>::CallbackType f;
    f = boost::bind(&BluerovTeleop::configCallback, this, _1, _2);
    server.setCallback(f);

    // Subscribe to incoming joystick commands
    joy_sub = nh_.subscribe<sensor_msgs::Joy>(joy_topic, 1, &BluerovTeleop::joy_callback, this);

    // Publish thrust commands on mavros rc_override
    rc_override_pub = nh_.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 1);

    // Mavros command services
    cmd_client = nh_.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");
    set_mode = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    // SoundClient for speaking sound responses.

    // Initial state of vehicle
    camera_tilt = CAM_TILT_RESET;
    initLT = false;
    initRT = false;
    lights_level = 0;

    // Initial flight mode: manual, stabilized, depth hold mode
    if (initial_mode == "manual") {
        mode = MODE_MANUAL;
    } else if (initial_mode == "stabilized") {
        mode = MODE_STABILIZE;
    } else if (initial_mode == "depth") {
        mode = MODE_DEPTH_HOLD;
    } else {
        mode = MODE_MANUAL;
    }


    ROS_INFO("BlueROV teleoperation ready: %s mode.", mode.c_str());

}


void BluerovTeleop::spin() {
    ros::Rate loop(config.pub_rate);

    while(ros::ok()) {
        ros::spinOnce();
        loop.sleep();
    }
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

    // RC OVERRIDE MESSAGE
    mavros_msgs::OverrideRCIn msg;

    // Joystick input
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
    if (buttonPress(joy, config.disarm_button)) {     // bluerov_teleop
       requestArm(false);

   } else if(buttonPress(joy, config.arm_button)) {  // bluerov_teleop
        requestArm(true);

    }

    // MODE SWITCHING: Manual, stabilize, depth hold
    if (buttonPress(joy, config.manual_button)) {
        mode = MODE_MANUAL;
        setMode(mode);

    } else if (buttonPress(joy, config.stabilize_button)) {
        mode = MODE_STABILIZE;
        setMode(mode);

    } else if (buttonPress(joy, config.depth_hold_button)) {
        mode = MODE_DEPTH_HOLD;
        setMode(mode);
    }


    // LIGHTS: on/off
    if (buttonPress(joy, config.lights_increase)) {
        lightsOnOff(true);      // true = light brightness increase

    } else if (buttonPress(joy, config.lights_decrease)) {
        lightsOnOff(false);     // false = light brightness decrease
    }


    // CAMERA TILT: reset to origin
    /*if (buttonPress(joy, config.cam_tilt_reset)) {
        camera_tilt = CAM_TILT_RESET;
        ROS_INFO("Resetting camera position.");

    // CAMERA TILT: up
    } else if (buttonPress(joy, config.cam_tilt_up)) {
        //ROS_INFO("Tilting camera up.");
        //camera_tilt = camera_tilt + config.cam_tilt_step;

        if (camera_tilt > PPS_MAX) {
            camera_tilt = PPS_MAX;
        }

    // CAMERA TILT: down
    } else if (buttonPress(joy, config.cam_tilt_down)) {
        //ROS_INFO("tilting camera down.");
        //camera_tilt = camera_tilt - config.cam_tilt_step;

        if (camera_tilt < PPS_MIN) {
            camera_tilt = PPS_MIN;
        }

    }*/





    // Auto descend/ascend
    if (buttonPress(joy, config.autoascend_button)) {
        autoDescendAscend(false);
    } else if (buttonPress(joy, config.autodescend_button)) {
        autoDescendAscend(true);
    }





    // Remember current button states for future comparison
    previous_buttons = std::vector<int>(joy->buttons);


    // THRUSTER CONTROL: forward, strafe, throttle, roll, pitch, yaw
    // Channel mappings still like this? https://www.ardusub.com/operators-manual/rc-input-and-output.html
    /*msg.channels[5] = mapToPpm(config.x_scaling  * computeAxisValue(joy, config.x_axis,  config.expo)); // forward  (x)
    msg.channels[6] = mapToPpm(config.y_scaling  * computeAxisValue(joy, config.y_axis,  config.expo)); // strafe   (y)
    msg.channels[2] = mapToPpm(config.z_scaling  * computeAxisValue(joy, config.z_axis,  config.expo)); // throttle (z)
    msg.channels[1] = mapToPpm(config.wx_scaling * computeAxisValue(joy, config.wx_axis, config.expo)); // roll     (wx)
    msg.channels[0] = mapToPpm(config.wy_scaling * computeAxisValue(joy, config.wy_axis, config.expo)); // pitch    (wy)
    msg.channels[3] = mapToPpm(config.wz_scaling * computeAxisValue(joy, config.wz_axis, config.expo)); // yaw      (wz)*/

    msg.channels[4] = mapToPpm(config.x_scaling  * computeAxisValue(joy, config.x_axis,  config.expo)); // forward  (x)
    msg.channels[5] = mapToPpm(config.y_scaling  * computeAxisValue(joy, config.y_axis,  config.expo)); // strafe   (y)
    msg.channels[2] = mapToPpm(config.z_scaling  * computeAxisValue(joy, config.z_axis,  config.expo)); // throttle (z)
    msg.channels[1] = mapToPpm(config.wx_scaling * computeAxisValue(joy, config.wx_axis, config.expo)); // roll     (wx)
    msg.channels[0] = mapToPpm(config.wy_scaling * computeAxisValue(joy, config.wy_axis, config.expo)); // pitch    (wy)
    msg.channels[3] = mapToPpm(config.wz_scaling * computeAxisValue(joy, config.wz_axis, config.expo)); // yaw      (wz)

    // MODE AND CAMERA CONTROL
    // channel 6 unused, we don't have camera pan
    //msg.channels[4] = mode; // mode       // why is this channel 4?? why not command set?
    //msg.channels[7] = camera_tilt; // camera tilt

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
bool BluerovTeleop::buttonPress(const sensor_msgs::Joy::ConstPtr& joy, int index) {
    return (joy->buttons[index] == 1 && previous_buttons[index] == 0);
}


/*
 * Sends a request to mavros CommandLong service to arm the robot.
 */
void BluerovTeleop::requestArm(bool arm_input) {

    mavros_msgs::CommandLong srv;
    srv.request.command = COMPONENT_ARM_DISARM;
    srv.request.param1 = (arm_input ? 1 : 0);
    srv.request.param2 = 21196; // force disarm (see GCS_Mavlink.cpp)


    //bluerov_robot::Arm srv;
    //srv.request.arm = arm_input;

    // Call bluerov_robot arming service (pixhawk_bridge handles logging info)
    if (cmd_client.call(srv)) {
        ROS_INFO (arm_input ? "Armed" : "Disarmed");

        if (arm_input == true) {        // TODO: NEED NEW THREAD
            //sc.say("Armed");
        } else {
            //sc.say("Disarmed");
        }

    } else {
        ROS_ERROR(arm_input ? "FAILED TO ARM.": "WARNING! FAILED TO DISARM! ");
    }

}

void BluerovTeleop::setMode(std::string new_mode) {

    mavros_msgs::SetMode srv;
    srv.request.base_mode = 0;
    srv.request.custom_mode = new_mode;

    // Send request to service
    if (set_mode.call(srv)) {
        ROS_INFO("Entered %s Flight Mode.", new_mode.c_str());
    } else {
        ROS_ERROR("Failed to set flight mode!");
    }

}


void BluerovTeleop::lightsOnOff(bool light_increase) {

    if (light_increase) {
        lights_level++;

        if (lights_level >= 4) {
            lights_level = 4;
        }
    } else {
        lights_level--;
        if (lights_level <= 0) {
            lights_level = 0;
        }
    }


    mavros_msgs::CommandLong srv;
    srv.request.command = DO_SET_SERVO;
    srv.request.param1 = LIGHTS_AUX_CHAN;
    srv.request.param2 = 1000;

    if(cmd_client.call(srv)) {

        ROS_INFO("Light Brightness: %s%%", std::to_string(lights_level*25).c_str());

    } else {
        ROS_ERROR("Couldn't change lights.");
    }

}

void BluerovTeleop::autoDescendAscend(bool autodepth) {

    mavros_msgs::CommandLong srv;
    srv.request.command = (autodepth ? NAV_TAKEOFF_LOCAL : NAV_LAND_LOCAL);
    srv.request.param1 = 0;
    srv.request.param2 = 0;
    srv.request.param3 = config.scend_rate;
    srv.request.param4 = 0;
    srv.request.param5 = 0;
    srv.request.param6 = 0;
    srv.request.param7 = (autodepth ? config.auto_depth : 0);


    if(cmd_client.call(srv)) {
        ROS_INFO(autodepth ? "Auto-Descending" : "Auto-Ascending");
    }
    else {
        ROS_ERROR("Failed to request auto ascend/descend.");
    }

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
