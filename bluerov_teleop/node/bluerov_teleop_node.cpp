/*
 * File: bluerov_teleop/node/bluerov_teleop_node.cpp
 * Author: Antonella Wilby <antonella@explorationlabs.org>
 * Date: Feb. 2020
 * Description: This software provides remote control (teleoperation) of the BlueROV2.
 *              This software was modified from the original teleoperation code in
 *              the original bluerov_apps ROS package.
 */

#include <BluerovTeleop.h>


/*
 * Initializes teleop node.
 */
int main(int argc, char ** argv) {

    // Initialize ROS node, including joystick parameter with current joystick type and joystick topic
    ros::init(argc, argv, "bluerov_teleop");

    ros::NodeHandle nh_;

    BluerovTeleop bluerov_teleop(&nh_);
    bluerov_teleop.spin();


    //ros::spin();

    return 0;

}
