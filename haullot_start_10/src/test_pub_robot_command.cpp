#include "./haullot_robot_control.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "test_pub_robot_command");

    haullotRobotControl controller;

    /* control car motion */
    /* unit meter, degree */
    std::vector<std::vector<double>> car_motion_data = {
        /* 
        x,y,rotation
         */
        {0,0,0},
        {1,0,0},
        {2,0.5,45},

    };
    controller.setMaxCarVelocity(0.2); /* unit: meter */
    controller.controlCarMotion(car_motion_data);
    /* control car motion end */

    /* control lifting mechanism up motion */
    /* unit: meter */
    std::vector<std::vector<double>> lifting_mechanism_motion_up_data = {
        /* 
        "Haullot_start_10_lift_joint_1","Haullot_start_10_lift_joint_2",\
        "Haullot_start_10_lift_joint_3","Haullot_start_10_lift_joint_4" 
         */
        {0,0,0,0},
        {0.5, 0.5, 0.5, 0.5, 0.5},
        {0.95,0.95,0.95,0.95},
    };
    controller.setMaxLiftTranslationVelocity(0.2); /* unit: meter */
    controller.controlLiftMechanismUpMotion(lifting_mechanism_motion_up_data);
    /* control lifting mechanism up motion end */

    /* control lifting mechanism rotate motion */
    /* unit: degree */
    std::vector<std::vector<double>> lifting_mechanism_motion_rotate_data = {
        /*
        "Haullot_start_10_rotate_joint"
        */
        {0},
        {45},
        {90},
    };
    controller.setMaxLiftRatateVelocity(30); /* unit: degree*/
    controller.controlLiftMechanismRotateMotion(    \
        lifting_mechanism_motion_rotate_data);
    /* control lifting mechanism rotate motion end */

    /* control guide rail motion */
    /* unit: meter */
    std::vector<std::vector<double>> guide_rail_motion_data = {
        /*
        "guide_rail_sliding_joint"
        */
        {0},
        {0.2},
        {-0.2},
    };
    controller.setMaxGuideRailVelocity(0.1); /* unit: meter */
    controller.controlGuideRailMotion(guide_rail_motion_data);
    /* control guide rail motion end */

    /* control robot motion */
    /* unit: degree */
    std::vector<std::vector<double>> robot_motion_data = {
        /*
        "joint_a1", "joint_a2", "joint_a3", "joint_a4", "joint_a5", "joint_a6"
        */

        {0,0,0,0,0,0},

        {   130.63437728982768,
            -74.48451336700703,
            -40.10704565915762,
            -110.00789666511805,
            9.740282517223996,
            -156.41747807071474},

        {   108.28902327972558,
            -38.96113006889598,
            -46.9825392007275,
            -37.24225668350351,
            23.49126960036375,
            132.35325067522018},
    };
    controller.setMaxRobotJointVelocity(40); /* unit: degree */
    controller.controlRobotMotion(robot_motion_data);
    /* control robot motion end */

    return 0;
}
