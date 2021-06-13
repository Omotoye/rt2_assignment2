#include "ros/ros.h"
#include "rt2_assignment2/RandomPosition.h"

double randMToN(double M, double N)
{
    return M + (rand() / (RAND_MAX / (N - M)));
}

float rand_x;
float rand_y;
float theta;
bool new_target = false;

bool myrandom(rt2_assignment2::RandomPosition::Request &req, rt2_assignment2::RandomPosition::Response &res)
{
    rand_x = randMToN(req.x_min, req.x_max);
    rand_y = randMToN(req.y_min, req.y_max);
    theta = randMToN(-3.14, 3.14);
    new_target = true;
    res.x = rand_x;
    res.y = rand_y;
    res.theta = theta;
    return true;
}

bool ui_handle(rt2_assignment2::RandomPosition::Request &req, rt2_assignment2::RandomPosition::Response &res)
{
    while (new_target == false)
    {
        // sleep for half a second
        ros::Duration(0, 500000000).sleep();
    }
    res.x = rand_x;
    res.y = rand_y;
    res.theta = theta;
    new_target = false;
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "random_position_server");
    ros::Time::init();
    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("/position_server", myrandom);
    ros::ServiceServer service2 = n.advertiseService("ui_target", ui_handle);
    ros::spin();

    return 0;
}