#include "level_planner/level_planner.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "level_planner");
    ros::NodeHandle nh;

    double dist = 5, delay = 0.5;
    double base_speed = 0.25, secs = 0.5;

    nh.getParamCached("/level_planner/test/speed", base_speed);
    nh.getParamCached("/level_planner/test/duration", secs);
    nh.getParamCached("/level_planner/test/dist", dist);
    nh.getParamCached("/level_planner/test/delay", delay);

    ROS_INFO("base_speed: %f, secs: %f, dist: %f, delay: %f", base_speed, secs, dist, delay);

    LevelPlanner planner(nh);

    //planner.test_wheel(dist, base_speed, secs, delay);
    planner.test_wheel_dist(dist, delay);
}
