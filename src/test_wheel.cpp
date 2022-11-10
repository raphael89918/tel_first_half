#include "level_planner/level_planner.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "level_planner");
    ros::NodeHandle nh;

    double base_speed = 0.25, secs = 0.5;

    nh.getParamCached("/level_planner/test/speed", base_speed);
    nh.getParamCached("/level_planner/test/step_duration", secs);

    LevelPlanner planner(nh);

    planner.test_wheel(base_speed, secs);
}
