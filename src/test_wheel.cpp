#include "level_planner/level_planner.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "level_planner");
    ros::NodeHandle nh;

    LevelPlanner planner(nh);
    planner.test_wheel(0.35, 2);
}
