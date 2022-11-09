#include "level_planner/level_planner.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "level_planner");
    ros::NodeHandle nh;

    LevelPlanner planner(nh);
    int pixel_threshold = 100;

    nh.getParamCached("/level_planner/level_2/vision/pixel_threshold", pixel_threshold);
    planner.test_wheel(0.35, 2);
    // planner.test_entry_color(GROUND_RED, pixel_threshold);
}
