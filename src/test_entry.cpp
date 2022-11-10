#include "level_planner/level_planner.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "level_planner");
    ros::NodeHandle nh;
    int pixel_threshold = 100, color_code=0;

    nh.getParamCached("/level_planner/test/color_code", color_code);
    
    LevelPlanner planner(nh);

    planner.test_entry_color(color_code);
}