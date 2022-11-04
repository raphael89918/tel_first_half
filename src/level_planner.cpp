#include "level_planner/level_planner.hpp"
#include <unordered_map>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "level_planner");
    ros::NodeHandle nh;

    int level_code = 1;
    nh.getParam("level_code", level_code);

    std::unordered_map<int, LevelState> code_map{{1, LevelState::LEVEL_1}, {2, LevelState::LEVEL_2}, {3, LevelState::LEVEL_3}};

    LevelPlanner planner(code_map[level_code], nh);
    planner.execute();
}