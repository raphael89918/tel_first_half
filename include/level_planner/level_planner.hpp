#ifndef _LEVEL_PLANNER_
#define _LEVEL_PLANNER_

#include <ros/ros.h>
#include <ground_color/GroundColor.h>
#include <wheel_tokyo_weili/wheel_planner.h>
#include <wheel_tokyo_weili/waitforidle.h>
#include "pid/pid.hpp"

const int IMAGE_WIDTH = 1280;
const int IMAGE_HEIGHT = 720;

enum Color
{
    GROUND_RED = 0,
    GROUND_GREEN = 1,
    GROUND_BLUE = 2,
    COMBINED_BLACK = 3,
    CONBINED_BLUE = 4
};

enum class LevelState
{
    LEVEL_1,
    LEVEL_2,
    LEVEL_3,
    TERMINATE
};

enum Level_2_Strategy
{
    LEVEL_2_VISION = 0,
    LEVEL_2_DISTANCE = 1
};

class LevelPlanner
{
public:
    LevelPlanner();
    LevelPlanner(ros::NodeHandle &nh);
    LevelPlanner(LevelState level, ros::NodeHandle &nh);
    ~LevelPlanner();

    void execute();

    void test_wheel_vel(float dist, float base_speed, double secs, double delay);
    void test_wheel_dist(float dist, double delay);
    void test_wheel_dist_vel(const float dist, const float base_speed, const double secs, const double delay);
    void test_entry_color(int color);

private:
    LevelState m_current_state;
    ros::NodeHandle m_nh;

    ros::Subscriber m_color_sub;
    ros::Subscriber m_wheel_wait;
    ros::Publisher m_wheel_pub;

    ros::Rate m_rate;

    ground_color::GroundColor m_color_msg;

    wheel_tokyo_weili::waitforidle m_wheel_idle_msg;
    wheel_tokyo_weili::wheel_planner m_wheel_planner_msg;

    void ground_color_callback(const ground_color::GroundColor &color_msg);
    void wheel_idle_callback(const wheel_tokyo_weili::waitforidle &wheel_idle_msg);

    void level_1();
    void level_2();
    void level_3();

    void level_2_vision_strategy();
    void level_2_distance_strategy();

    void entry_color(int color);
    void forward_color(int color);

    double clamp(const double value, const double min, const double max);

    void wheel_planner_msg_init();

    void wheel_planner_msg_dist_xyz(const float x, const float y, const float z);
    void wheel_planner_msg_vel_xyz(const float x, const float y, const float z); // remember to set it zero
    void wheel_planner_msg_vel_xyz_duration(const float x, const float y, const float z, const double secs);
    void wheel_planner_msg_far_left();
    void wheel_planner_msg_far_right();
    void wheel_planner_msg_stop();
};

#endif