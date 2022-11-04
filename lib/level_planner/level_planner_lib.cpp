#include "level_planner/level_planner.hpp"

LevelPlanner::LevelPlanner()
{
    ROS_INFO("default constructor");
}

LevelPlanner::LevelPlanner(LevelState level, ros::NodeHandle &nh)
    : m_nh(nh), m_current_state(level)
{
    ROS_INFO("level constructed");
    m_color_sub = m_nh.subscribe("/ground_color", 1, &LevelPlanner::ground_color_callback, this);
    m_wheel_wait = m_nh.subscribe("/wheel/waitforidle", 1, &LevelPlanner::wheel_idle_callback, this);
    m_wheel_pub = m_nh.advertise<wheel_tokyo_weili::wheel_planner>("/wheel/wheel_planner", 1);
}

LevelPlanner::~LevelPlanner()
{
    ROS_INFO("uwu");
}

void LevelPlanner::execute()
{
    while (m_current_state != LevelState::TERMINATE)
    {
        switch (m_current_state)
        {
        case LevelState::LEVEL_1:
            level_1();
            break;

        case LevelState::LEVEL_2:
            level_2();
            break;

        case LevelState::LEVEL_3:
            level_3();
            break;
        }
    }
    ROS_INFO("Terminated");
}

void LevelPlanner::ground_color_callback(const ground_color::GroundColor &color_msg)
{
    m_color_msg = color_msg;
}

void LevelPlanner::wheel_idle_callback(const wheel_tokyo_weili::waitforidle &wheel_idle_msg)
{
    m_wheel_idle_msg = wheel_idle_msg;
}

void LevelPlanner::entry_color(int color, float base_speed, int pixel_threshold)
{
    while (abs(IMAGE_WIDTH / 2 - m_color_msg.rect[color].x_center) > pixel_threshold)
    {
        if (m_color_msg.rect[GROUND_RED].x_center < IMAGE_WIDTH / 2)
            wheel_planner_msg_vel_xyz(0, -base_speed, 0);
        else
            wheel_planner_msg_vel_xyz(0, base_speed, 0);
        wheel_planner_msg_init();
        ros::spinOnce();
    }
}

void LevelPlanner::forward_color(int color, float base_speed, int pixel_threshold)
{
    while (abs(IMAGE_WIDTH / 2 - m_color_msg.rect[color].x_center) > pixel_threshold)
    {
        if (m_color_msg.rect[GROUND_RED].x_center < IMAGE_WIDTH / 2)
            wheel_planner_msg_vel_xyz(0, -base_speed, 0);
        else
            wheel_planner_msg_vel_xyz(0, base_speed, 0);
        wheel_planner_msg_init();

        wheel_planner_msg_dist_xyz(base_speed, 0, 0);
        wheel_planner_msg_init();

        ros::spinOnce();
    }
}

void LevelPlanner::level_1()
{
    ROS_INFO("level1");
    /*

    */
    m_current_state = LevelState::LEVEL_2;
}

void LevelPlanner::level_2()
{
    ROS_INFO("level2");

    wheel_planner_msg_dist_xyz(30, 0, 0);
    wheel_planner_msg_dist_xyz(0, 20, 0);

    entry_color(GROUND_RED, 0.25, 10);
    forward_color(GROUND_RED, 0.25, 10000);

    wheel_planner_msg_dist_xyz(50, 0, 0);
    wheel_planner_msg_dist_xyz(0, -52.5, 0);

    entry_color(GROUND_GREEN, 0.25, 10);
    forward_color(GROUND_GREEN, 0.25, 10000);

    wheel_planner_msg_dist_xyz(60, 0, 0);
    wheel_planner_msg_dist_xyz(0, 42.5, 0);

    entry_color(GROUND_BLUE, 0.25, 10);
    forward_color(GROUND_BLUE, 0.25, 10000);

    wheel_planner_msg_dist_xyz(45, 0, 0);
    wheel_planner_msg_dist_xyz(0, -10, 0);

    m_current_state = LevelState::LEVEL_3;
}

void LevelPlanner::level_3()
{
    ROS_INFO("level3");
    /*

    */
    m_current_state = LevelState::TERMINATE;
}

void LevelPlanner::wheel_planner_msg_init()
{
    m_wheel_planner_msg.distance_x = 0;
    m_wheel_planner_msg.distance_y = 0;
    m_wheel_planner_msg.distance_z = 0;
    m_wheel_planner_msg.velocity_x = 0;
    m_wheel_planner_msg.velocity_y = 0;
    m_wheel_planner_msg.velocity_z = 0;
    m_wheel_planner_msg.far_left = false;
    m_wheel_planner_msg.far_right = false;
}

void LevelPlanner::wheel_planner_msg_dist_xyz(const float x, const float y, const float z)
{
    m_wheel_planner_msg.distance_x = x;
    m_wheel_planner_msg.distance_y = y;
    m_wheel_planner_msg.distance_z = z;
    m_wheel_pub.publish(m_wheel_planner_msg);
    wheel_planner_msg_init();
    while (m_wheel_idle_msg.wait == true)
    {
        ros::spinOnce();
    }
}

void LevelPlanner::wheel_planner_msg_vel_xyz(const float x, const float y, const float z)
{
    m_wheel_planner_msg.velocity_x = x;
    m_wheel_planner_msg.velocity_y = y;
    m_wheel_planner_msg.velocity_z = z;
    m_wheel_pub.publish(m_wheel_planner_msg);
}

void LevelPlanner::wheel_planner_msg_far_left()
{
    m_wheel_planner_msg.far_left = true;
    m_wheel_pub.publish(m_wheel_planner_msg);
    wheel_planner_msg_init();
    while (m_wheel_idle_msg.wait == true)
    {
        ros::spinOnce();
    }
}

void LevelPlanner::wheel_planner_msg_far_right()
{
    m_wheel_planner_msg.far_right = true;
    m_wheel_pub.publish(m_wheel_planner_msg);
    wheel_planner_msg_init();
    while (m_wheel_idle_msg.wait == true)
    {
        ros::spinOnce();
    }
}