#include "level_planner/level_planner.hpp"

LevelPlanner::LevelPlanner()
{
    ROS_INFO("default constructor");
}

LevelPlanner::LevelPlanner(ros::NodeHandle &nh)
    : m_nh(nh)
{
    ROS_INFO("for test, not init level");
    m_color_sub = m_nh.subscribe("/ground_color", 1, &LevelPlanner::ground_color_callback, this);
    m_wheel_wait = m_nh.subscribe("/wheel/waitforidle", 1, &LevelPlanner::wheel_idle_callback, this);
    m_wheel_pub = m_nh.advertise<wheel_tokyo_weili::wheel_planner>("/wheel/planner", 1);

    ros::Duration(10).sleep();

    wheel_planner_msg_stop();

    ROS_INFO("test is up");
}

LevelPlanner::LevelPlanner(LevelState level, ros::NodeHandle &nh)
    : m_nh(nh), m_current_state(level)
{
    ROS_INFO("level constructed");
    m_color_sub = m_nh.subscribe("/ground_color", 1, &LevelPlanner::ground_color_callback, this);
    m_wheel_wait = m_nh.subscribe("/wheel/waitforidle", 1, &LevelPlanner::wheel_idle_callback, this);
    m_wheel_pub = m_nh.advertise<wheel_tokyo_weili::wheel_planner>("/wheel/planner", 1);

    ros::Duration(10).sleep();

    wheel_planner_msg_stop();

    ROS_INFO("Level Planner is up!!");
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

void LevelPlanner::test_wheel(float base_speed, double secs)
{
    wheel_planner_msg_init();

    wheel_planner_msg_vel_xyz_duration(base_speed, 0, 0, secs);
    ROS_INFO("FRONT");

    wheel_planner_msg_vel_xyz_duration(-base_speed, 0, 0, secs);
    ROS_INFO("BACK");

    wheel_planner_msg_vel_xyz_duration(0, base_speed, 0, secs);
    ROS_INFO("RIGHT");

    wheel_planner_msg_vel_xyz_duration(0, -base_speed, 0, secs);
    ROS_INFO("LEFT");

    wheel_planner_msg_vel_xyz_duration(0, 0, base_speed, secs);
    ROS_INFO("SPIN_RIGHT");

    wheel_planner_msg_vel_xyz_duration(0, 0, -base_speed, secs);
    ROS_INFO("SPIN_LEFT");

    wheel_planner_msg_stop();
}

void LevelPlanner::ground_color_callback(const ground_color::GroundColor &color_msg)
{
    m_color_msg = color_msg;
}

void LevelPlanner::wheel_idle_callback(const wheel_tokyo_weili::waitforidle &wheel_idle_msg)
{
    m_wheel_idle_msg = wheel_idle_msg;
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
    wheel_planner_msg_init();

    // level_2_vision_strategy();
    level_2_distance_strategy();

    m_current_state = LevelState::LEVEL_3;
}

void LevelPlanner::level_3()
{
    ROS_INFO("level3");
    /*

    */
    m_current_state = LevelState::TERMINATE;
}

void LevelPlanner::level_2_vision_strategy()
{
    double base_speed = 0.25;
    int area_threashold = 10000;
    int pixel_threshold = 100;

    ROS_INFO("level2 vision strategy");
    wheel_planner_msg_dist_xyz(20, 0, 0);
    ROS_INFO("FRONT");
    wheel_planner_msg_dist_xyz(0, 20, 0);
    ROS_INFO("LEFT");

    entry_color(GROUND_RED, base_speed, pixel_threshold);
    forward_color(GROUND_RED, base_speed, pixel_threshold, area_threashold);

    wheel_planner_msg_dist_xyz(70, 0, 0);
    ROS_INFO("FRONT");
    wheel_planner_msg_dist_xyz(0, -37.5, 0);
    ROS_INFO("LEFT");

    entry_color(GROUND_GREEN, base_speed, pixel_threshold);
    forward_color(GROUND_GREEN, base_speed, pixel_threshold, area_threashold);

    wheel_planner_msg_dist_xyz(60, 0, 0);
    ROS_INFO("FRONT");
    wheel_planner_msg_dist_xyz(0, 32.5, 0);
    ROS_INFO("RIGHT");

    entry_color(GROUND_BLUE, base_speed, pixel_threshold);
    forward_color(GROUND_BLUE, base_speed, pixel_threshold, area_threashold);

    wheel_planner_msg_dist_xyz(40, 0, 0);
    ROS_INFO("FRONT");
    wheel_planner_msg_dist_xyz(0, -15, 0);
    ROS_INFO("LEFT");
}

void LevelPlanner::level_2_distance_strategy()
{
    ROS_INFO("level2 distance strategy");

    wheel_planner_msg_dist_xyz(20, 0, 0);
    wheel_planner_msg_dist_xyz(0, 20, 0);

    wheel_planner_msg_dist_xyz(50, 0, 0);

    wheel_planner_msg_dist_xyz(70, 0, 0);
    wheel_planner_msg_dist_xyz(0, -37.5, 0);

    wheel_planner_msg_dist_xyz(45, 0, 0);

    wheel_planner_msg_dist_xyz(60, 0, 0);
    wheel_planner_msg_dist_xyz(0, 32.5, 0);

    wheel_planner_msg_dist_xyz(40, 0, 0);

    wheel_planner_msg_dist_xyz(40, 0, 0);
    wheel_planner_msg_dist_xyz(0, -15, 0);

    wheel_planner_msg_dist_xyz(55, 0, 0);

    wheel_planner_msg_stop();
}

void LevelPlanner::entry_color(int color, float base_speed, int mid_pixel_threshold)
{
    ros::Rate loop_rate(60);

    do
    {
        ROS_INFO("Center: %d", m_color_msg.rect[color].x_center);
        if (m_color_msg.rect[color].x_center < IMAGE_WIDTH / 2)
            wheel_planner_msg_vel_xyz(0, -base_speed, 0);
        else if (m_color_msg.rect[color].x_center > IMAGE_WIDTH / 2)
            wheel_planner_msg_vel_xyz(0, base_speed, 0);
        ros::spinOnce();
        loop_rate.sleep();
    } while (abs(IMAGE_WIDTH / 2 - m_color_msg.rect[color].x_center) > mid_pixel_threshold);

    wheel_planner_msg_stop();
}

void LevelPlanner::forward_color(int color, float base_speed, int pixel_threshold, int complete_area_threshold)
{
    ros::Rate loop_rate(60);
    // while (abs(IMAGE_WIDTH / 2 - m_color_msg.rect[color].x_center) > pixel_threshold)
    // {
    //     if (m_color_msg.rect[color].x_center < IMAGE_WIDTH / 2)
    //         wheel_planner_msg_vel_xyz(0, -base_speed, 0);
    //     else if (m_color_msg.rect[color].x_center > IMAGE_WIDTH / 2)
    //         wheel_planner_msg_vel_xyz(0, base_speed, 0);

    //     ros::Duration(0.1).sleep();
    //     ros::spinOnce();
    // }
    do
    {
        wheel_planner_msg_vel_xyz(base_speed, 0, 0);
        ros::spinOnce();
        loop_rate.sleep();
    } while (m_color_msg.rect[color].rect_size > complete_area_threshold);

    wheel_planner_msg_stop();
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
    wheel_planner_msg_init();

    m_wheel_planner_msg.distance_x = x;
    m_wheel_planner_msg.distance_y = y;
    m_wheel_planner_msg.distance_z = z;
    m_wheel_pub.publish(m_wheel_planner_msg);

    ROS_INFO("sended distance: %d, %d, %d");

    ros::Rate loop_rate(60);

    do
    {
        ROS_INFO("waiting for idle");
        ros::spinOnce();
        loop_rate.sleep();
    } while (m_wheel_idle_msg.wait == false);

    wheel_planner_msg_stop();
}

void LevelPlanner::wheel_planner_msg_vel_xyz(const float x, const float y, const float z)
{
    wheel_planner_msg_init();

    m_wheel_planner_msg.velocity_x = x;
    m_wheel_planner_msg.velocity_y = y;
    m_wheel_planner_msg.velocity_z = z;
    m_wheel_pub.publish(m_wheel_planner_msg);
}

void LevelPlanner::wheel_planner_msg_vel_xyz_duration(const float x, const float y, const float z, const double secs)
{
    wheel_planner_msg_init();

    ros::Time startTime = ros::Time::now();
    ros::Duration loopDuration(secs);
    ros::Rate loop_rate(60);

    while (ros::Time::now() < startTime + loopDuration)
    {
        wheel_planner_msg_vel_xyz(x, y, z);
        ros::spinOnce();
        loop_rate.sleep();
    }

    wheel_planner_msg_stop();
}

void LevelPlanner::wheel_planner_msg_far_left()
{
    m_wheel_planner_msg.far_left = true;
    m_wheel_pub.publish(m_wheel_planner_msg);

    ros::Rate loop_rate(60);

    do
    {
        ROS_INFO("waiting for idle");
        ros::spinOnce();
        loop_rate.sleep();
    } while (m_wheel_idle_msg.wait == false);

    wheel_planner_msg_stop();
}

void LevelPlanner::wheel_planner_msg_far_right()
{
    m_wheel_planner_msg.far_right = true;
    m_wheel_pub.publish(m_wheel_planner_msg);

    ros::Rate loop_rate(60);

    do
    {
        ROS_INFO("waiting for idle");
        ros::spinOnce();
        loop_rate.sleep();
    } while (m_wheel_idle_msg.wait == false);

    wheel_planner_msg_stop();
}

void LevelPlanner::wheel_planner_msg_stop()
{
    wheel_planner_msg_init();
    m_wheel_pub.publish(m_wheel_planner_msg);
}