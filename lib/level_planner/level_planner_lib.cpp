#include "level_planner/level_planner.hpp"

LevelPlanner::LevelPlanner()
    : m_rate(60)
{
    ROS_INFO("default constructor");
}

LevelPlanner::LevelPlanner(ros::NodeHandle &nh)
    : m_nh(nh), m_rate(60)
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
    : m_current_state(level), m_nh(nh), m_rate(60)
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

void LevelPlanner::test_wheel_dist(float dist, double delay)
{
    wheel_planner_msg_stop();

    wheel_planner_msg_dist_xyz(dist, 0, 0);
    ROS_INFO("FRONT");
    ros::Duration(delay).sleep();

    wheel_planner_msg_dist_xyz(-dist, 0, 0);
    ROS_INFO("BACK");
    ros::Duration(delay).sleep();

    wheel_planner_msg_dist_xyz(0, dist, 0);
    ROS_INFO("RIGHT");
    ros::Duration(delay).sleep();

    wheel_planner_msg_dist_xyz(0, -dist, 0);
    ROS_INFO("LEFT");
    ros::Duration(delay).sleep();

    wheel_planner_msg_dist_xyz(0, 0, dist);
    ROS_INFO("SPIN_RIGHT");
    ros::Duration(delay).sleep();

    wheel_planner_msg_dist_xyz(0, 0, -dist);
    ROS_INFO("SPIN_LEFT");
    ros::Duration(delay).sleep();
}

void LevelPlanner::test_wheel(float dist, float base_speed, double secs, double delay)
{
    wheel_planner_msg_stop();

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
    ros::Duration(delay).sleep();
}

void LevelPlanner::test_entry_color(int color)
{
    entry_color(color);
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

    int strategy = 0;

    m_nh.getParamCached("/level_planner/level_2/strategy", strategy);

    switch (strategy)
    {
    case LEVEL_2_VISION:
    {
        level_2_vision_strategy();
    }
    case LEVEL_2_DISTANCE:
    {
        level_2_distance_strategy();
    }
    default:
    {
        ROS_ERROR("Invalid strategy");
    }
    }

    // level_2_vision_strategy();

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
    ROS_INFO("level2 vision strategy");
    wheel_planner_msg_dist_xyz(20, 0, 0);
    ROS_INFO("FRONT");
    wheel_planner_msg_dist_xyz(0, 20, 0);
    ROS_INFO("RIGHT");

    entry_color(GROUND_RED);
    forward_color(GROUND_RED);

    wheel_planner_msg_dist_xyz(70, 0, 0);
    ROS_INFO("FRONT");
    wheel_planner_msg_dist_xyz(0, -37.5, 0);
    ROS_INFO("LEFT");

    entry_color(GROUND_GREEN);
    forward_color(GROUND_GREEN);

    wheel_planner_msg_dist_xyz(60, 0, 0);
    ROS_INFO("FRONT");
    wheel_planner_msg_dist_xyz(0, 32.5, 0);
    ROS_INFO("RIGHT");

    entry_color(GROUND_BLUE);
    forward_color(GROUND_BLUE);

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
}

void LevelPlanner::entry_color(int color)
{
    double clamp_max = 0.5, clamp_min = -0.5;
    double p = 0.001, i = 0.0005, d = 0;
    int mid_pixel_threshold = 10, min_area = 5000;

    m_nh.getParamCached("/level_planner/level_2/entry_xy/p", p);
    m_nh.getParamCached("/level_planner/level_2/entry_xy/i", i);
    m_nh.getParamCached("/level_planner/level_2/entry_xy/d", d);

    m_nh.getParamCached("/level_planner/level_2/pixel_threshold", mid_pixel_threshold);
    m_nh.getParamCached("/level_planner/xy_clamp_max", clamp_max);
    m_nh.getParamCached("/level_planner/xy_clamp_min", clamp_min);

    m_nh.getParamCached("/level_planner/level_2/entry_color/min_area", min_area);

    PID pid(p, i, d);

    ROS_INFO("PID: %f %f %f", p, i, d);

    ros::spinOnce();
    double offset = IMAGE_WIDTH / 2.0 - m_color_msg.rect[color].x_center;
    double speed = 0;

    while (abs(offset) > mid_pixel_threshold)
    {
        if (m_color_msg.rect[color].rect_size < min_area)
        {
            ROS_ERROR("Entry color not found");
            exit(1);
        }

        offset = IMAGE_WIDTH / 2.0 - m_color_msg.rect[color].x_center;
        speed = pid.calculate(offset);
        wheel_planner_msg_vel_xyz(0, -clamp(speed, clamp_min, clamp_max), 0);
        ROS_INFO("Offset: %lf, Not clamped Speed: %lf, Clamped Speed: %lf, Color area: %d", offset, speed, clamp(speed, clamp_min, clamp_max), m_color_msg.rect[color].rect_size);

        ros::spinOnce();
        m_rate.sleep();
    }

    wheel_planner_msg_stop();
}

void LevelPlanner::forward_color(int color)
{
    // while (abs(IMAGE_WIDTH / 2 - m_color_msg.rect[color].x_center) > pixel_threshold)
    // {
    //     if (m_color_msg.rect[color].x_center < IMAGE_WIDTH / 2)
    //         wheel_planner_msg_vel_xyz(0, -base_speed, 0);
    //     else if (m_color_msg.rect[color].x_center > IMAGE_WIDTH / 2)
    //         wheel_planner_msg_vel_xyz(0, base_speed, 0);

    //     ros::Duration(0.1).sleep();
    //     ros::spinOnce();
    // }
    int complete_area_threshold = 5000, base_speed = 0.25;

    m_nh.getParamCached("/level_planner/level_2/base_speed", base_speed);
    m_nh.getParamCached("/level_planner/level_2/area_threshold", complete_area_threshold);

    while (m_color_msg.rect[color].rect_size > complete_area_threshold)
    {
        wheel_planner_msg_vel_xyz(base_speed, 0, 0);

        ros::spinOnce();
        m_rate.sleep();
    }
    wheel_planner_msg_stop();
}

double LevelPlanner::clamp(const double value, const double min, const double max)
{
    if (value < min)
        return min;
    else if (value > max)
        return max;
    else
        return value;
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

    ROS_INFO("sended distance: %f, %f, %f", x, y, z);

    ros::spinOnce();

    while (m_wheel_idle_msg.wait == false)
    {
        ROS_INFO("waiting for idle");
        ros::spinOnce();
        m_rate.sleep();
    }
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

    while (ros::Time::now() < startTime + loopDuration)
    {
        wheel_planner_msg_vel_xyz(x, y, z);
        ros::spinOnce();
        m_rate.sleep();
    }

    wheel_planner_msg_stop();
}

void LevelPlanner::wheel_planner_msg_far_left()
{
    m_wheel_planner_msg.far_left = true;
    m_wheel_pub.publish(m_wheel_planner_msg);

    ros::spinOnce();

    while (m_wheel_idle_msg.wait == false)
    {
        ROS_INFO("waiting for idle");
        ros::spinOnce();
        m_rate.sleep();
    }
}

void LevelPlanner::wheel_planner_msg_far_right()
{
    m_wheel_planner_msg.far_right = true;
    m_wheel_pub.publish(m_wheel_planner_msg);

    ros::spinOnce();

    while (m_wheel_idle_msg.wait == false)
    {
        ROS_INFO("waiting for idle");
        ros::spinOnce();
        m_rate.sleep();
    }
}

void LevelPlanner::wheel_planner_msg_stop()
{
    ROS_INFO("Stopping");
    wheel_planner_msg_init();
    m_wheel_pub.publish(m_wheel_planner_msg);
    ROS_INFO("Stopped");
}