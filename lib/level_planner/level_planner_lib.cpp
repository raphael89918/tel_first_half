#include "level_planner/level_planner.hpp"

LevelPlanner::LevelPlanner()
    : m_rate(60)
{
    ROS_INFO("default constructor");
}

LevelPlanner::LevelPlanner(ros::NodeHandle &nh)
    : m_nh(nh), m_rate(30)
{
    ROS_INFO("for test, not init level");
    m_color_sub = m_nh.subscribe("/ground_color", 1, &LevelPlanner::ground_color_callback, this);
    m_wheel_wait = m_nh.subscribe("/wheel/waitforidle", 1, &LevelPlanner::wheel_idle_callback, this);
    m_wheel_pub = m_nh.advertise<wheel_tokyo_weili::wheel_planner>("/wheel/planner", 1);

    ros::Duration(10).sleep();

    wheel_planner_msg_init();

    ROS_INFO("test is up");
}

LevelPlanner::LevelPlanner(LevelState level, ros::NodeHandle &nh)
    : m_current_state(level), m_nh(nh), m_rate(30)
{
    ROS_INFO("level constructed");
    m_color_sub = m_nh.subscribe("/ground_color", 1, &LevelPlanner::ground_color_callback, this);
    m_wheel_wait = m_nh.subscribe("/wheel/waitforidle", 1, &LevelPlanner::wheel_idle_callback, this);
    m_wheel_pub = m_nh.advertise<wheel_tokyo_weili::wheel_planner>("/wheel/planner", 1);

    std::vector<std::string> query_topics = {
        "/wheel/waitforidle",
        "/wheel/planner",
        "/wheel/motor",
        "/wheel/distance",
        "/cmd_vel",
        "/planner/encoder",
        //"/camera/realsense2_camera_manager/bond",
        //"/camera/color/image_raw",
        "/encoder",
    };

    while (!are_topics_ready(query_topics))
    {
        ros::Duration(1).sleep();
    }

    wheel_planner_msg_init();

    ros::Duration(10).sleep();
    ROS_INFO("Level Planner is up!!");
}

LevelPlanner::~LevelPlanner()
{
    ROS_INFO("uwu");
}

void LevelPlanner::execute()
{
    // wait for enter key
    ROS_INFO("Press enter to start");
    fscanf(stdin, "%*c");

    while (m_current_state != LevelState::TERMINATE)
    {
        switch (m_current_state)
        {
        case LevelState::LEVEL_1:
        {
            level_1();
            break;
        }

        case LevelState::LEVEL_2:
        {
            level_2();
            break;
        }

        case LevelState::LEVEL_3:
        {
            level_3();
            break;
        }
        }
    }
    ROS_INFO("Terminated");
}

void LevelPlanner::test_wheel_dist(float dist, double delay)
{
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

void LevelPlanner::test_wheel_vel(float dist, float base_speed, double secs, double delay)
{
    wheel_planner_msg_init();

    wheel_planner_msg_vel_xyz_duration(base_speed, 0, 0, secs);
    ROS_INFO("FRONT");

    ros::Duration(delay).sleep();

    wheel_planner_msg_vel_xyz_duration(-base_speed, 0, 0, secs);
    ROS_INFO("BACK");

    ros::Duration(delay).sleep();

    wheel_planner_msg_vel_xyz_duration(0, base_speed, 0, secs);
    ROS_INFO("RIGHT");

    ros::Duration(delay).sleep();

    wheel_planner_msg_vel_xyz_duration(0, -base_speed, 0, secs);
    ROS_INFO("LEFT");

    ros::Duration(delay).sleep();

    wheel_planner_msg_vel_xyz_duration(0, 0, base_speed, secs);
    ROS_INFO("SPIN_RIGHT");

    ros::Duration(delay).sleep();

    wheel_planner_msg_vel_xyz_duration(0, 0, -base_speed, secs);
    ROS_INFO("SPIN_LEFT");

    ros::Duration(delay).sleep();
}

void LevelPlanner::test_wheel_dist_vel(const float dist, const float base_speed, const double secs, const double delay)
{
    ROS_INFO("dist and vel is staggered test");

    wheel_planner_msg_dist_xyz(dist, 0, 0);
    ROS_INFO("dist FRONT");

    ros::Duration(delay).sleep();

    wheel_planner_msg_dist_xyz(-dist, 0, 0);
    ROS_INFO("dist BACK");

    ros::Duration(delay).sleep();

    wheel_planner_msg_vel_xyz_duration(base_speed, 0, 0, secs);
    ROS_INFO("vel FRONT");

    ros::Duration(delay).sleep();

    wheel_planner_msg_vel_xyz_duration(-base_speed, 0, 0, secs);
    ROS_INFO("vel BACK");

    ros::Duration(delay).sleep();

    wheel_planner_msg_dist_xyz(0, dist, 0);
    ROS_INFO("dist RIGHT");

    ros::Duration(delay).sleep();

    wheel_planner_msg_vel_xyz_duration(0, -base_speed, 0, secs);
    ROS_INFO("vel LEFT");

    ros::Duration(delay).sleep();

    wheel_planner_msg_vel_xyz_duration(0, base_speed, 0, secs);
    ROS_INFO("vel RIGHT");

    ros::Duration(delay).sleep();

    wheel_planner_msg_dist_xyz(0, -dist, 0);
    ROS_INFO("dist LEFT");

    ros::Duration(delay).sleep();

    wheel_planner_msg_dist_xyz(0, 0, dist);
    ROS_INFO("dist SPIN_RIGHT");

    ros::Duration(delay).sleep();

    wheel_planner_msg_dist_xyz(0, 0, -dist);
    ROS_INFO("dist SPIN_LEFT");

    ros::Duration(delay).sleep();

    wheel_planner_msg_vel_xyz_duration(0, 0, base_speed, secs);
    ROS_INFO("vel SPIN_RIGHT");

    ros::Duration(delay).sleep();

    wheel_planner_msg_vel_xyz_duration(0, 0, -base_speed, secs);
    ROS_INFO("vel SPIN_LEFT");

    ros::Duration(delay).sleep();

    wheel_planner_msg_stop();
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

bool LevelPlanner::are_topics_ready(const std::vector<std::string> &query_topics)
{
    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);

    std::unordered_set<std::string> topic_set;

    for (auto topic : master_topics)
    {
        topic_set.insert(topic.name);
    }

    for (auto search_topic : query_topics)
    {
        if (topic_set.find(search_topic) == topic_set.end())
        {
            ROS_INFO("Waiting for topic %s", search_topic.c_str());
            return false;
        }
    }

    return true;
}

void LevelPlanner::level_1()
{
    ROS_INFO("level1");

    first_level first_level(m_nh);
    first_level.init_pubsub();

    ROS_INFO("GO");

    int strategy = 69;

    m_nh.getParamCached("/level_planner/level_1/strategy", strategy);

    std::vector<std::string> gpio_topics = {"/laser"};
    std::vector<std::string> dynamixel_topics = {"/dynamixel/arm_storage",
                                                 "/dynamixel/wheel_laser"};
    std::vector<std::string> vision_topics = {"/alphabet", "/detectnet/detections", "/camera/depth/image_rect_raw"};

    bool exit_strat = false;

    while (!exit_strat)
    {
        switch (strategy)
        {
        case LEVEL_1_VISION:
        {
            if (!are_topics_ready(vision_topics) ||
                !are_topics_ready(gpio_topics) ||
                !are_topics_ready(dynamixel_topics))
            {
                strategy = LEVEL_1_DISTANCE;
                break;
            }
            first_level.vision_strategy();
            exit_strat = true;
            break;
        }
        case LEVEL_1_DISTANCE:
        {
            if (!are_topics_ready(gpio_topics))
            {
                strategy = LEVEL_1_NO_GPIO;
                break;
            }

            first_level.distance_strategy();
            exit_strat = true;
            break;
        }
        case LEVEL_1_NO_GPIO:
        {
            first_level.no_gpio_strategy();
            exit_strat = true;
            break;
        }
        case LEVEL_1_TEST:
        {
            first_level.test();
            exit_strat = true;
            break;
        }
        default:
        {
            ROS_ERROR("Invalid strategy");
            exit(1);
            break;
        }
        }
    }

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
        break;
    }
    case LEVEL_2_GPIO:
    {
        level_2_gpio_strategy();
        break;
    } 
    case LEVEL_2_DISTANCE:
    {
        level_2_distance_strategy();
        break;
    }
    default:
    {
        ROS_ERROR("Invalid strategy");
        exit(1);
        break;
    }
    }

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

    wheel_planner_msg_dist_xyz(140, 0, 0);
    ROS_INFO("FRONT");
    ros::Duration(2.0).sleep();
    wheel_planner_msg_dist_xyz(0, -47.5, 0);
    ROS_INFO("LEFT");

    entry_color(GROUND_GREEN);
    wheel_planner_msg_dist_xyz(125, 0, 0);
    ros::Duration(2.0).sleep();

    ROS_INFO("FRONT");
    wheel_planner_msg_dist_xyz(0, 30, 0);
    ROS_INFO("RIGHT");

    entry_color(GROUND_BLUE);
    wheel_planner_msg_dist_xyz(130, 0, 0);
    ros::Duration(2.0).sleep();

    ROS_INFO("FRONT");
    wheel_planner_msg_dist_xyz(0, -15, 0);
    ROS_INFO("LEFT");
}

void LevelPlanner::level_2_distance_strategy()
{
    ROS_INFO("level2 distance strategy");

    wheel_planner_msg_dist_xyz(20, 0, 0);
    wheel_planner_msg_dist_xyz(0, 20, 0);

    wheel_planner_msg_dist_xyz(140, 0, 0);
    ros::Duration(2.0).sleep();
    wheel_planner_msg_dist_xyz(0, -47.5, 0);

    wheel_planner_msg_dist_xyz(125, 0, 0);
    ros::Duration(2.0).sleep();
    wheel_planner_msg_dist_xyz(0, 30, 0);

    wheel_planner_msg_dist_xyz(130, 0, 0);
    ros::Duration(2.0).sleep();
    wheel_planner_msg_dist_xyz(0, -15, 0);
}

void LevelPlanner::level_2_gpio_strategy()
{
    first_level second_level(m_nh);

    second_level.init_pubsub();

    ROS_INFO("go front 20");
    second_level.robot_move(front, 20);

    ROS_INFO("go to far right");
    second_level.robot_far(right);

    ROS_INFO("go left 10");
    second_level.robot_move(left, 15);

    ROS_INFO("go front 100");
    second_level.robot_move(front, 130);

    ROS_INFO("go to far left");
    second_level.robot_far(left);

    ROS_INFO("go right 10");
    second_level.robot_move(right, 17);

    ROS_INFO("go front 120");
    second_level.robot_move(front, 135);

    ROS_INFO("go to far right");
    second_level.robot_far(right);

    ROS_INFO("go left 10");
    second_level.robot_move(left, 20);

    ROS_INFO("go front 80");
    second_level.robot_move(front, 130);

    ROS_INFO("go left 50");
    second_level.robot_move(left, 50);
}

void LevelPlanner::entry_color(int color)
{
    int x_threshold = 5, y_threshold = 5, z_threshold = 3;
    double max_time = 10.0;

    ros::spinOnce();

    m_nh.getParamCached("/level_planner/level_2/x_threshold", x_threshold);
    m_nh.getParamCached("/level_planner/level_2/y_threshold", y_threshold);
    m_nh.getParamCached("/level_planner/level_2/z_threshold", z_threshold);
    m_nh.getParamCached("/level_planner/level_2/max_time", max_time);

    ros::Time startTime = ros::Time::now();
    ros::Duration maxTime(max_time);

    // while (((abs(get_y_offset(color)) > y_threshold) ||
    //         (abs(get_z_offset(color)) > z_threshold)) &&
    //        (ros::Time::now() < startTime + maxTime))
    //{
    //     entry_color_y(color);
    //     entry_color_z(color);
    //     ros::spinOnce();
    //     m_rate.sleep();
    //     ROS_INFO("time elapsed: %f", (ros::Time::now() - startTime).toSec());
    // }

    entry_color_y(color);
    entry_color_z(color);
    ros::spinOnce();
    m_rate.sleep();
    ROS_INFO("time elapsed: %f", (ros::Time::now() - startTime).toSec());
    entry_color_y(color);
    entry_color_z(color);
    ros::spinOnce();
    m_rate.sleep();
    ROS_INFO("time elapsed: %f", (ros::Time::now() - startTime).toSec());

    wheel_planner_msg_stop();
}

void LevelPlanner::entry_color_x(int color)
{
    double clamp_max = 0.5, clamp_min = -0.5;
    double p = 0.001, i = 0.0005, d = 0;
    int x_threshold = 10, min_area = 5000;
    double max_time = 5.0;

    m_nh.getParamCached("/level_planner/level_2/entry_x/p", p);
    m_nh.getParamCached("/level_planner/level_2/entry_x/i", i);
    m_nh.getParamCached("/level_planner/level_2/entry_x/d", d);

    m_nh.getParamCached("/level_planner/level_2/x_threshold", x_threshold);
    m_nh.getParamCached("/level_planner/x_clamp_max", clamp_max);
    m_nh.getParamCached("/level_planner/x_clamp_min", clamp_min);

    m_nh.getParamCached("/level_planner/level_2/entry_color/min_area", min_area);
    m_nh.getParamCached("/level_planner/level_2/single_max_time", max_time);

    ros::Time startTime = ros::Time::now();
    ros::Duration maxTime(max_time);

    PID pid(p, i, d);

    ROS_INFO("PID: %f %f %f", p, i, d);

    ros::spinOnce();

    while ((abs(get_x_offset(color)) > x_threshold) &&
           (ros::Time::now() < startTime + maxTime))
    {
        if (m_color_msg.rect[color].rect_size < min_area)
        {
            ROS_ERROR("Entry color not found");
            exit(1);
        }

        double speed = pid.calculate(get_x_offset(color));
        wheel_planner_msg_vel_xyz(-clamp(speed, clamp_min, clamp_max), 0, 0);

        ros::spinOnce();
        m_rate.sleep();
    }

    wheel_planner_msg_stop();
}

void LevelPlanner::entry_color_y(int color)
{
    double clamp_max = 0.5, clamp_min = -0.5;
    double p = 0.001, i = 0.0005, d = 0;
    int y_threshold = 10, min_area = 5000;
    double max_time = 5.0;

    m_nh.getParamCached("/level_planner/level_2/entry_y/p", p);
    m_nh.getParamCached("/level_planner/level_2/entry_y/i", i);
    m_nh.getParamCached("/level_planner/level_2/entry_y/d", d);

    m_nh.getParamCached("/level_planner/level_2/y_threshold", y_threshold);
    m_nh.getParamCached("/level_planner/y_clamp_max", clamp_max);
    m_nh.getParamCached("/level_planner/y_clamp_min", clamp_min);
    m_nh.getParamCached("/level_planner/level_2/single_max_time", max_time);

    m_nh.getParamCached("/level_planner/level_2/entry_color/min_area", min_area);

    ros::Time startTime = ros::Time::now();
    ros::Duration maxTime(max_time);

    PID pid(p, i, d);

    ROS_INFO("PID: %f %f %f", p, i, d);

    ros::spinOnce();

    while (abs(get_y_offset(color)) > y_threshold &&
           ((ros::Time::now() < startTime + maxTime)))
    {
        if (m_color_msg.rect[color].rect_size < min_area)
        {
            ROS_ERROR("Entry color not found");
            exit(1);
        }

        double speed = pid.calculate(get_y_offset(color));
        wheel_planner_msg_vel_xyz(0, -clamp(speed, clamp_min, clamp_max), 0);

        ros::spinOnce();
        m_rate.sleep();
    }

    wheel_planner_msg_stop();
}

void LevelPlanner::entry_color_z(int color)
{
    double clamp_max = 0.5, clamp_min = -0.5;
    double p = 0.001, i = 0.0005, d = 0;
    int z_threshold = 3, min_area = 5000;
    double max_time = 5.0;

    m_nh.getParamCached("/level_planner/level_2/entry_z/p", p);
    m_nh.getParamCached("/level_planner/level_2/entry_z/i", i);
    m_nh.getParamCached("/level_planner/level_2/entry_z/d", d);

    m_nh.getParamCached("/level_planner/level_2/z_threshold", z_threshold);
    m_nh.getParamCached("/level_planner/z_clamp_max", clamp_max);
    m_nh.getParamCached("/level_planner/z_clamp_min", clamp_min);
    m_nh.getParamCached("/level_planner/level_2/single_max_time", max_time);

    m_nh.getParamCached("/level_planner/level_2/entry_color/min_area", min_area);

    PID pid(p, i, d);

    ROS_INFO("PID: %f %f %f", p, i, d);

    ros::Time startTime = ros::Time::now();
    ros::Duration maxTime(max_time);

    ros::spinOnce();

    double speed = 0;

    while (abs(get_z_offset(color)) > z_threshold &&
           ((ros::Time::now() < startTime + maxTime)))
    {
        if (m_color_msg.rect[color].rect_size < min_area)
        {
            ROS_ERROR("Entry color not found");
            exit(1);
        }

        speed = pid.calculate(get_z_offset(color));
        wheel_planner_msg_vel_xyz(0, 0, clamp(speed, clamp_min, clamp_max));

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

int LevelPlanner::get_x_offset(int color)
{
    int x = 300;
    m_nh.getParamCached("/level_planner/level_2/entry/pixel_x", x);
    int offset = m_color_msg.rect[color].ymax - x;
    ROS_INFO("ymax: %d", m_color_msg.rect[color].ymax);
    ROS_INFO("x_Offset: %d", offset);
    return offset;
}

int LevelPlanner::get_y_offset(int color)
{
    int offset = IMAGE_WIDTH / 2 - m_color_msg.rect[color].x_center;
    ROS_INFO("y_Offset: %d", offset);
    return offset;
}

int LevelPlanner::get_z_offset(int color)
{
    int offset = -m_color_msg.rect[color].angle;
    ROS_INFO("z_offset: %d", offset);
    return offset;
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
    double gain_x = 1.0, gain_y = 1.0, gain_z = 1.0;
    double bias_x = 0.0, bias_y = 0.0, bias_z = 0.0;
    double min_dis = 1.0;

    m_nh.getParamCached("/level_planner/gain_x", gain_x);
    m_nh.getParamCached("/level_planner/gain_y", gain_y);
    m_nh.getParamCached("/level_planner/gain_z", gain_z);

    m_nh.getParamCached("/level_planner/bias_x", bias_x);
    m_nh.getParamCached("/level_planner/bias_y", bias_y);
    m_nh.getParamCached("/level_planner/bias_z", bias_z);

    wheel_planner_msg_init();

    float result_x = static_cast<float>(x * gain_x);
    float result_y = static_cast<float>(y * gain_y);
    float result_z = static_cast<float>(z * gain_z);

    int sign_x = ((0 < result_x) - (result_x < 0));
    int sign_y = ((0 < result_y) - (result_y < 0));
    int sign_z = ((0 < result_z) - (result_z < 0));

    result_x = result_x - sign_x * bias_x;
    result_y = result_y - sign_y * bias_y;
    result_z = result_z - sign_z * bias_z;

    result_x = (fabs(result_x) < min_dis) ? (min_dis * sign_x) : (result_x);
    result_y = (fabs(result_y) < min_dis) ? (min_dis * sign_y) : (result_y);
    result_z = (fabs(result_z) < min_dis) ? (min_dis * sign_z) : (result_z);

    m_wheel_planner_msg.distance_x = static_cast<float>(result_x);
    m_wheel_planner_msg.distance_y = static_cast<float>(result_y);
    m_wheel_planner_msg.distance_z = static_cast<float>(result_z);
    m_wheel_pub.publish(m_wheel_planner_msg);

    ROS_INFO("sended distance: %f, %f, %f", result_x, result_y, result_z);

    wheel_planner_msg_wait();
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
    wheel_planner_msg_init();

    m_wheel_planner_msg.far_left = true;
    m_wheel_pub.publish(m_wheel_planner_msg);

    wheel_planner_msg_wait();
}

void LevelPlanner::wheel_planner_msg_far_right()
{
    wheel_planner_msg_init();

    m_wheel_planner_msg.far_right = true;
    m_wheel_pub.publish(m_wheel_planner_msg);

    wheel_planner_msg_wait();
}

void LevelPlanner::wheel_planner_msg_stop()
{
    double step_delay = 1.0;

    wheel_planner_msg_init();
    m_wheel_pub.publish(m_wheel_planner_msg);

    m_nh.getParamCached("/level_planner/step_delay", step_delay);
    ros::Duration(1).sleep();
}

void LevelPlanner::wheel_planner_msg_wait()
{
    double step_delay = 1.0;
    m_nh.getParamCached("/level_planner/step_delay", step_delay);

    ros::spinOnce();

    while (m_wheel_idle_msg.wait == false)
    {
        ROS_INFO("waiting for idle");
        ros::spinOnce();
        m_rate.sleep();
    }

    ROS_INFO("step_delay: %lf", step_delay);

    ros::Duration(step_delay).sleep();
}
