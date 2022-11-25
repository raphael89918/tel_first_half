#include "first_level_planner/first_level_planner.hpp"

first_level::first_level(const ros::NodeHandle &nh) : nh(nh), direction(0)
{
    ROS_INFO("first_level constructed");
}

void first_level::init_pubsub()
{
    wheel_pub = nh.advertise<wheel_tokyo_weili::wheel_planner>("/wheel/planner", 10);
    arm_pub = nh.advertise<dynamixel_control::arm_trunk>("/dynamixel/arm_storage", 10);
    wait_sub = nh.subscribe("/wheel/waitforidle", 1, &first_level::wait_callback, this);
    visual_sub = nh.subscribe("/alphabet", 1, &first_level::visual_callback, this);
    msg_init();
    waitforidle = false;
}

void first_level::wait_callback(const wheel_tokyo_weili::waitforidle &msg)
{
    this->waitforidle = msg.wait;
}

void first_level::robot_far(uint8_t dir)
{
    direction = dir;
    switch (direction)
    {
    case left:
        this->wheel_msg.far_left = true;
        break;

    case right:
        this->wheel_msg.far_right = true;
        break;

    case front:
        this->wheel_msg.far_front = true;
        break;
    }
    wheel_pub.publish(wheel_msg);
    ros::Duration(0.1).sleep();
    robot_wait();
    msg_init();
}
void first_level::robot_wait()
{
    this->waitforidle = false;
    while (this->waitforidle == false)
    {
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }
    ros::Duration(1).sleep();
}
void first_level::msg_init()
{
    wheel_msg.distance_x = 0;
    wheel_msg.distance_y = 0;
    wheel_msg.distance_z = 0;
    wheel_msg.far_front = false;
    wheel_msg.far_left = false;
    wheel_msg.far_right = false;
    wheel_msg.velocity_x = 0;
    wheel_msg.velocity_y = 0;
    wheel_msg.velocity_z = 0;
    arm_msg.control = 0;
    // wheel_pub.publish(wheel_msg);
}

void first_level::visual_callback(const ros_deep_learning::alphabet &msg)
{
    T_x = msg.T.x;
    E_x = msg.E.x;
    L_x = msg.L.x;
    C_x = msg.C.x;
    F_x = msg.F.x;

    T_z = msg.T.z;
    E_z = msg.E.z;
    L_z = msg.L.z;
    C_z = msg.C.z;
    F_z = msg.F.z;
}

void first_level::choose_target()
{
    ros::spinOnce();
    if (T_z == 0 && E_z == 0 && L_z == 0)
    {
        ROS_INFO("no TEL target");
        return;
    }
    if (T_z >= E_z && T_z >= L_z)
    {
        if (E_z < L_z && E_z != 0)
        {
            trace_target(E, L, T);
        }
        if (L_z < E_z && L_z != 0)
        {
            trace_target(L, E, T);
        }
    }
    if (E_z >= T_z && E_z >= L_z)
    {
        if (T_z < L_z && T_z != 0)
        {
            trace_target(T, L, E);
        }
        if (L_z < T_z && L_z != 0)
        {
            trace_target(L, T, E);
        }
    }
    if (L_z >= E_z && L_z >= T_z)
    {
        if (T_z < E_z && T_z != 0)
        {
            trace_target(T, E, L);
        }
        if (E_z < T_z && E_z != 0)
        {
            trace_target(E, T, L);
        }
    }
    robot_wait();
    msg_init();
}

void first_level::ready_grab_target()
{
    arm_msg.control = 4;
    arm_pub.publish(arm_msg);
    ros::Duration(0.1).sleep();
    arm_msg.control = 1;
    arm_pub.publish(arm_msg);
    ros::Duration(0.1).sleep();
}

void first_level::grab_target()
{
    arm_msg.control = 2;
    arm_pub.publish(arm_msg);
    ros::Duration(0.1).sleep();
    arm_msg.control = 3;
    arm_pub.publish(arm_msg);
    ros::Duration(0.1).sleep();
}

void first_level::heap_target()
{
    arm_msg.control = 5;
    arm_pub.publish(arm_msg);
    ros::Duration(0.1).sleep();
    arm_msg.control = 6;
    arm_pub.publish(arm_msg);
    ros::Duration(0.1).sleep();
}

void first_level::trace_target(uint8_t first, uint8_t second, uint8_t third)
{
    float max_speed = 0.2;
    int temp[3] = {first, second, third};
    float center_z = 250;
    int center_x = 640;
    PID pid_x(0.0005, 0.0001, 0);
    PID pid_z(0.0005, 0.0001, 0);
    ready_grab_target();
    for (int i = 0; i < 3; i++)
    {
        pid_x.init();
        pid_z.init();
        switch (temp[i])
        {
        case T:
            ros::spinOnce();
            if (T_z == 0)
            {
                ROS_INFO("cannot find T");
                break;
            }
            ROS_INFO("trace T");
            while (T_z >= 255 || T_z <= 245)
            {
                if (T_z == 0)
                {
                    wheel_msg.velocity_x = 0;
                    wheel_pub.publish(wheel_msg);
                    break;
                }
                int target = T_z - center_z;
                wheel_msg.velocity_x = pid_z.calculate(target);
                if (wheel_msg.velocity_x >= max_speed)
                {
                    wheel_msg.velocity_x = max_speed;
                }
                if (wheel_msg.velocity_x <= -max_speed)
                {
                    wheel_msg.velocity_x = -max_speed;
                }
                wheel_pub.publish(wheel_msg);
                ros::Duration(0.01).sleep();
                ros::spinOnce();
            }
            pid_z.init();
            msg_init();
            wheel_pub.publish(wheel_msg);
            ros::Duration(1).sleep();
            while (T_x >= 645 || T_x <= 635)
            {
                if (T_x == -1)
                {
                    wheel_msg.velocity_y = 0;
                    wheel_pub.publish(wheel_msg);
                    break;
                }
                int target = T_x - center_x;
                wheel_msg.velocity_y = pid_x.calculate(target);
                if (wheel_msg.velocity_y >= max_speed)
                {
                    wheel_msg.velocity_y = max_speed;
                }
                if (wheel_msg.velocity_y <= -max_speed)
                {
                    wheel_msg.velocity_y = -max_speed;
                }
                wheel_pub.publish(wheel_msg);
                ros::spinOnce();
                ros::Duration(0.01).sleep();
            }
            pid_x.init();
            msg_init();
            wheel_pub.publish(wheel_msg);
            grab_target();
            ros::Duration(10).sleep();
            break;
        case E:
            ros::spinOnce();
            if (E_z == 0)
            {
                ROS_INFO("cannot find E");
                break;
            }
            ROS_INFO("trace E");
            while (E_z >= 255 || E_z <= 245)
            {
                if (E_z == 0)
                {
                    wheel_msg.velocity_x = 0;
                    wheel_pub.publish(wheel_msg);
                    break;
                }
                int target = E_z - center_z;
                wheel_msg.velocity_x = pid_z.calculate(target);
                if (wheel_msg.velocity_x >= max_speed)
                {
                    wheel_msg.velocity_x = max_speed;
                }
                if (wheel_msg.velocity_x <= -max_speed)
                {
                    wheel_msg.velocity_x = -max_speed;
                }
                wheel_pub.publish(wheel_msg);
                ros::Duration(0.01).sleep();
                ros::spinOnce();
            }
            pid_z.init();
            msg_init();
            wheel_pub.publish(wheel_msg);
            ros::Duration(1).sleep();
            while (E_x >= 645 || E_x <= 635)
            {
                if (E_x == -1)
                {
                    wheel_msg.velocity_y = 0;
                    wheel_pub.publish(wheel_msg);
                    break;
                }
                int target = E_x - center_x;
                wheel_msg.velocity_y = pid_x.calculate(target);
                if (wheel_msg.velocity_y >= max_speed)
                {
                    wheel_msg.velocity_y = max_speed;
                }
                if (wheel_msg.velocity_y <= -max_speed)
                {
                    wheel_msg.velocity_y = -max_speed;
                }
                wheel_pub.publish(wheel_msg);
                ros::spinOnce();
                ros::Duration(0.01).sleep();
            }
            pid_x.init();
            msg_init();
            wheel_pub.publish(wheel_msg);
            grab_target();
            ros::Duration(10).sleep();
            break;
        case L:
            ros::spinOnce();
            if (L_z == 0)
            {
                ROS_INFO("cannot find L");
                break;
            }
            ROS_INFO("trace L");
            while (L_z >= 255 || L_z <= 245)
            {
                if (L_z == 0)
                {
                    wheel_msg.velocity_x = 0;
                    wheel_pub.publish(wheel_msg);
                    break;
                }
                int target = L_z - center_z;
                wheel_msg.velocity_x = pid_z.calculate(target);
                if (wheel_msg.velocity_x >= max_speed)
                {
                    wheel_msg.velocity_x = max_speed;
                }
                if (wheel_msg.velocity_x <= -max_speed)
                {
                    wheel_msg.velocity_x = -max_speed;
                }
                wheel_pub.publish(wheel_msg);
                ros::Duration(0.01).sleep();
                ros::spinOnce();
            }
            pid_z.init();
            msg_init();
            wheel_pub.publish(wheel_msg);
            ros::Duration(1).sleep();
            while (L_x >= 645 || L_x <= 635)
            {
                if (L_x == -1)
                {
                    wheel_msg.velocity_y = 0;
                    wheel_pub.publish(wheel_msg);
                    break;
                }
                int target = L_x - center_x;
                wheel_msg.velocity_y = pid_x.calculate(target);
                if (wheel_msg.velocity_y >= max_speed)
                {
                    wheel_msg.velocity_y = max_speed;
                }
                if (wheel_msg.velocity_y <= -max_speed)
                {
                    wheel_msg.velocity_y = -max_speed;
                }
                wheel_pub.publish(wheel_msg);
                ros::spinOnce();
                ros::Duration(0.01).sleep();
            }
            pid_x.init();
            msg_init();
            wheel_pub.publish(wheel_msg);
            grab_target();
            ros::Duration(10).sleep();
            break;
        }
        msg_init();
    }
}

void first_level::robot_move(uint8_t direction, int distance)
{
    switch (direction)
    {
    case left:
        wheel_msg.distance_y = -distance;
        break;

    case right:
        wheel_msg.distance_y = distance;
        break;

    case front:
        wheel_msg.distance_x = distance;
        break;

    case rotate:
        wheel_msg.distance_z = distance;
        break;

    case back:
        wheel_msg.distance_x = -distance;
        break;
    }

    wheel_pub.publish(wheel_msg);
    ros::Duration(0.05).sleep();
    robot_wait();
    msg_init();
}
