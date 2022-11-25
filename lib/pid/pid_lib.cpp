#include "pid/pid.hpp"

PID::PID()
    : m_kp(0.0), m_ki(0.0), m_kd(0.0),
      m_ka(0.0), m_kb(0.0), m_kc(0.0),
      m_error(0.0), m_error_1(0.0), m_error_2(0.0),
      m_integral(0.0), m_derivative(0.0), m_output(0.0) {}

PID::PID(double kp, double ki, double kd)
    : m_kp(kp), m_ki(ki), m_kd(kd),
      m_ka(kp + ki + kd), m_kb(-2 * kd - kp), m_kc(kd),
      m_error(0.0), m_error_1(0.0), m_error_2(0.0),
      m_integral(0.0), m_derivative(0.0), m_output(0.0) {}

PID::~PID() {}

double PID::get_kp() { return m_kp; }

double PID::get_ki() { return m_ki; }

double PID::get_kd() { return m_kd; }

double PID::get_ka() { return m_kp + m_ki + m_kd; }

double PID::get_kb() { return -2 * m_kd - m_kp; }

double PID::get_kc() { return m_kd; }

void PID::init()
{
    m_integral = 0;
    m_derivative = 0;
    m_output = 0;
    m_error = 0;
    m_error_1 = 0;
    m_error_2 = 0;
}

void PID::set_kp(double kp) { m_kp = kp; }

void PID::set_ki(double ki) { m_ki = ki; }

void PID::set_kd(double kd) { m_kd = kd; }

void PID::set(double kp, double ki, double kd)
{
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;
}

double PID::calculate(double error)
{
    m_error = error;
    m_integral += m_error;
    m_derivative = (m_error - m_derivative);

    m_output = m_kp * m_error + m_ki * m_integral + m_kd * m_derivative;

    return m_output;
}

double PID::calculate_incremental(double error)
{
    m_error = error;

    m_output = get_ka() * error + get_kb() * m_error_1 + get_kc() * m_error_2;
    m_error_2 = m_error_1;
    m_error_1 = error;

    return m_output;
}