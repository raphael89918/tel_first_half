#include "pid/pid.hpp"

PID::PID()
    : m_kp(0.0), m_ki(0.0), m_kd(0.0), m_error(0.0), m_integral(0.0), m_derivative(0.0), m_output(0.0) {}

PID::PID(double kp, double ki, double kd)
    : m_kp(kp), m_ki(ki), m_kd(kd), m_error(0.0), m_integral(0.0), m_derivative(0.0), m_output(0.0) {}

PID::~PID() {}

double PID::get_kp() { return m_kp; }

double PID::get_ki() { return m_ki; }

double PID::get_kd() { return m_kd; }

void PID::set_kp(double kp) { m_kp = kp; }

void PID::set_ki(double ki) { m_ki = ki; }

void PID::set_kd(double kd) { m_kd = kd; }

void PID::set(double kp, double ki, double kd)
{
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;
}

double PID::calculate(double error, double dt)
{
    m_error = error;
    m_integral += m_error * dt;
    m_derivative = (m_error - m_derivative) / dt;

    m_output = m_kp * m_error + m_ki * m_integral + m_kd * m_derivative;

    return m_output;
}