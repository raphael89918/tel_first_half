#ifndef __PID_HPP__
#define __PID_HPP__

class PID
{
public:
    PID();
    PID(double kp, double ki, double kd);
    ~PID();

    double get_kp();
    double get_ki();
    double get_kd();

    void set_kp(double kp);
    void set_ki(double ki);
    void set_kd(double kd);

    void set(double kp, double ki, double kd);

    double calculate(double error, double dt);
    double imcremental_calculate(double error, double dt);

private:
    double m_kp;
    double m_ki;
    double m_kd;

    double m_error;
    double m_integral;
    double m_derivative;

    double m_output;
};

#endif // __PID_HPP__