package org.firstinspires.ftc.teamcode.Autonomous;


public class PIDFController
{
    double kP, kI, kD, kF;
    double integral;
    double previousError;
    double setpoint;

    public PIDFController(double kP, double kI, double kD, double kF)
    {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.integral = 0.0;
        this.previousError = 0.0;
        this.setpoint = 0.0;
    }

    public void setSetpoint(double setpoint)
    {
        this.setpoint = setpoint;
    }

    public double calculate( double measured, double dt)
    {
        double error = setpoint - measured;
        integral += error * dt;
        double derivative = (error - previousError) / dt;
        previousError = error;
        return (kP * error) + (kI * integral) + (kD * derivative) + (kF * setpoint);
    }

    public void reset()
    {
        integral = 0.0;
        previousError = 0.0;
    }
}
