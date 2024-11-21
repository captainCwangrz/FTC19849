package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class MotorControlHelper
{
    DcMotorEx lf, rf, lb, rb;

    public static double TICKS_PER_REV = 537.7;
    public static double WHEEL_DIAMETER = 0.1;
    public static double WHEELBASE_LENGTH = 0.295;
    public static double WHEELBASE_WIDTH = 0.38;

    public static double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    public static double r = Math.hypot(WHEELBASE_LENGTH / 2.0, WHEELBASE_WIDTH / 2.0);

    public static double MAX_POWER = 0.3;

    public static int P = 3;
    public static int I = 0;
    public static int D = 0;
    public static int F = 0;

    FtcDashboard dashboard;
    Telemetry dashboardTelemetry;

    public MotorControlHelper(DcMotorEx lf, DcMotorEx rf, DcMotorEx lb, DcMotorEx rb)
    {
        this.lf = lf;
        this.rf = rf;
        this.lb = lb;
        this.rb = rb;

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, I, D, F);
        lf.setPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION, pidfCoefficients);
        rf.setPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION, pidfCoefficients);
        lb.setPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION, pidfCoefficients);
        rb.setPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION, pidfCoefficients);

        PIDFCoefficients activeCoefficients = lf.getPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION);
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
        dashboardTelemetry.addData("FL Power", 0);
        dashboardTelemetry.addData("FR Power", 0);
        dashboardTelemetry.addData("BL Power", 0);
        dashboardTelemetry.addData("BR Power", 0);
        dashboardTelemetry.addData("P", activeCoefficients.p);
        dashboardTelemetry.addData("I", activeCoefficients.i);
        dashboardTelemetry.addData("D", activeCoefficients.d);
        dashboardTelemetry.addData("F", activeCoefficients.f);
        dashboardTelemetry.update();

        resetEncoders();
    }

    private void resetEncoders()
    {
        lf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        lf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    private void setRunToPositionMode()
    {
        lf.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rf.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        lb.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rb.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    private void setRunUsingEncoderMode()
    {
        lf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    private void setMotorPower(double lfPower, double rfPower, double lbPower, double rbPower)
    {
        lf.setPower(lfPower);
        rf.setPower(rfPower);
        lb.setPower(lbPower);
        rb.setPower(rbPower);
    }

    private boolean motorsAreBusy()
    {
        return lf.isBusy() || rf.isBusy() || lb.isBusy() || rb.isBusy();
    }

    private void stopMotors()
    {
        setMotorPower(0, 0, 0, 0);
    }

    public void translate(double headingDegrees, double distanceMeters)
    {
        double headingRadians = Math.toRadians(headingDegrees);

        double Vx = distanceMeters * Math.sin(headingRadians);
        double Vy = distanceMeters * Math.cos(headingRadians);
        double omega = 0; //ro rotation

        double lfDistance = Vy + Vx + omega * r;
        double rfDistance = Vy - Vx - omega * r;
        double lbDistance = Vy - Vx + omega * r;
        double rbDistance = Vy + Vx - omega * r;

        double lfRotations = lfDistance / WHEEL_CIRCUMFERENCE;
        double rfRotations = rfDistance / WHEEL_CIRCUMFERENCE;
        double lbRotations = lbDistance / WHEEL_CIRCUMFERENCE;
        double rbRotations = rbDistance / WHEEL_CIRCUMFERENCE;

        int lfTicks = (int) Math.round(lfRotations * TICKS_PER_REV);
        int lbTicks = (int) Math.round(lbRotations * TICKS_PER_REV);
        int rfTicks = (int) Math.round(rfRotations * TICKS_PER_REV);
        int rbTicks = (int) Math.round(rbRotations * TICKS_PER_REV);

        lf.setTargetPosition(lf.getCurrentPosition() + lfTicks);
        rf.setTargetPosition(rf.getCurrentPosition() + rfTicks);
        lb.setTargetPosition(lb.getCurrentPosition() + lbTicks);
        rb.setTargetPosition(rb.getCurrentPosition() + rbTicks);

        setRunToPositionMode();

        double maxTicks = Math.max(Math.max(Math.abs(lfTicks), Math.abs(rfTicks)), Math.max(Math.abs(lbTicks), Math.abs(rbTicks)));
        if (maxTicks > 0 )
        {
            double lfPower = MAX_POWER * lfTicks / maxTicks;
            double rfPower = MAX_POWER * rfTicks / maxTicks;
            double lbPower = MAX_POWER * lbTicks / maxTicks;
            double rbPower = MAX_POWER * rbTicks / maxTicks;

            setMotorPower(lfPower, rfPower, lbPower, rbPower);
        }

        while (motorsAreBusy()) { }

        stopMotors();
        setRunUsingEncoderMode();
    }
}
