package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class MotorControlHelper
{
    LinearOpMode opMode;
    DcMotorEx lf, rf, lb, rb;
    IMU imu;

    FtcDashboard dashboard;
    Telemetry dashboardTelemetry;

    PIDFController lfPIDF, rfPIDF, lbPIDF, rbPIDF;
    PIDFController headingPIDF;

    ElapsedTime timer = new ElapsedTime();

    public static double TICKS_PER_REV = 537.7;
    public static double WHEEL_DIAMETER = 0.1;
    public static double WHEELBASE_LENGTH = 0.295;
    public static double WHEELBASE_WIDTH = 0.38;
    public static double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    public static double r = Math.hypot(WHEELBASE_LENGTH / 2.0, WHEELBASE_WIDTH / 2.0);

    public static double MAX_POWER = 0.3;
    public static double MAX_VELOCITY_TICKS = 2376.63;
    public static double MAX_ACCELERATION_TICKS = 2000;

    public MotorControlHelper(LinearOpMode opMode)
    {
        this.opMode = opMode;

        this.lf = opMode.hardwareMap.get(DcMotorEx.class, "lf");
        this.rf = opMode.hardwareMap.get(DcMotorEx.class, "rf");
        this.lb = opMode.hardwareMap.get(DcMotorEx.class, "lb");
        this.rb = opMode.hardwareMap.get(DcMotorEx.class, "rb");

        this.imu = opMode.hardwareMap.get(IMU.class, "imu");

        double motorKp = 1.4;
        double motorKi = 0.0;
        double motorKd = 0.0;
        double motorKf = 0.0;
        lfPIDF = new PIDFController(motorKp, motorKi, motorKd, motorKf);
        rfPIDF = new PIDFController(motorKp, motorKi, motorKd, motorKf);
        lbPIDF = new PIDFController(motorKp, motorKi, motorKd, motorKf);
        rbPIDF = new PIDFController(motorKp, motorKi, motorKd, motorKf);

        double headingKp = 0.5;
        double headingKi = 0.0;
        double headingKd = 0.0;
        double headingKf = 0.0;
        headingPIDF = new PIDFController(headingKp, headingKi, headingKd, headingKf);

        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
        dashboardTelemetry.addData("FL Power", 0);
        dashboardTelemetry.addData("FR Power", 0);
        dashboardTelemetry.addData("BL Power", 0);
        dashboardTelemetry.addData("BR Power", 0);
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

    public void translate(double heading, double distance, double temp)
    {
        move(heading, distance, 0.0);
    }

    public void rotate(double angle, double temp)
    {
        move(0.0, 0.0, angle);
    }

    private void move(double heading, double distance, double angle)
    {
        int lfTarget, rfTarget, lbTarget, rbTarget;
        double headingRad = Math.toRadians(heading);

        double vy = Math.cos(headingRad) * distance;
        double vx = Math.sin(headingRad) * distance;
        double omega = Math.toRadians(angle);

        double lfDistance = vy + vx + omega * r;
        double rfDistance = vy - vx - omega * r;
        double lbDistance = vy - vx + omega * r;
        double rbDistance = vy + vx - omega * r;

        lfTarget = lf.getCurrentPosition() + (int) (lfDistance / WHEEL_CIRCUMFERENCE * TICKS_PER_REV);
        rfTarget = rf.getCurrentPosition() + (int) (rfDistance / WHEEL_CIRCUMFERENCE * TICKS_PER_REV);
        lbTarget = lb.getCurrentPosition() + (int) (lbDistance / WHEEL_CIRCUMFERENCE * TICKS_PER_REV);
        rbTarget = rb.getCurrentPosition() + (int) (rbDistance / WHEEL_CIRCUMFERENCE * TICKS_PER_REV);

        lf.setTargetPosition(lfTarget);
        rf.setTargetPosition(rfTarget);
        lb.setTargetPosition(lbTarget);
        rb.setTargetPosition(rbTarget);

        setRunToPositionMode();

        setMotorPower(1.0, 1.0, 1.0, 1.0);

        timer.reset();

        while (opMode.opModeIsActive() && motorsAreBusy())
        {
            double dt = timer.seconds();
            if (dt == 0)
            {
                dt = 0.01;
            }
            timer.reset();

            if (distance > 0.0)
            {

            }
        }
    }
}
