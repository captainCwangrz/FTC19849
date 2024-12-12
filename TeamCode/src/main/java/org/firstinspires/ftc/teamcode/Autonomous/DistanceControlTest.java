package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "DistanceControlTest", group = "Testing")
public class DistanceControlTest extends LinearOpMode
{
    DcMotorEx lf;
    DcMotorEx rf;
    DcMotorEx lb;
    DcMotorEx rb;

    IMU imu;
    RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    double maxRPM = 312;
    double gearing = 19.2;
    double TICKS_PER_REV = 537.7;
    double WHEEL_DIAMETER = 0.1;
    double WHEELBASE_LENGTH = 0.295;
    double WHEELBASE_WIDTH = 0.38;
    double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    double r = Math.hypot(WHEELBASE_LENGTH / 2.0, WHEELBASE_WIDTH / 2.0);

    @Override
    public void runOpMode()
    {
        initIMU();
        initMotors();

        waitForStart();

        while (opModeIsActive())
        {
            translate(0, 1, 0.4);
            sleep(3000);
            translate(180, 1, 0.4);
            sleep(3000);
            translate(-45, 0.5, 0.4);
            sleep(3000);
            translate(135, 0.5, 0.4);
            break;
        }
    }

    public void translate(double directionDegrees, double distanceMeters, double maxPower)
    {
        double radians = Math.toRadians(directionDegrees);
        double vx = distanceMeters * Math.sin(radians);
        double vy = distanceMeters * Math.cos(radians);

        double lfDistance = vy + vx;
        double rfDistance = vy - vx;
        double lbDistance = vy - vx;
        double rbDistance = vy + vx;

        int lfTargetTicks = metersToTicks(lfDistance) + lf.getCurrentPosition();
        int rfTargetTicks = metersToTicks(rfDistance) + rf.getCurrentPosition();
        int lbTargetTicks = metersToTicks(lbDistance) + lb.getCurrentPosition();
        int rbTargetTicks = metersToTicks(rbDistance) + rb.getCurrentPosition();

        lf.setTargetPosition(lfTargetTicks);
        rf.setTargetPosition(rfTargetTicks);
        lb.setTargetPosition(lbTargetTicks);
        rb.setTargetPosition(rbTargetTicks);

        lf.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rf.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        lb.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rb.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        lf.setPower(maxPower);
        rf.setPower(maxPower);
        lb.setPower(maxPower);
        rb.setPower(maxPower);

        while ((lf.isBusy() || rf.isBusy() || lb.isBusy() || rb.isBusy()))
        {

        }
        stopAllMotors();
        resetEncoders();
    }

    private void initMotors()
    {
        lf = hardwareMap.get(DcMotorEx.class, "lf");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        lb = hardwareMap.get(DcMotorEx.class, "lb");
        rb = hardwareMap.get(DcMotorEx.class, "rb");

        //GoBilda 5203-2402-0019
        lf.getMotorType().setMaxRPM(maxRPM);
        lf.getMotorType().setTicksPerRev(TICKS_PER_REV);
        lf.getMotorType().setGearing(gearing);
        rf.getMotorType().setMaxRPM(maxRPM);
        rf.getMotorType().setTicksPerRev(TICKS_PER_REV);
        rf.getMotorType().setGearing(gearing);
        lb.getMotorType().setMaxRPM(maxRPM);
        lb.getMotorType().setTicksPerRev(TICKS_PER_REV);
        lb.getMotorType().setGearing(gearing);
        rb.getMotorType().setMaxRPM(maxRPM);
        rb.getMotorType().setTicksPerRev(TICKS_PER_REV);
        rb.getMotorType().setGearing(gearing);

        lf.setDirection(DcMotorEx.Direction.REVERSE);
        lb.setDirection(DcMotorEx.Direction.REVERSE);

        lf.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        resetEncoders();
        setPIDFCoefficients();
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

    private void setPIDFCoefficients()
    {
        lf.setVelocityPIDFCoefficients(1.2, 0.2, 0, 11);
        lf.setPositionPIDFCoefficients(5);
        rf.setVelocityPIDFCoefficients(1.2, 0.2, 0, 11);
        rf.setPositionPIDFCoefficients(5);
        lb.setVelocityPIDFCoefficients(1.2, 0.2, 0, 11);
        lb.setPositionPIDFCoefficients(5);
        rb.setVelocityPIDFCoefficients(1.2, 0.2, 0, 11);
        rb.setPositionPIDFCoefficients(5);
    }

    private void initIMU()
    {
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        IMU.Parameters imuParams = new IMU.Parameters(orientationOnRobot);
        imu.initialize(imuParams);
        imu.resetYaw(); //careful here, might wanna store inital heading from autoOp first
    }

    private double ticksToMeters(int ticks)
    {
        return (ticks / TICKS_PER_REV) * WHEEL_CIRCUMFERENCE;
    }

    private int metersToTicks(double meters)
    {
        return (int) ((meters / WHEEL_CIRCUMFERENCE) * TICKS_PER_REV);
    }

    private void stopAllMotors()
    {
        lf.setPower(0);
        rf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);
    }
}
