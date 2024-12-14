package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
@TeleOp(name = "DistanceControlTest", group = "Testing")
public class DistanceControlTest extends LinearOpMode
{
    DcMotorEx lf;
    DcMotorEx rf;
    DcMotorEx lb;
    DcMotorEx rb;
    DcMotorEx lLift;

    Servo lClaw;
    Servo rClaw;
    Servo elbow;
    double lClawPos = 0.5;
    double rClawPos = 0.5;
    double elbowPos = 0.5;

    IMU imu;
    RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
    double filteredYaw = 0.0;
    double alpha = 0.2;

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
        initMotors();
        initServos();
        initIMU();

        waitForStart();

        while (opModeIsActive())
        {
            if (gamepad1.y)
            {
                translate(40,0.65,0.6);
                translate(90,0.565,0.6);
            }
            if (gamepad1.a)
            {
                translate(180,1,0.6);
            }
            telemetry.addData("Heading:",getHeadingDegrees());
            telemetry.update();
            if (gamepad1.x)
            {
                rotate(90, 0.2);
            }
            idle();
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

        while ((lf.isBusy() || rf.isBusy() || lb.isBusy() || rb.isBusy()) && opModeIsActive())
        {
            int lfRemain = Math.abs(lfTargetTicks - lf.getCurrentPosition());
            int rfRemain = Math.abs(rfTargetTicks - rf.getCurrentPosition());
            int lbRemain = Math.abs(lbTargetTicks - lb.getCurrentPosition());
            int rbRemain = Math.abs(rbTargetTicks - rb.getCurrentPosition());
            double avgRemain = ticksToMeters((lfRemain + rfRemain + lbRemain + rbRemain) / 4.0);
            telemetry.addData("avg remain (m)", avgRemain);
            telemetry.update();
            if (avgRemain <= 0.03)
            {
                break;
            }
            idle();
        }
        stopAllMotors();
        resetEncoders();
    }

    public void rotate(double degrees, double maxPower)
    {
        double initialHeading = getHeadingDegrees();
        double targetHeading = initialHeading - degrees;
        targetHeading = normalizeAngle(targetHeading);

        while (opModeIsActive())
        {
            double currentHeading = getHeadingDegrees();
            double error = getAngleDifference(targetHeading, currentHeading);
            double power = error * 0.03;
            power = Math.max(Math.min(power, maxPower), -maxPower);
            lf.setPower(-power);
            rf.setPower(power);
            lb.setPower(-power);
            rb.setPower(power);
            telemetry.addData("Current Heading", currentHeading);
            telemetry.addData("Target Heading", targetHeading);
            telemetry.addData("Error", error);
            telemetry.addData("Power", power);
            telemetry.update();
            if (Math.abs(error) <= 5.0)
            {
                break;
            }
            idle();
        }
        stopAllMotors();
    }

    public void clawClamp()
    {
        rClawPos = 0.83;
        lClawPos = 0.17;
        lClaw.setPosition(lClawPos);
        rClaw.setPosition(rClawPos);
    }

    public void clawOpen()
    {
        rClawPos = 0.4;
        lClawPos = 0.6;
        lClaw.setPosition(lClawPos);
        rClaw.setPosition(rClawPos);
    }

    public void elbowGrab()
    {
        elbow.setPosition(elbowPos);
    }

    public void elbowPick()
    {
        elbow.setPosition(elbowPos);
    }

    public void elbowHang()
    {
        elbow.setPosition(elbowPos);
    }

    public void liftPrep()
    {

    }

    public void liftHang()
    {

    }

    public void armRest()
    {

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
        lLift.getMotorType().setMaxRPM(maxRPM);
        lLift.getMotorType().setTicksPerRev(TICKS_PER_REV);
        lLift.getMotorType().setGearing(gearing);

        lf.setDirection(DcMotorEx.Direction.REVERSE);
        lb.setDirection(DcMotorEx.Direction.REVERSE);

        lf.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lLift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        resetEncoders();
        setPIDFCoefficients(3,2,0,9,1.5);
    }

    private void resetEncoders()
    {
        lf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lLift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        lf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        lLift.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    private void setPIDFCoefficients(double encoderP, double encoderI, double encoderD, double encoderF, double positionP)
    {
        lf.setVelocityPIDFCoefficients(encoderP, encoderI, encoderD, encoderF);
        lf.setPositionPIDFCoefficients(positionP);
        rf.setVelocityPIDFCoefficients(encoderP, encoderI, encoderD, encoderF);
        rf.setPositionPIDFCoefficients(positionP);
        lb.setVelocityPIDFCoefficients(encoderP, encoderI, encoderD, encoderF);
        lb.setPositionPIDFCoefficients(positionP);
        rb.setVelocityPIDFCoefficients(encoderP, encoderI, encoderD, encoderF);
        rb.setPositionPIDFCoefficients(positionP);
    }

    private void initServos()
    {
        lClaw = hardwareMap.get(Servo.class, "lclaw");
        rClaw = hardwareMap.get(Servo.class, "rclaw");
        elbow = hardwareMap.get(Servo.class, "elbow");

        lClaw.setPosition(lClawPos);
        rClaw.setPosition(rClawPos);
        elbow.setPosition(elbowPos);
    }

    private void initIMU()
    {
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        IMU.Parameters imuParams = new IMU.Parameters(orientationOnRobot);
        imu.initialize(imuParams);
        imu.resetYaw(); //careful here, might wanna store inital heading from autoOp first
    }

    private double ticksToMeters(double ticks)
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

    //Returns current heading [-180,180] where negative degrees is clockwise and vice versa
    private double getHeadingDegrees()
    {
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        double newYaw = angles.getYaw(AngleUnit.DEGREES);
        filteredYaw = alpha * newYaw + (1 - alpha) * filteredYaw;
        return normalizeAngle(filteredYaw);
    }

    private double normalizeAngle(double angle)
    {
        while (angle > 180.0) angle -= 360.0;
        while (angle <= -180.0) angle += 360.0;
        return angle;
    }

    private double getAngleDifference(double target, double current)
    {
        double difference = target - current;
        difference = normalizeAngle(difference);
        return difference;
    }
}