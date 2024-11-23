package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Autonomous.MotorControlHelper;

@Config
@TeleOp(name = "EnhancedTeleOp", group = "Enhanced")
public class EnhancedTeleOp extends LinearOpMode
{
    DcMotorEx lf;
    DcMotorEx rf;
    DcMotorEx lb;
    DcMotorEx rb;

    IMU imu;

    boolean INPUT_DEADZONE_ON = false;
    boolean INPUT_SCALING_ON = false;
    boolean INPUT_SMOOTHING_ON = false;
    boolean FIELD_CENTRIC_ON = false;
    boolean ACCEL_LIMITING_ON = false;
    boolean PIDF_ON = false;


    final double DEADZONE = 0.02;
    final double RAMP_RATE = 1.0;
    final double MAX_ACCEL = 1.0;

    double prevY = 0;
    double prevX = 0;
    double prevR = 0;

    ElapsedTime timer = new ElapsedTime();

    double maxTicksPerSecond;

    MotorControlHelper helper;
    public static double HEADING = 0.0;
    public static double DISTANCE = 1.5;

    @Override
    public void runOpMode()
    {
        lf = hardwareMap.get(DcMotorEx.class, "lf");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        lb = hardwareMap.get(DcMotorEx.class, "lb");
        rb = hardwareMap.get(DcMotorEx.class, "rb");

        //GoBilda 5203-2402-0019
        lf.getMotorType().setMaxRPM(312);
        lf.getMotorType().setTicksPerRev(537.7);
        lf.getMotorType().setGearing(19.2);
        rf.getMotorType().setMaxRPM(312);
        rf.getMotorType().setTicksPerRev(537.7);
        rf.getMotorType().setGearing(19.2);
        lb.getMotorType().setMaxRPM(312);
        lb.getMotorType().setTicksPerRev(537.7);
        lf.getMotorType().setGearing(19.2);
        rb.getMotorType().setMaxRPM(312);
        rb.getMotorType().setTicksPerRev(537.7);
        rb.getMotorType().setGearing(19.2);

        maxTicksPerSecond = lf.getMotorType().getAchieveableMaxTicksPerSecond();

        lf.setDirection(DcMotorEx.Direction.REVERSE);
        lb.setDirection(DcMotorEx.Direction.REVERSE);

        lf.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        lf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        IMU.Parameters imuParams = new IMU.Parameters(orientationOnRobot);
        imu.initialize(imuParams);
        imu.resetYaw(); //careful here, might wanna store inital heading from autoOp first

        //helper = new MotorControlHelper(this);
        waitForStart();

        //helper.translate(HEADING, DISTANCE);

        timer.reset();

        while (opModeIsActive())
        {
            double loopTime = timer.seconds();
            timer.reset();

            if (gamepad1.a)
            {
                INPUT_DEADZONE_ON = !INPUT_DEADZONE_ON;
                while (gamepad1.a){}
            }
            if (gamepad1.b)
            {
                INPUT_SCALING_ON = !INPUT_SCALING_ON;
                while (gamepad1.b){}
            }
            if (gamepad1.x)
            {
                INPUT_SMOOTHING_ON = !INPUT_SMOOTHING_ON;
                while (gamepad1.x){}
            }
            if (gamepad1.y)
            {
                FIELD_CENTRIC_ON = !FIELD_CENTRIC_ON;
                while (gamepad1.y){}
            }
            if (gamepad1.left_bumper)
            {
                ACCEL_LIMITING_ON = !ACCEL_LIMITING_ON;
                while (gamepad1.left_bumper){}
            }
            if (gamepad1.right_bumper)
            {
                PIDF_ON = !PIDF_ON;
                while (gamepad1.right_bumper){}
            }

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double r = gamepad1.right_stick_x;

            if (INPUT_DEADZONE_ON)
            {
                y = applyDeadzone(y);
                x = applyDeadzone(x);
                r = applyDeadzone(r);
            }

            if (INPUT_SCALING_ON)
            {
                y = scaleInput(y);
                x = scaleInput(x);
                r = scaleInput(r);
            }

            if (INPUT_SMOOTHING_ON)
            {
                y = rampInput(y, prevY, loopTime);
                x = rampInput(x, prevX, loopTime);
                r = rampInput(r, prevR, loopTime);
            }
            prevY = y;
            prevX = x;
            prevR = r;

            if (FIELD_CENTRIC_ON)
            {
                double headingRadian = getHeadingRadian();
                double cosA = Math.cos(-headingRadian);
                double sinA = Math.sin(-headingRadian);
                double oldx = x;
                x = x * cosA - y * sinA;
                y = oldx * sinA + y * cosA;
            }

            double lfOutput = y + x + r;
            double rfOutput = y - x - r;
            double lbOutput = y - x + r;
            double rbOutput = y + x - r;

            double maxOutput = Math.max(Math.abs(lfOutput), Math.abs(rfOutput));
            maxOutput = Math.max(maxOutput, Math.abs(lbOutput));
            maxOutput = Math.max(maxOutput, Math.abs(rbOutput));
            if (maxOutput > 1.0)
            {
                lfOutput /= maxOutput;
                rfOutput /= maxOutput;
                lbOutput /= maxOutput;
                rbOutput /= maxOutput;
            }

            if (PIDF_ON)
            {
                lfOutput *= maxTicksPerSecond;
                rfOutput *= maxTicksPerSecond;
                lbOutput *= maxTicksPerSecond;
                rbOutput *= maxTicksPerSecond;

                lf.setVelocity(lfOutput);
                rf.setVelocity(rfOutput);
                lb.setVelocity(lbOutput);
                rb.setVelocity(rbOutput);
            }
            else
            {
                if (ACCEL_LIMITING_ON)
                {
                    lfOutput = limitAcceleration(lfOutput, lf.getPower(), loopTime);
                    rfOutput = limitAcceleration(rfOutput, rf.getPower(), loopTime);
                    lbOutput = limitAcceleration(lbOutput, lb.getPower(), loopTime);
                    rbOutput = limitAcceleration(rbOutput, rb.getPower(), loopTime);
                }
                lf.setPower(lfOutput);
                rf.setPower(rfOutput);
                lb.setPower(lbOutput);
                rb.setPower(rbOutput);
            }


            telemetry.addData("Dead Zone", INPUT_DEADZONE_ON);
            telemetry.addData("Input Scaling", INPUT_SCALING_ON);
            telemetry.addData("Input Smoothing", INPUT_SMOOTHING_ON);
            telemetry.addData("Field Centric", FIELD_CENTRIC_ON);
            telemetry.addData("PIDF", PIDF_ON);
            telemetry.addData("Accel Limiting", ACCEL_LIMITING_ON);
            telemetry.update();

        }

    }

    private double applyDeadzone(double input)
    {
        return Math.abs(input) > DEADZONE ? input : 0.0;
    }

    private double scaleInput(double input)
    {
        return Math.pow(input, 3);
    }

    private double rampInput(double target, double prev, double loopTime)
    {
        if (Math.abs(target) < DEADZONE)
        {
            return 0.0;
        }
        double maxDelta = RAMP_RATE * loopTime;
        double delta = target-prev;
        if (delta > maxDelta)
        {
            delta = maxDelta;
        }
        else if (delta < -maxDelta)
        {
            delta = -maxDelta;
        }
        return prev + delta;
    }

    private double limitAcceleration(double target, double current, double loopTime)
    {
        if (Math.abs(target) < DEADZONE)
        {
            return 0.0;
        }
        double maxDelta = MAX_ACCEL * loopTime;
        double delta = target - current;
        if (delta > maxDelta)
        {
            delta = maxDelta;
        }
        else if (delta < -maxDelta)
        {
            delta = -maxDelta;
        }
        return current + delta;
    }

    private double getHeadingRadian()
    {
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        return angles.getYaw(AngleUnit.RADIANS);
    }
}
