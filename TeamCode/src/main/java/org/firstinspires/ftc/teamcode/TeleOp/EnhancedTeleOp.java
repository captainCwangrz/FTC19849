//20241208
package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "EnhancedTeleOp", group = "Enhanced")
public class EnhancedTeleOp extends LinearOpMode
{
    DcMotorEx lf;
    DcMotorEx rf;
    DcMotorEx lb;
    DcMotorEx rb;

    DcMotorEx lLift;
    DcMotorEx rLift;

    Servo lClaw;
    Servo rClaw;
    Servo elbow;

    IMU imu;
    final RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    final RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    double DEADZONE = 0.02;

    boolean INPUT_SCALING_ON = true;

    boolean INPUT_SMOOTHING_ON = true;
    double dynamicEmaAlpha;
    double prevMagnitude = 0.0;

    boolean FIELD_CENTRIC_ON = false;

    boolean ACCEL_LIMITING_ON = false;
    double dynamicMaxAccel;

    boolean REVERSE_ELBOW = false;
    boolean PAD2_ROTATE = false;

    final double maxRPM = 312;
    final double TPR = 537.7;
    final double gearing = 19.2;
    double maxTicksLimiter = 0.6;
    double maxTicksPerSecond;

    double filteredYaw = 0.0;
    double alpha = 0.2;

    double lClawPos = 0.5;
    double rClawPos = 0.5;
    double elbowPos = 0.5;
    double clawIncrement = 0.003;
    double maxElbowIncrement = 0.006;

    ElapsedTime timer = new ElapsedTime();

    private void initMotors()
    {
        lf = hardwareMap.get(DcMotorEx.class, "lf");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        lb = hardwareMap.get(DcMotorEx.class, "lb");
        rb = hardwareMap.get(DcMotorEx.class, "rb");
        lLift = hardwareMap.get(DcMotorEx.class, "llift");
        rLift = hardwareMap.get(DcMotorEx.class, "rlift");

        //GoBilda 5203-2402-0019
        lf.getMotorType().setMaxRPM(maxRPM);
        lf.getMotorType().setTicksPerRev(TPR);
        lf.getMotorType().setGearing(gearing);
        rf.getMotorType().setMaxRPM(maxRPM);
        rf.getMotorType().setTicksPerRev(TPR);
        rf.getMotorType().setGearing(gearing);
        lb.getMotorType().setMaxRPM(maxRPM);
        lb.getMotorType().setTicksPerRev(TPR);
        lb.getMotorType().setGearing(gearing);
        rb.getMotorType().setMaxRPM(maxRPM);
        rb.getMotorType().setTicksPerRev(TPR);
        rb.getMotorType().setGearing(gearing);
        lLift.getMotorType().setMaxRPM(maxRPM);
        lLift.getMotorType().setTicksPerRev(TPR);
        lLift.getMotorType().setGearing(gearing);
        rLift.getMotorType().setMaxRPM(maxRPM);
        rLift.getMotorType().setTicksPerRev(TPR);
        rLift.getMotorType().setGearing(gearing);

        lf.setDirection(DcMotorEx.Direction.REVERSE);
        lb.setDirection(DcMotorEx.Direction.REVERSE);

        lf.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lLift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rLift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        resetEncoders();
    }

    private void resetEncoders()
    {
        lf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lLift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rLift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        lf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        lLift.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rLift.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
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

    @Override
    public void runOpMode()
    {
        initMotors();
        maxTicksPerSecond = lf.getMotorType().getAchieveableMaxTicksPerSecond();

        initServos();

        initIMU();

        waitForStart();
        timer.reset();

        while (opModeIsActive())
        {
            double loopTime = timer.seconds();
            timer.reset();

            if (gamepad1.y)
            {
                FIELD_CENTRIC_ON = !FIELD_CENTRIC_ON;
                while (gamepad1.y){}
            }
            if (gamepad2.a)
            {
                REVERSE_ELBOW = !REVERSE_ELBOW;
                while (gamepad2.a){}
            }
            if (gamepad2.y)
            {
                PAD2_ROTATE = !PAD2_ROTATE;
                while (gamepad2.y){}
            }

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double r = gamepad1.right_stick_x;
            if (PAD2_ROTATE)
            {
                r = gamepad2.right_stick_x;
            }

            double lift = -gamepad2.left_stick_y;
            double arm = -gamepad2.right_stick_y;
            boolean openClaw = gamepad2.x;
            boolean closeClaw = gamepad2.b;

            y = applyDeadzone(y);
            x = applyDeadzone(x);
            r = applyDeadzone(r);
            lift = applyDeadzone(lift);
            arm = applyDeadzone(arm);

            if (INPUT_SCALING_ON)
            {
                y = scaleInput(y);
                x = scaleInput(x);
                r = scaleInput(r);
                lift = scaleInput(lift);
                arm = scaleInput(arm);
            }

            double magnitude = Math.sqrt(y * y + x * x + r * r);
            double directionX = (magnitude != 0) ? x / magnitude : 0;
            double directionY = (magnitude != 0) ? y / magnitude : 0;
            double directionR = (magnitude != 0) ? r / magnitude : 0;
            boolean suddenDirectionChange = Math.abs(magnitude - prevMagnitude) > 0.5;
            if (suddenDirectionChange)
            {
                dynamicMaxAccel = 3.0;
                dynamicEmaAlpha = 0.9;
            }
            else
            {
                if (magnitude < 0.3)
                {
                    dynamicMaxAccel = 0.5;
                    dynamicEmaAlpha = 0.3;
                }
                else
                {
                    dynamicMaxAccel = 1.5;
                    dynamicEmaAlpha = 0.5;
                }
            }

            if (INPUT_SMOOTHING_ON)
            {
                magnitude = dynamicEmaAlpha * magnitude + (1 - dynamicEmaAlpha) * prevMagnitude;
            }
            y = magnitude * directionY;
            x = magnitude * directionX;
            r = magnitude * directionR;
            prevMagnitude = magnitude;


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


            lfOutput = lfOutput * maxTicksPerSecond * maxTicksLimiter;
            rfOutput = rfOutput * maxTicksPerSecond * maxTicksLimiter;
            lbOutput = lbOutput * maxTicksPerSecond * maxTicksLimiter;
            rbOutput = rbOutput * maxTicksPerSecond * maxTicksLimiter;
            lift = lift * maxTicksPerSecond;
            if (ACCEL_LIMITING_ON)
            {
                lfOutput = limitAcceleration(lfOutput, lf.getVelocity(), loopTime, dynamicMaxAccel);
                rfOutput = limitAcceleration(rfOutput, rf.getVelocity(), loopTime, dynamicMaxAccel);
                lbOutput = limitAcceleration(lbOutput, lb.getVelocity(), loopTime, dynamicMaxAccel);
                rbOutput = limitAcceleration(rbOutput, rb.getVelocity(), loopTime, dynamicMaxAccel);
            }

            if (openClaw)
            {
                lClawPos += clawIncrement;
                rClawPos -= clawIncrement;
            }
            if (closeClaw)
            {
                rClawPos += clawIncrement;
                lClawPos -= clawIncrement;
            }
            if (gamepad2.left_bumper)
            {
                rClawPos = 0.6;
                lClawPos = 0.4;
            }
            if (gamepad2.right_bumper)
            {
                rClawPos = 0.83;
                lClawPos = 0.17;
            }

            if (REVERSE_ELBOW)
            {
                elbowPos -= arm * maxElbowIncrement;
            }
            else
            {
                elbowPos += arm * maxElbowIncrement;
            }

            lClaw.setPosition(lClawPos);
            rClaw.setPosition(rClawPos);
            elbow.setPosition(elbowPos);

            lf.setVelocity(lfOutput);
            rf.setVelocity(rfOutput);
            lb.setVelocity(lbOutput);
            rb.setVelocity(rbOutput);
            lLift.setVelocity(lift);
            rLift.setVelocity(lift);

            telemetry.addData("Field Centric", FIELD_CENTRIC_ON);
            telemetry.addData("y", y);
            telemetry.addData("x", x);
            telemetry.addData("r", r);
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

    private double limitAcceleration(double target, double current, double loopTime, double MAX_ACCEL)
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
        double newYaw = angles.getYaw(AngleUnit.RADIANS);
        filteredYaw = alpha * newYaw + (1 - alpha) * filteredYaw;
        return filteredYaw;
    }
}
