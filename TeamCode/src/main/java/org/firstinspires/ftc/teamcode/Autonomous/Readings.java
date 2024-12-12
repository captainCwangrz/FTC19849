package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "Readings", group = "Testing")
public class Readings extends LinearOpMode
{
    DcMotorEx lf;
    DcMotorEx rf;
    DcMotorEx lb;
    DcMotorEx rb;

    IMU imu;
    RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
    double filteredYaw = 0.0;
    double alpha = 0.2;

    double maxRPM = 312;
    double TPR = 537.7;
    double gearing = 19.2;

    @Override
    public void runOpMode()
    {
        lf = hardwareMap.get(DcMotorEx.class, "lf");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        lb = hardwareMap.get(DcMotorEx.class, "lb");
        rb = hardwareMap.get(DcMotorEx.class, "rb");

        lf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        lf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        initIMU();

        waitForStart();

        while (opModeIsActive())
        {


            break;
        }
    }

    private void initIMU()
    {
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        IMU.Parameters imuParams = new IMU.Parameters(orientationOnRobot);
        imu.initialize(imuParams);
        imu.resetYaw(); //careful here, might wanna store inital heading from autoOp first
    }

    private double getHeadingRadian()
    {
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        double newYaw = angles.getYaw(AngleUnit.RADIANS);
        filteredYaw = alpha * newYaw + (1 - alpha) * filteredYaw;
        return filteredYaw;
    }
}
