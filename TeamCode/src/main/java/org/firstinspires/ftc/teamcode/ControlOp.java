package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ControlOp extends LinearOpMode
{
    //声明底盘电机
    private DcMotor lf;
    private DcMotor lb;
    private DcMotor rf;
    private DcMotor rb;

    //声明升降电机
    private DcMotor pulley;

    //声明爪子舵机
    private Servo leftClaw;
    private Servo rightClaw;

    //声明肘部舵机
    private Servo leftElbow;
    private Servo rightElbow;

    //定义控制参数(可调)
    private double leftClawPos = 0.5; //左爪舵机角度
    private double rightClawPos = 0.5; //右爪舵机角度
    private double leftElbowPos = 0.5; //左肘舵机角度
    private double rightElbowPos = 0.5; //右肘舵机角度
    private final double clawSpeed = 0.005; //爪子张合速度
    private final double elbowSpeed = 0.003; //肘部旋转速度
    private final double pulleySpeed = 0.4; //升降电机功率

    @Override
    public void runOpMode()
    {
        /*
            注意配置文件中的硬件名称
            左前电机: lf
            右前电机: rf
            左后电机: lb
            右后电机: rb

            升降电机: pulley

            左爪舵机: leftclaw
            右爪舵机: rightclaw

            左肘舵机: leftelbow
            右肘舵机: rightelbow
         */
        //初始化底盘电机
        lf = hardwareMap.get(DcMotor.class, "lf");
        lb = hardwareMap.get(DcMotor.class, "lb");
        rf = hardwareMap.get(DcMotor.class, "rf");
        rb = hardwareMap.get(DcMotor.class, "rb");
        //设置底盘左侧电机反转(电机对称安装方式原因)
        lf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);
        //设置底盘电机0速模式为带载
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //初始化升降电机
        pulley = hardwareMap.get(DcMotor.class, "pulley");
        //设置升降电机反转(电机安装方向原因)
        pulley.setDirection(DcMotor.Direction.REVERSE);
        //设置升降电机0速模式为带载
        pulley.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //初始化爪子舵机
        leftClaw = hardwareMap.get(Servo.class, "leftclaw");
        rightClaw =hardwareMap.get(Servo.class, "rightclaw");
        //初始化爪子舵机角度
        leftClaw.setPosition(leftClawPos);
        rightClaw.setPosition((rightClawPos));

        //初始化肘部舵机
        leftElbow = hardwareMap.get(Servo.class, "leftelbow");
        rightElbow = hardwareMap.get(Servo.class, "rightelbow");
        //初始化肘部舵机角度
        leftElbow.setPosition(leftElbowPos);
        rightElbow.setPosition(rightElbowPos);

        //初始化完毕,等待程序开始
        waitForStart();

        //主循环
        while (opModeIsActive())
        {
            /*
                此程序中使用以下手柄逻辑(测试用)
                手柄1:
                    左摇杆上下: 底盘前后平移
                    左摇杆左右: 底盘左右平移
                    右摇杆左右: 底盘左右旋转
                    上下方向键: 升降
                手柄2:
                    上下方向键: 肘关节旋转
                    左右方向键: 爪子张合
             */
            //获取手柄1输入
            double ly = -gamepad1.left_stick_y; //乘以-1颠倒y轴正方向因为y轴正方向默认为下方
            double lx = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            boolean up = gamepad1.dpad_up;
            boolean down = gamepad1.dpad_down;
            //获取手柄2输入
            boolean elbowUp = gamepad2.dpad_up;
            boolean elbowDown = gamepad2.dpad_down;
            boolean openClaw = gamepad2.dpad_left;
            boolean closeClaw = gamepad2.dpad_right;

            //计算底盘电机功率
            double lfPower = ly + lx + rx;
            double rfPower = ly - lx - rx;
            double lbPower = ly - lx + rx;
            double rbPower = ly + lx - rx;
            //底盘电机功率常规化
            double max = Math.max(Math.abs(lfPower), Math.abs(rfPower));
            max = Math.max(max, Math.abs(lbPower));
            max = Math.max(max, Math.abs(rbPower));
            if (max > 1) {
                lfPower /= max;
                rfPower /= max;
                lbPower /= max;
                rbPower /= max;
            }

            //输出底盘电机功率
            lf.setPower(lfPower);
            rf.setPower(rfPower);
            lb.setPower(lbPower);
            rb.setPower(rbPower);

            //升降电机控制逻辑
            if (up)
            {
                pulley.setPower(pulleySpeed);
            }
            else if (down)
            {
                pulley.setPower(-pulleySpeed);
            }
            else
            {
                pulley.setPower(0);
            }

            //爪子舵机控制逻辑
            if (openClaw)
            {
                leftClawPos += clawSpeed;
                rightClawPos -= clawSpeed;
            }
            else if (closeClaw)
            {
                leftClawPos -= clawSpeed;
                rightClawPos += clawSpeed;
            }

            //肘部舵机控制逻辑
            if (elbowUp)
            {
                leftElbowPos += elbowSpeed;
                rightElbowPos -= elbowSpeed;
            }
            else if (elbowDown)
            {
                leftElbowPos -= elbowSpeed;
                rightElbowPos += elbowSpeed;
            }

            //舵机角度输出
            leftClaw.setPosition(leftClawPos);
            rightClaw.setPosition(rightClawPos);
            leftElbow.setPosition(leftElbowPos);
            rightElbow.setPosition(rightElbowPos);
        }
    }
}
