package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Master.ServoParams.LEFT_CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.LEFT_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.OUT_LEFT_CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.OUT_LEFT_CLAW_OPEN;

import static org.firstinspires.ftc.teamcode.Master.ServoParams.OUT_PIVOT_DOWN;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.OUT_PIVOT_UP;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.OUT_RIGHT_CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.OUT_RIGHT_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.OUT_WRIST_DOWN;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.OUT_WRIST_UP;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.PIVOT_DOWN;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.PIVOT_UP;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.RIGHT_CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.RIGHT_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.WRIST_DOWN;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.WRIST_UP;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp
public class Master extends LinearOpMode {

    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor rearLeft;
    public DcMotor rearRight;

    public DcMotor Elevatorright;

    public DcMotor Elevatorleft;

    public DcMotor Horizontalright;

    public DcMotor Horizontalleft;



    IMU imu;

    public DcMotor elevatorright;

    // port 1 control hub
    public Servo leftClaw;
    //port2 control hub
    public Servo rightClaw;
    //port0 control hub
    public Servo wrist;
    // port 3 control hub
    public Servo axle;
    //port 0 expansion hub
    public Servo outAxle;
    //port 1 expansion hub
    public Servo outWrist;
    // port 2 expansion hub
    public Servo outRightClaw;
    // port 3 expansion hub
    public Servo outLeftClaw;

    public static class ServoParams {
        public static final double LEFT_CLAW_CLOSE = 0.075;

        public static final double LEFT_CLAW_READJUST = 0.085;

        public static final double LEFT_CLAW_OPEN = 0.3;

        public static final double RIGHT_CLAW_CLOSE = 0.55;

        public static final double RIGHT_CLAW_READJUST = 0.51;
        public static final double RIGHT_CLAW_OPEN = 0.3;

        public static final double WRIST_UP = .2;
        public static final double WRIST_DOWN = 1;

        public static final double PIVOT_UP = 0;
        public static final double PIVOT_DOWN = 1;

        public static final double OUT_LEFT_CLAW_OPEN = 1;
        public static final double OUT_LEFT_CLAW_CLOSE = .4;

        public static final double OUT_RIGHT_CLAW_OPEN = .7;
        public static final double OUT_RIGHT_CLAW_CLOSE = 1;

        public static final double OUT_PIVOT_UP = 0;
        public static final double OUT_PIVOT_DOWN = 1;

        public static final double OUT_WRIST_UP = .81;
        public static final double OUT_WRIST_DOWN = .65;

        public static final int elevatorup = 3000;
    }

    @Override
    public void runOpMode() throws InterruptedException {


    }

    public void motorInit() {

        frontLeft = hardwareMap.get(DcMotor.class, "left front");
        frontRight = hardwareMap.get(DcMotor.class, "right front");
        rearLeft = hardwareMap.get(DcMotor.class, "left rear");
        rearRight = hardwareMap.get(DcMotor.class, "right rear");

        leftClaw = hardwareMap.get(Servo.class, "left claw servo");
        rightClaw = hardwareMap.get(Servo.class, "right claw servo");
        wrist = hardwareMap.get(Servo.class, "wrist servo");
        axle = hardwareMap.get(Servo.class, "pivot servo");

        outAxle = hardwareMap.get(Servo.class, "outtake servo");
        outLeftClaw = hardwareMap.get(Servo.class, "out left claw servo");
        outRightClaw = hardwareMap.get(Servo.class, "out right claw servo");
        outWrist = hardwareMap.get(Servo.class, "out wrist servo");

       Elevatorright = hardwareMap.get(DcMotor.class, "vertical 1");
       Elevatorleft = hardwareMap.get(DcMotor.class, "vertical 2");
       Horizontalright = hardwareMap.get(DcMotor.class, "horizontal 1");
       Horizontalleft = hardwareMap.get(DcMotor.class, "horizontal 2");
        // Change to match drivetrain
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        rearLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        rearRight.setDirection(DcMotor.Direction.FORWARD);
        Horizontalleft.setDirection(DcMotor.Direction.REVERSE);
        Elevatorleft.setDirection(DcMotor.Direction.REVERSE);
        imu = hardwareMap.get(IMU.class, "imu");

        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));

        // initialize the new parameter or go with the assumed - logo up / USB forward
        imu.initialize(parameters);
    }

    public void centricMovement(Boolean fieldCentric) {

        motorInit();



       if (fieldCentric == Boolean.TRUE) {

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // just to reset for driver control.
            if (gamepad1.start) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            // clips the power to not exceed the maximum value.
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeft.setPower(frontLeftPower);
            rearLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            rearRight.setPower(backRightPower);

        } else if (fieldCentric == Boolean.FALSE) {

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // Clipping the power to make sure it doesn't exceed the maximum value
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeft.setPower(frontLeftPower);
            rearLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            rearRight.setPower(backRightPower);

        }
    }

    public void servoMovements() {
        leftClaw = hardwareMap.get(Servo.class, "left claw servo");
        rightClaw = hardwareMap.get(Servo.class, "right claw servo");
        wrist = hardwareMap.get(Servo.class, "wrist servo");
        axle = hardwareMap.get(Servo.class, "pivot servo");

        outAxle = hardwareMap.get(Servo.class, "outtake servo");
        outLeftClaw = hardwareMap.get(Servo.class, "out left claw servo");
        outRightClaw = hardwareMap.get(Servo.class, "out right claw servo");
        outWrist = hardwareMap.get(Servo.class, "out wrist servo");

        if (gamepad1.dpad_up) {
            wrist.setPosition(WRIST_UP);
            axle.setPosition(PIVOT_UP);
        } else if (gamepad1.dpad_down) {
            wrist.setPosition(WRIST_DOWN);
            axle.setPosition(PIVOT_DOWN);
        } else if (gamepad1.right_bumper) {
            rightClaw.setPosition(RIGHT_CLAW_CLOSE);
            leftClaw.setPosition(LEFT_CLAW_CLOSE);
        } else if (gamepad1.left_bumper) {
            rightClaw.setPosition(RIGHT_CLAW_OPEN);
            leftClaw.setPosition(LEFT_CLAW_OPEN);
        } else if (gamepad2.a) {
            outRightClaw.setPosition(OUT_RIGHT_CLAW_CLOSE);
            outLeftClaw.setPosition(OUT_LEFT_CLAW_CLOSE);
        } else if (gamepad2.b) {
            outLeftClaw.setPosition(OUT_LEFT_CLAW_OPEN);
            outRightClaw.setPosition(OUT_RIGHT_CLAW_OPEN);
        } else if (gamepad1.y) {
            outWrist.setPosition(OUT_WRIST_UP);
            outAxle.setPosition(OUT_PIVOT_UP);
        } else if (gamepad1.a) {
            outWrist.setPosition(OUT_WRIST_DOWN);
            outAxle.setPosition(OUT_PIVOT_DOWN);
        }

    }
}
