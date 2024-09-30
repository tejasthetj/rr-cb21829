package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Robot Centric", group = "exercises")
public class TeleOpProgram extends LinearOpMode {

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor rearLeft;
    DcMotor rearRight;

    // port1 controlhub
    public Servo leftClaw;
    //port2 controlhub
    public Servo rightClaw;
    //port0 control
    public Servo wrist;
    // port 3 control hub
    public Servo lebronJames;
    //port 0 expansion hub
    public Servo bethBurry;
    //port 1 expansion hub
    public Servo outWrist;
    // port 2 expansion hub
    public Servo outRightClaw;
    // port 3 expansion hub
    public Servo outLeftClaw;

    public static final double LEFT_CLAW_CLOSE = 0.075;
    public static final double LEFT_CLAW_OPEN = 0.3;

    public static final double RIGHT_CLAW_CLOSE = 0.55;
    public static final double RIGHT_CLAW_OPEN = 0.3;

    public static final double WRIST_UP = 1;
    public static final double WRIST_DOWN = 0.5;

    public static final double PIVOT_UP = 1;
    public static final double PIVOT_DOWN = 0;

    public static final double OUT_LEFT_CLAW_OPEN = 0;
    public static final double OUT_LEFT_CLAW_CLOSE = 1;

    public static final double OUT_RIGHT_CLAW_OPEN = 0;
    public static final double OUT_RIGHT_CLAW_CLOSE = 1;

    public static final double OUT_PIVOT_UP = 0;
    public static final double OUT_PIVOT_DOWN = 1;


    Boolean fCentric = Boolean.FALSE;

    IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {

        //Change to match DS names
        frontLeft = hardwareMap.get(DcMotor.class, "left front");
        frontRight = hardwareMap.get(DcMotor.class, "right front");
        rearLeft = hardwareMap.get(DcMotor.class, "left rear");
        rearRight = hardwareMap.get(DcMotor.class, "right rear");

        leftClaw = hardwareMap.get(Servo.class, "left claw servo");
        rightClaw = hardwareMap.get(Servo.class, "right claw servo");
        wrist = hardwareMap.get(Servo.class, "wrist servo");
        lebronJames = hardwareMap.get(Servo.class, "pivot servo");

        bethBurry = hardwareMap.get(Servo.class, "outtake servo");
        outLeftClaw = hardwareMap.get(Servo.class, "out left claw servo");
        outRightClaw = hardwareMap.get(Servo.class, "out right claw servo");
        outWrist = hardwareMap.get(Servo.class, "out wrist servo");

        // Change to match drivetrain
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        rearLeft.setDirection(DcMotor.Direction.REVERSE);
        rearRight.setDirection(DcMotor.Direction.FORWARD);

        // Change to match DS name
        imu = hardwareMap.get(IMU.class, "imu");

        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        // initialize the new parameter or go with the assumed - logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        while (opModeInInit()) {
            if (gamepad1.a) {
                fCentric = Boolean.FALSE;
            } else if (gamepad1.b) {
                fCentric = Boolean.TRUE;
            }
        }

        while (opModeIsActive()) {

            if (fCentric == Boolean.FALSE) {

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

            } else if (fCentric == Boolean.TRUE) {

                double y = -gamepad1.left_stick_y;
                double x = gamepad1.left_stick_x;
                double rx = gamepad1.right_stick_x;

                // just to reset for driver control.
                if (gamepad1.options) {
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
            }



        }

    }
}
