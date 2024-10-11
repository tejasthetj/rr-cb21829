package org.firstinspires.ftc.teamcode.tuning;

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
import static org.firstinspires.ftc.teamcode.Master.ServoParams.RIGHT_CLAW_READJUST;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.LEFT_CLAW_READJUST;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Teleop", group = "Exercises")
public class master_copy extends LinearOpMode {

    public DcMotor frontLeft, frontRight, rearLeft, rearRight;
    public DcMotor Elevatorright, Elevatorleft, Horizontalright, Horizontalleft;

    IMU imu;

    public Servo leftClaw, rightClaw, wrist, axle, outAxle, outWrist, outRightClaw, outLeftClaw;


    public static class ServoParams {
        public static final double LEFT_CLAW_CLOSE = 0.075;
        public static final double LEFT_CLAW_OPEN = 0.3;

        public static final double RIGHT_CLAW_CLOSE = 0.55;
        public static final double RIGHT_CLAW_OPEN = 0.3;

        public static final double WRIST_UP = .2;
        public static final double WRIST_DOWN = 1;

        public static final double PIVOT_UP = 0.075;
        public static final double PIVOT_DOWN = 1;

        public static final double OUT_LEFT_CLAW_OPEN = 1;
        public static final double OUT_LEFT_CLAW_CLOSE = .4;

        public static final double OUT_RIGHT_CLAW_OPEN = .7;
        public static final double OUT_RIGHT_CLAW_CLOSE = 1;

        public static final double OUT_PIVOT_UP = 0;
        public static final double OUT_PIVOT_DOWN = 0.85;

        public static final double OUT_WRIST_UP = .85;
        public static final double OUT_WRIST_DOWN = .65;

    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the hardware
        motorInit();

        // Wait for the start button to be pressed
        waitForStart();

        // Main loop, runs until stop is pressed
        while (opModeIsActive()) {
            // Perform movement based on field-centric or robot-centric control
            centricMovement(true);

            // Handle servo movements
            servoMovements();


        }
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

        // Set motor directions based on the robot configuration
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        rearLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        rearRight.setDirection(DcMotor.Direction.FORWARD);
        Horizontalright.setDirection(DcMotorSimple.Direction.REVERSE);
        Elevatorright.setDirection(DcMotorSimple.Direction.REVERSE);



        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));

        // Initialize the IMU with parameters
        imu.initialize(parameters);
    }

    public void centricMovement(Boolean fieldCentric) {
        // Initialize motors before any movement

        motorInit();

        if (fieldCentric == Boolean.FALSE) {
            // Field-centric control
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;


            if (gamepad1.start) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            rotX = rotX * 1.1;
            double vert = gamepad2.right_stick_y;
            double hor = gamepad2.right_stick_x;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;
            double verticalPower = vert;
            double horizontalPower = hor;

            frontLeft.setPower(frontLeftPower);
            rearLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            rearRight.setPower(backRightPower);
            Elevatorright.setPower(verticalPower);
            Elevatorleft.setPower(0.42*verticalPower);
            Horizontalleft.setPower(horizontalPower);
            Horizontalright.setPower(horizontalPower);

        } else if (fieldCentric == Boolean.TRUE) {
            // Robot-centric control
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            double vert = gamepad2.right_stick_y;
            double hor = gamepad2.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 2);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;
            double verticalPower = vert;
            double horizontalPower = hor;

            Elevatorright.setPower(verticalPower);
            Elevatorleft.setPower(0.42*verticalPower);
            Horizontalleft.setPower(horizontalPower);
            Horizontalright.setPower(horizontalPower);

            frontLeft.setPower(frontLeftPower);
            rearLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            rearRight.setPower(backRightPower);


        }

    }

    public void servoMovements()  {



        if (gamepad1.dpad_up) {
            wrist.setPosition(WRIST_UP);
            axle.setPosition(PIVOT_UP);
            rightClaw.setPosition(RIGHT_CLAW_READJUST);
            leftClaw.setPosition(LEFT_CLAW_READJUST);
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
        } else if (gamepad1.b) {
            outWrist.setPosition(OUT_WRIST_UP);
            outAxle.setPosition(OUT_PIVOT_UP);
        } else if (gamepad1.a) {
            outWrist.setPosition(OUT_WRIST_DOWN);
            outAxle.setPosition(OUT_PIVOT_DOWN);
        } else if (gamepad2.right_bumper) {



            rightClaw.setPosition(RIGHT_CLAW_CLOSE);
            leftClaw.setPosition(LEFT_CLAW_CLOSE);
            sleep(500);
            wrist.setPosition(WRIST_UP);
            axle.setPosition(PIVOT_UP);
            rightClaw.setPosition(RIGHT_CLAW_READJUST);
            leftClaw.setPosition(LEFT_CLAW_READJUST);
            sleep(200);



        } else if (gamepad2.left_bumper){

            rightClaw.setPosition(RIGHT_CLAW_OPEN);
            leftClaw.setPosition(LEFT_CLAW_OPEN);
            sleep(200);
            outLeftClaw.setPosition(OUT_LEFT_CLAW_CLOSE);
            outRightClaw.setPosition(OUT_RIGHT_CLAW_CLOSE);
            sleep(200);
            outWrist.setPosition(OUT_WRIST_UP);
            outAxle.setPosition(OUT_PIVOT_UP);
            sleep(200);
            wrist.setPosition(WRIST_DOWN);
            axle.setPosition(PIVOT_DOWN);
            sleep(200);

        }

    }




    }


