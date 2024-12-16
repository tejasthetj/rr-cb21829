package org.firstinspires.ftc.teamcode.teleop.actionsTesting;

import static org.firstinspires.ftc.teamcode.Master.ServoParams.LEFT_CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.LEFT_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.OUT_LEFT_CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.OUT_LEFT_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.OUT_PIVOT_DOWN;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.OUT_PIVOT_UP;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.OUT_PIVOT_UP_Tele;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.OUT_RIGHT_CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.OUT_RIGHT_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.OUT_WRIST_DOWN;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.OUT_WRIST_DOWN_Tele;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.OUT_WRIST_SPECIMEN;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.OUT_WRIST_SPECIMEN_Tele;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.OUT_WRIST_UP;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.PIVOT_DOWN;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.PIVOT_READJUST;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.PIVOT_UP;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.RIGHT_CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.RIGHT_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.WRIST_DOWN;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.WRIST_READJUST;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.WRIST_UP;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.RIGHT_CLAW_READJUST;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.LEFT_CLAW_READJUST;
import static org.firstinspires.ftc.teamcode.autoTrajTesting.AllMechForRR.fh;

import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PDController;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import java.util.Objects;

@TeleOp(name = "Teleop working", group = "Exercises")
public class master_copy extends LinearOpMode {

    private PIDController rightVertController;
    private PIDController leftVertController;
    private PIDController rightHorController;
    private PIDController leftHorController;

    public static double pv = 0.007, iv = 0, dv = 0.0002;
    public static double ph = 0.005, ih = 0, dh = 0.0001;
    public static double fv = 0.05, fh = 0;

    public static int vertTarget = 0;
    public static int horTarget;


    private final double ticks_in_degrees = 576.7/180;

    private OpenCvCamera camera;

    public DcMotor frontLeft, frontRight, rearLeft, rearRight;
    public DcMotor elevatorRight, elevatorLeft, horizontalRight, horizontalLeft;
    IMU imu;

    public Servo leftClaw, rightClaw, wrist, axle, outAxle, outWrist, outRightClaw, outLeftClaw;

    private MecanumDrive drive;

    private Pose2d initialPose;// Variable for the initial pose

    private Pose2d initial_pose = new Pose2d(-55, -55, Math.toRadians(225));
    private boolean initialPoseSet = false;  // Flag to track if the initial pose is set

    @Override
    public void runOpMode() throws InterruptedException {


        // Initialize the hardware and camera
        motorInit();

        // Initialize MecanumDrive here, assuming you have it set up
        drive = new MecanumDrive(hardwareMap,initial_pose);

        // Wait for the start button to be pressed
        waitForStart();


        // Main loop, runs until stop is pressed
        while (opModeIsActive()) {
             if (gamepad2.b){

                rightClaw.setPosition(RIGHT_CLAW_OPEN);
                leftClaw.setPosition(LEFT_CLAW_OPEN);
                outLeftClaw.setPosition(OUT_LEFT_CLAW_CLOSE);
                outRightClaw.setPosition(OUT_RIGHT_CLAW_CLOSE);

                outWrist.setPosition(OUT_WRIST_UP);
                outAxle.setPosition(OUT_PIVOT_DOWN);
                wrist.setPosition(WRIST_READJUST);
                axle.setPosition(PIVOT_READJUST);


            }



            centricMovement(true);  // Perform movement based on field-centric control
            servoMovements();// Handle servo movements
            Elevator_set();


            telemetry.update();  // Update telemetry
        }

        // Close the camera after stopping the OpMode
        closeCamera();
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

        elevatorRight = hardwareMap.get(DcMotor.class, "vertical 1");
        elevatorLeft = hardwareMap.get(DcMotor.class, "vertical 2");
        horizontalRight = hardwareMap.get(DcMotor.class, "horizontal 1");
        horizontalLeft = hardwareMap.get(DcMotor.class, "horizontal 2");

        // Set motor directions based on the robot configuration
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        rearLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        rearRight.setDirection(DcMotor.Direction.FORWARD);
        horizontalRight.setDirection(DcMotorSimple.Direction.REVERSE);
        elevatorLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        elevatorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevatorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightVertController = new PIDController(pv,iv,dv);
        leftVertController = new PIDController(pv, iv, dv);
        rightHorController = new PIDController(ph,0,dh);
        leftHorController = new PIDController(ph,0,dh);

        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));

        // Initialize the IMU with parameters
        imu.initialize(parameters);
    }

    private void initializeCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Initialize the pipeline


        // Start streaming from the camera asynchronously
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
            }
        });
    }

    private void closeCamera() {
        if (camera != null) {
            camera.closeCameraDevice();
        }
    }

    public void centricMovement(Boolean fieldCentric) {
        if (!fieldCentric) {
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

            horizontalLeft.setPower(horizontalPower);
            horizontalRight.setPower(horizontalPower);

        } else {
            // Robot-centric control
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            double vert = gamepad2.left_stick_y;
            double hor = gamepad2.right_stick_y;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            if (gamepad1.right_bumper){
                denominator = 2;

            }
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;
            double verticalPower = vert;
            double horizontalPower = hor;

            frontLeft.setPower(frontLeftPower);
            rearLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            rearRight.setPower(backRightPower);
            horizontalLeft.setPower(horizontalPower);
            horizontalRight.setPower(horizontalPower);
        }
    }


    public void servoMovements()  {



         if (gamepad2.a) {

            wrist.setPosition(WRIST_DOWN);
            axle.setPosition(PIVOT_DOWN);
            sleep(200);
            rightClaw.setPosition(RIGHT_CLAW_CLOSE);
            leftClaw.setPosition(LEFT_CLAW_CLOSE);
            sleep(250);
            wrist.setPosition(WRIST_UP);
            axle.setPosition(PIVOT_UP);
            rightClaw.setPosition(RIGHT_CLAW_READJUST);
            leftClaw.setPosition(LEFT_CLAW_READJUST);





        }
        else if(gamepad2.x){
            outLeftClaw.setPosition(OUT_LEFT_CLAW_OPEN);
            outRightClaw.setPosition(OUT_RIGHT_CLAW_OPEN);
            sleep(200);
            rightClaw.setPosition(RIGHT_CLAW_OPEN);
            leftClaw.setPosition(LEFT_CLAW_OPEN);
            sleep(100);
            wrist.setPosition(WRIST_READJUST);
            axle.setPosition(PIVOT_READJUST);
            sleep(500);
            outWrist.setPosition(OUT_WRIST_DOWN_Tele);
            outAxle.setPosition(OUT_PIVOT_UP_Tele);


        }
        else if (gamepad2.y){
            outWrist.setPosition(OUT_WRIST_SPECIMEN_Tele);


        }

    }

    public void Elevator_set() {
        rightVertController.setPID(pv,iv,dv);
        leftVertController.setPID(pv,iv,dv);
        rightHorController.setPID(ph,0,dh);
        leftHorController.setPID(ph,0,dh);

        int rightVertPos = elevatorRight.getCurrentPosition();
        int leftVertPos = elevatorLeft.getCurrentPosition();
        int rightHorPos = horizontalRight.getCurrentPosition();
        int leftHorPos = horizontalLeft.getCurrentPosition();

        double rightVertPid = rightVertController.calculate(rightVertPos,vertTarget);
        double leftVertPid = leftVertController.calculate(leftVertPos,vertTarget);
        double rightHorPid = rightHorController.calculate(rightHorPos, horTarget);
        double leftHorPid = leftHorController.calculate(leftHorPos, horTarget);

        double vertFF = Math.cos(Math.toRadians(vertTarget / ticks_in_degrees)) * fv;
        double hotFF = Math.cos(Math.toRadians(horTarget / ticks_in_degrees)) * fh;

        double rightVertPower = rightVertPid + vertFF;
        double leftVertPower = leftVertPid + vertFF;
        double rightHorPower = rightHorPid + hotFF;
        double leftHorPower = leftHorPid + hotFF;

        elevatorRight.setPower(rightVertPower);
        elevatorLeft.setPower(leftVertPower);
//        horizontalRight.setPower(rightHorPower);
//        horizontalLeft.setPower(leftHorPower);

        if (gamepad2.dpad_up) {

            vertTarget = 3600;

        } else if (gamepad2.dpad_down) {

            vertTarget = 40;
        }
        else if  (gamepad2.dpad_left){

            vertTarget = 1800;

        }
        else if (gamepad2.dpad_right){

            vertTarget = 1100;
        }
    }






}