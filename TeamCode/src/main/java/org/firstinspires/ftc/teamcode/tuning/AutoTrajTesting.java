package org.firstinspires.ftc.teamcode.tuning;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;
// Servo imports
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
import static org.firstinspires.ftc.teamcode.Master.ServoParams.PIVOT_READJUST;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.PIVOT_UP;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.RIGHT_CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.RIGHT_CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.WRIST_DOWN;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.WRIST_READJUST;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.WRIST_UP;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.RIGHT_CLAW_READJUST;
import static org.firstinspires.ftc.teamcode.Master.ServoParams.LEFT_CLAW_READJUST;

@Config
@Autonomous(name = "red traj testing", group = "Autonomous")
public class AutoTrajTesting extends LinearOpMode {
    public DcMotor frontLeft, frontRight, rearLeft, rearRight;
    public DcMotor Elevatorright, Elevatorleft, Horizontalright, Horizontalleft;

    IMU imu;

    public Servo leftClaw, rightClaw, wrist, axle, outAxle, outWrist, outRightClaw, outLeftClaw;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the motors, servos, and IMU
        motorAndServoInit();

        Pose2d initialPoseSample1 = new Pose2d(-30, -63, Math.toRadians(90));
        Pose2d initialPoseBasketOuttake = new Pose2d(-48, -37, Math.PI/2);
        Pose2d initialPoseSample2 = new Pose2d(-55, -55, Math.toRadians(225));
        Pose2d initialPoseSample3 = new Pose2d(-55, -55, Math.toRadians(225));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPoseSample1);


        TrajectoryActionBuilder myTraj = drive.actionBuilder(initialPoseSample1)
                .waitSeconds(1)
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-48, -37, Math.PI/2), Math.PI/2);




        TrajectoryActionBuilder myTraj1 = drive.actionBuilder(initialPoseBasketOuttake)
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-55, -55), Math.toRadians(225));





        TrajectoryActionBuilder myTraj2 = drive.actionBuilder(initialPoseSample2)
                .waitSeconds(1)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-58, -37, Math.toRadians(90)), Math.toRadians(540));



        TrajectoryActionBuilder myTraj3 = drive.actionBuilder(initialPoseSample3)
                .waitSeconds(2)
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-60, -25, Math.toRadians(180)), Math.toRadians(180));



        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        myTraj.build()
                )
        );




    }

    public void Servo(String pos){
        if ( pos == "Reset"){
            rightClaw.setPosition(RIGHT_CLAW_OPEN);
            leftClaw.setPosition(LEFT_CLAW_OPEN);
            sleep(200);
            outLeftClaw.setPosition(OUT_LEFT_CLAW_CLOSE);
            outRightClaw.setPosition(OUT_RIGHT_CLAW_CLOSE);
            sleep(200);
            outWrist.setPosition(OUT_WRIST_UP);
            outAxle.setPosition(OUT_PIVOT_DOWN);
            sleep(200);
            wrist.setPosition(WRIST_READJUST);
            axle.setPosition(PIVOT_READJUST);
        } if ( pos == "Intake"){
            wrist.setPosition(WRIST_DOWN);
            axle.setPosition(PIVOT_DOWN);
            sleep(500);
            rightClaw.setPosition(RIGHT_CLAW_CLOSE);
            leftClaw.setPosition(LEFT_CLAW_CLOSE);
            sleep(700);
            wrist.setPosition(WRIST_UP);
            axle.setPosition(PIVOT_UP);
            rightClaw.setPosition(RIGHT_CLAW_READJUST);
            leftClaw.setPosition(LEFT_CLAW_READJUST);
            sleep(200);
        } if ( pos == "Outtake"){
            outLeftClaw.setPosition(OUT_LEFT_CLAW_OPEN);
            outRightClaw.setPosition(OUT_RIGHT_CLAW_OPEN);
            sleep(200);
            rightClaw.setPosition(RIGHT_CLAW_OPEN);
            leftClaw.setPosition(LEFT_CLAW_OPEN);
            sleep(100);
            wrist.setPosition(WRIST_READJUST);
            axle.setPosition(PIVOT_READJUST);
            sleep(500);
            outWrist.setPosition(OUT_WRIST_DOWN);
            outAxle.setPosition(OUT_PIVOT_UP);
        }


    }

    public void motorAndServoInit() {
        // Initialize drive motors
        frontLeft = hardwareMap.get(DcMotor.class, "left front");
        frontRight = hardwareMap.get(DcMotor.class, "right front");
        rearLeft = hardwareMap.get(DcMotor.class, "left rear");
        rearRight = hardwareMap.get(DcMotor.class, "right rear");

        // Initialize control motors
        Elevatorright = hardwareMap.get(DcMotor.class, "vertical 1");
        Elevatorleft = hardwareMap.get(DcMotor.class, "vertical 2");
        Horizontalright = hardwareMap.get(DcMotor.class, "horizontal 1");
        Horizontalleft = hardwareMap.get(DcMotor.class, "horizontal 2");

        // Set motor directions
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        rearLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        rearRight.setDirection(DcMotor.Direction.FORWARD);
        Horizontalleft.setDirection(DcMotor.Direction.REVERSE);
        Elevatorright.setDirection(DcMotor.Direction.REVERSE);

        // Initialize servos
        leftClaw = hardwareMap.get(Servo.class, "left claw servo");
        rightClaw = hardwareMap.get(Servo.class, "right claw servo");
        wrist = hardwareMap.get(Servo.class, "wrist servo");
        axle = hardwareMap.get(Servo.class, "pivot servo");

        outAxle = hardwareMap.get(Servo.class, "outtake servo");
        outLeftClaw = hardwareMap.get(Servo.class, "out left claw servo");
        outRightClaw = hardwareMap.get(Servo.class, "out right claw servo");
        outWrist = hardwareMap.get(Servo.class, "out wrist servo");

        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        imu.initialize(parameters);
    }
}
