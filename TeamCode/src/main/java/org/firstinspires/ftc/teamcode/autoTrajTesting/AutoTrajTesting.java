package org.firstinspires.ftc.teamcode.autoTrajTesting;

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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "red traj testing", group = "Autonomous")
public class AutoTrajTesting extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        //Pose2d initialPose = new Pose2d(-30, -63, Math.toRadians(90));
        Pose2d initialPoseSample1 = new Pose2d(-30, -63, Math.toRadians(90));
        Pose2d initialPoseBasketOuttake = new Pose2d(-48, -37, Math.PI/2);
        Pose2d initialPoseBasketOuttake1 = new Pose2d(-58, -37, Math.toRadians(90));
        Pose2d initialPoseBasketOuttake2 = new Pose2d(-60, -25, Math.toRadians(180));
        Pose2d initialPoseSample2 = new Pose2d(-55, -55, Math.toRadians(225));
        Pose2d initialPoseSample3 = new Pose2d(-55, -55, Math.toRadians(225));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPoseSample1);
        AllMechForRR robot = new AllMechForRR(hardwareMap);

        TrajectoryActionBuilder myTraj = drive.actionBuilder(initialPoseSample1)
                .waitSeconds(1)
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-48, -40 , Math.PI/2), Math.PI/2);
        TrajectoryActionBuilder myTraj1 = drive.actionBuilder(initialPoseBasketOuttake)
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-55, -55), Math.toRadians(45));
        TrajectoryActionBuilder myTraj2 = drive.actionBuilder(initialPoseSample2)
                .waitSeconds(1)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-58, -41, Math.toRadians(90)), Math.toRadians(540));
        TrajectoryActionBuilder myTraj3 = drive.actionBuilder(initialPoseSample3)
                .waitSeconds(2)
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-55, -25, Math.toRadians(180)), Math.toRadians(180));
        TrajectoryActionBuilder myTraj4 = drive.actionBuilder(initialPoseBasketOuttake1)
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-55, -55), Math.toRadians(45));
        TrajectoryActionBuilder myTraj5 = drive.actionBuilder(initialPoseBasketOuttake2)
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-55, -55), Math.toRadians(45));

        waitForStart();
        Actions.runBlocking(
                new SequentialAction(
                        robot.resetClassAction(),
                        myTraj.build(),
                        robot.intakeClawAction(),
                        robot.outtakeClawAction(),
                        myTraj1.build(),
                        robot.resetClassAction(),
                        myTraj2.build(),
                        robot.intakeClawAction(),
                        robot.outtakeClawAction(),
                        myTraj4.build(),
                        robot.resetClassAction(),
                        myTraj3.build(),
                        robot.intakeClawAction(),
                        robot.outtakeClawAction(),
                        myTraj5.build(),
                        robot.resetClassAction()
                )
        );

    }
}
