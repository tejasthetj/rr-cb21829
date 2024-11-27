package org.firstinspires.ftc.teamcode.autoTrajTesting;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
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

import java.util.Random;

@Config
@Autonomous(name = "red traj testing", group = "Autonomous")
public class AutoTrajTesting extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        //Pose2d initialPose = new Pose2d(-30, -63, Math.toRadians(90));
        Pose2d redCloseStartingPose = new Pose2d(-30, -63, Math.toRadians(90));
        Pose2d initialPoseBasketOuttake = new Pose2d(-48, -40, Math.PI/2);
        Pose2d initialPoseBasketOuttake1 = new Pose2d(-58, -37, Math.toRadians(90));
        Pose2d initialPoseBasketOuttake2 = new Pose2d(-60, -25, Math.toRadians(180));
        Pose2d initialPoseSample2 = new Pose2d(-55, -55, Math.toRadians(45));
        Pose2d initialPoseSample3 = new Pose2d(-55, -55, Math.toRadians(225));
        MecanumDrive drive = new MecanumDrive(hardwareMap, redCloseStartingPose);
        AllMechForRR robot = new AllMechForRR(hardwareMap);


        TrajectoryActionBuilder firstSample = drive.actionBuilder(redCloseStartingPose)
                .waitSeconds(1)
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-48, -40 , Math.PI/2), Math.PI/2);
        TrajectoryActionBuilder myTraj1 = drive.actionBuilder(initialPoseBasketOuttake)
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-55, -55), Math.toRadians(45));
        TrajectoryActionBuilder myTraj2 = drive.actionBuilder(initialPoseSample2)
                .waitSeconds(1)
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-60, -41, Math.toRadians(90)), Math.toRadians(540));
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

        
        // Test runblocking for testing PID control loop.
//        Actions.runBlocking(
//                new ParallelAction(
//                    robot.updatePID(),
//                    new SequentialAction(
//                        robot.setElevatorTarget(500),
//                        new SleepAction(2),
//                        robot.setElevatorTarget(3000),
//                        new SleepAction(2),
//                        robot.setElevatorTarget(20)
//                    )
//                )
//
//        );

         Actions.runBlocking(
                 new ParallelAction(
                         robot.updatePID(),
                         new SequentialAction(
                                 robot.resetClassAction(),
                                 firstSample.build(),
                                 robot.intakeClawAction(),
                                 robot.outtakeClawAction(),
                                 robot.setElevatorTarget(3300),
                                 myTraj1.build(),
                                 robot.resetClassAction(),
                                 robot.setElevatorTarget(20),
                                 myTraj2.build(),
                                 robot.intakeClawAction(),
                                 robot.outtakeClawAction(),
                                 robot.setElevatorTarget(3300),
                                 myTraj4.build(),
                                 robot.resetClassAction(),
                                 robot.setElevatorTarget(20),
                                 myTraj3.build(),
                                 robot.intakeClawAction(),
                                 robot.outtakeClawAction(),
                                 robot.setElevatorTarget(3300),
                                 myTraj5.build(),
                                 robot.resetClassAction()
                         )

                 )
         );

    }
}
