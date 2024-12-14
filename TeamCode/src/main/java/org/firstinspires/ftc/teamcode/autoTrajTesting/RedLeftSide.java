package org.firstinspires.ftc.teamcode.autoTrajTesting;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "red left traj", group = "Autonomous")
public class RedLeftSide extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d redCloseStartingPose = new Pose2d(-30, -63, Math.toRadians(270));
        Pose2d initialPoseBasketOuttake = new Pose2d(-48, -40, Math.PI/2);
        Pose2d initialPoseBasketOuttake1 = new Pose2d(-58, -37, Math.toRadians(90));
        Pose2d initialPoseBasketOuttake2 = new Pose2d(-60, -25, Math.toRadians(180));
        Pose2d initialPoseSample2 = new Pose2d(-57, -75, Math.toRadians(45));
        Pose2d initialPoseSample3 = new Pose2d(-57, -57, Math.toRadians(45));
        MecanumDrive drive = new MecanumDrive(hardwareMap, redCloseStartingPose);
        AllMechForRR robot = new AllMechForRR(hardwareMap);


        TrajectoryActionBuilder dropSpecimen = drive.actionBuilder(redCloseStartingPose)
                .strafeTo(new Vector2d(0, -37));
        TrajectoryActionBuilder firstSample = drive.actionBuilder(new Pose2d(0, -37, Math.toRadians(270)))
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-48, -44 , Math.PI/2), Math.PI/2);
        TrajectoryActionBuilder dropFirstSample = drive.actionBuilder(new Pose2d(-48, -44, Math.PI/2))
                .strafeToLinearHeading(new Vector2d(-53, -59), Math.toRadians(45));
        TrajectoryActionBuilder pickSecondSample = drive.actionBuilder(new Pose2d(-53, -59, Math.toRadians(45)))
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-60, -44, Math.toRadians(90)), Math.toRadians(540));
        TrajectoryActionBuilder pickThirdSample = drive.actionBuilder(new Pose2d(-53, -59, Math.toRadians(45)))
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(-52, -45, Math.toRadians(135)), Math.toRadians(90));
        TrajectoryActionBuilder dropSecondSample = drive.actionBuilder(new Pose2d(-60, -44, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(-53, -59), Math.toRadians(45));
        TrajectoryActionBuilder dropThirdSample = drive.actionBuilder(new Pose2d(-52, -45, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(-53, -59), Math.toRadians(45));

        waitForStart();

         Actions.runBlocking(
                 new ParallelAction(
                         robot.updatePID(),
                         new SequentialAction(
                                 new ParallelAction(
                                         robot.specimenOuttakeClawAction(),
                                         robot.setElevatorTarget(1300),
                                         dropSpecimen.build()
                                 ),
                                 robot.setElevatorTarget(500),
                                 new SleepAction(0.25),
                                 new ParallelAction(
                                         robot.resetClassAction(),
                                         firstSample.build(),
                                         robot.setElevatorTarget(20)
                                 ),
                                 robot.intakeClawAction(),
                                 robot.outtakeClawAction(),
                                 new ParallelAction(
                                         robot.setElevatorTarget(3500),
                                         new SequentialAction(
                                                 new SleepAction(0.5),
                                                 dropFirstSample.build()

                                         )
                                 ),
                                 robot.resetClassAction(),
                                 new ParallelAction(
                                         robot.setElevatorTarget(20),
                                         pickSecondSample.build()
                                 ),
                                 robot.intakeClawAction(),
                                 robot.outtakeClawAction(),
                                 new ParallelAction(
                                         robot.setElevatorTarget(3500),
                                         new SequentialAction(
                                                 new SleepAction(0.5),
                                                 dropSecondSample.build()


                                         )
                                 ),
                                 robot.resetClassAction(),
                                 robot.setElevatorTarget(20),
                                 new ParallelAction(
                                         pickThirdSample.build(),
                                         robot.setHorizontalTarget(1000)
                                 ),
                                 robot.intakeClawAction(),
                                 robot.setHorizontalTarget(30),
                                 new SleepAction(0.25),
                                 robot.outtakeClawAction(),
                                 robot.setElevatorTarget(3500),
                                 new SleepAction(0.5),
                                 dropThirdSample.build(),
                                 robot.resetClassAction()
                         )

                 )
         );

    }
}
