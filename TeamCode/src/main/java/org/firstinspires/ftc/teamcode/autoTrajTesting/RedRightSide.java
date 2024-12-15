package org.firstinspires.ftc.teamcode.autoTrajTesting;

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

@Autonomous(name = "red right traj testing", group = "Autonomous")
public class RedRightSide extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initStartPos = new Pose2d(10, -63, Math.toRadians(270));



        MecanumDrive drive = new MecanumDrive(hardwareMap, initStartPos);
        AllMechForRR robot = new AllMechForRR(hardwareMap);


//        TrajectoryActionBuilder dropPreLoaded = drive.actionBuilder(initStartPos)
//                        .strafeTo(new Vector2d(0, -35));
//
//        TrajectoryActionBuilder pickFirstSample = drive.actionBuilder(new Pose2d(0, -30, Math.toRadians(270)))
//                        .setReversed(false)
//                        .splineToLinearHeading(new Pose2d(48, -40, Math.toRadians(90)), Math.PI / 4);
//
//        TrajectoryActionBuilder dropFirstSample = drive.actionBuilder(new Pose2d(48, -38, Math.toRadians(90)))
//                        .strafeToLinearHeading(new Vector2d(58, -55), Math.toRadians(90));
//
//        TrajectoryActionBuilder pickSecondSample = drive.actionBuilder(new Pose2d(58, -55, Math.PI/2))
//                        .strafeToConstantHeading(new Vector2d(58, -40));
//
//        TrajectoryActionBuilder dropSecondSample = drive.actionBuilder(new Pose2d(58, -40, Math.PI/2))
//                        .strafeToConstantHeading(new Vector2d(58, -55));
//
//        TrajectoryActionBuilder pickThirdSample = drive.actionBuilder(new Pose2d(58, -55, Math.PI/2))
//                .setReversed(true)
//                .splineToSplineHeading(new Pose2d(50, -30, Math.toRadians(0)), Math.PI/2)
//                .splineToLinearHeading(new Pose2d(63, -10, Math.toRadians(-90)), Math.PI/6);
//
//        TrajectoryActionBuilder dropThirdSample = drive.actionBuilder(new Pose2d(63, -10, Math.toRadians(-90)))
//                .strafeToConstantHeading(new Vector2d(62, -55));
//
//        TrajectoryActionBuilder waitPatiently = drive.actionBuilder(new Pose2d(62, -55, Math.toRadians(-90)))
//                .strafeToLinearHeading(new Vector2d(25, -58), Math.toRadians(0))
//                .waitSeconds(2);
//
//        TrajectoryActionBuilder pickSpecimen = drive.actionBuilder(new Pose2d(25, -58, Math.toRadians(0)))
//                .strafeToLinearHeading(new Vector2d(40, -58), Math.toRadians(0));
//
//        TrajectoryActionBuilder dropSpecimen = drive.actionBuilder(new Pose2d(40, -58, Math.toRadians(0)))
//                .strafeToLinearHeading(new Vector2d(0, -34), Math.toRadians(-90));

        TrajectoryActionBuilder dropPreLoaded = drive.actionBuilder(initStartPos)
                .strafeTo(new Vector2d(0, -36));

        TrajectoryActionBuilder pickFirstSample = drive.actionBuilder(new Pose2d(0, -34, Math.toRadians(270)))
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(48, -40, Math.toRadians(90)), Math.PI / 4);

        TrajectoryActionBuilder dropFirstSample = drive.actionBuilder(new Pose2d(48, -40, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(48, -55), Math.toRadians(90));

        TrajectoryActionBuilder pickSecondSample = drive.actionBuilder(new Pose2d(48, -55, Math.PI/2))
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(40, -10, Math.toRadians(0)), Math.PI/2)
                .strafeToLinearHeading(new Vector2d(58, -10), Math.toRadians(270));

        TrajectoryActionBuilder dropSecondSample = drive.actionBuilder(new Pose2d(58, -10, Math.toRadians(270)))
                .strafeToConstantHeading(new Vector2d(58, -58));


        TrajectoryActionBuilder waitPatiently = drive.actionBuilder(new Pose2d(0, -36, Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(25, -60), Math.toRadians(0))
                .waitSeconds(2);

        TrajectoryActionBuilder pickSpecimen1 = drive.actionBuilder(new Pose2d(25, -60, Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(34, -62), Math.toRadians(0));

        TrajectoryActionBuilder dropSpecimen = drive.actionBuilder(new Pose2d(34, -62, Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(5, -35), Math.toRadians(270));

        TrajectoryActionBuilder pickSpecimen = drive.actionBuilder(new Pose2d(5, -35, Math.toRadians(270)))
                .strafeToLinearHeading(new Vector2d(34, -62), Math.toRadians(0));

        TrajectoryActionBuilder dropSpecimen1 = drive.actionBuilder(new Pose2d(34, -62, Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(-5, -35), Math.toRadians(270));


        waitForStart();
        robot.resetElevators();

        Actions.runBlocking(
                new ParallelAction(
                        robot.updatePID(),
                        new SequentialAction(
                                new ParallelAction(

                                        robot.setElevatorTarget(1300),
                                        new SequentialAction(
                                                dropPreLoaded.build(),
                                                robot.specimenOuttakeClawActionReset()
                                        )

                                ),
                                robot.setElevatorTarget(20),
                                new SleepAction(0.25),
                                new ParallelAction(
                                        robot.resetClassAction(),
                                        waitPatiently.build()
                                ),
                                pickSpecimen1.build(),
                                robot.intakeClawAction(),
                                new ParallelAction(
                                        robot.specimenOuttakeClawActionReset(),
                                        dropSpecimen.build(),
                                        robot.setElevatorTarget(1300)
                                ),
                                robot.setElevatorTarget(20),
                                new SleepAction(0.25),
                                new ParallelAction(
                                        robot.resetClassAction(),
                                        pickSpecimen.build()
                                ),
                                robot.intakeClawAction(),
                                new ParallelAction(
                                        robot.specimenOuttakeClawActionReset(),
                                        dropSpecimen1.build(),
                                        robot.setElevatorTarget(1300)
                                ),
                                robot.setElevatorTarget(20),
                                new SleepAction(0.25),
                                new ParallelAction(
                                      robot.resetClassAction(),
                                        pickSpecimen.build()
                                ),
                                new SleepAction(0.25)




//                                // first specimen
//                                pickSpecimen.build(),
//                                robot.intakeClawAction(),
//                                robot.specimenOuttakeClawAction(),
//                                robot.setElevatorTarget(1500),
//                                dropSpecimen.build(),
//                                robot.setElevatorTarget(1000),
//                                robot.resetClassAction(),
//                                robot.setElevatorTarget(20),
//                                // second specimen
//                                pickSpecimen.build(),
//                                robot.intakeClawAction(),
//                                robot.specimenOuttakeClawAction(),
//                                robot.setElevatorTarget(1500),
//                                dropSpecimen.build(),
//                                robot.setElevatorTarget(1000),
//                                robot.resetClassAction(),
//                                robot.setElevatorTarget(20),
//                                // third specimen
//                                robot.intakeClawAction(),
//                                robot.specimenOuttakeClawAction(),
//                                robot.setElevatorTarget(1500),
//                                dropSpecimen.build(),
//                                robot.setElevatorTarget(1000),
//                                robot.resetClassAction(),
//                                robot.setElevatorTarget(20)

                        )
                )
        );
    }
}
