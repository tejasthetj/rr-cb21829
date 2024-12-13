package org.firstinspires.ftc.teamcode.Auto;

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
@Autonomous(name = "red specimen auto", group = "Autonomous")
public class AutoRedRight extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        //Pose2d initialPose = new Pose2d(-30, -63, Math.toRadians(90));
        Pose2d redCloseStartingPose = new Pose2d(10, -63, Math.toRadians(270));
        Pose2d initialPosePush = new Pose2d(0,-36.5,Math.toRadians(270));
        Pose2d initialPoseSampleScorePush = new Pose2d(55,-33,Math.toRadians(0));
        Pose2d initialPoseSampleScore = new Pose2d(0, -36.5, Math.toRadians(270));
        Pose2d initialPoseSampleGet = new Pose2d(36, -56.5, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, redCloseStartingPose);
        AllMechForRR robot = new AllMechForRR(hardwareMap);


        TrajectoryActionBuilder firstSample = drive.actionBuilder(redCloseStartingPose)

                .strafeToLinearHeading(new Vector2d(0,-36.5),Math.toRadians(270));
                //first specimen

        TrajectoryActionBuilder Push = drive.actionBuilder(initialPosePush)
                .strafeTo(new Vector2d(20,-40))
                .splineToSplineHeading(new Pose2d(46,-13,Math.toRadians(270)),Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(46,-53),Math.toRadians(270))
//                .strafeTo(new Vector2d(44,-13))
//                .splineToLinearHeading(new Pose2d(55,-11.5,Math.toRadians(270)),Math.toRadians(0))
//                .strafeToLinearHeading(new Vector2d(55,-53),Math.toRadians(270))
                .strafeToLinearHeading(new Vector2d(55,-33),Math.toRadians(0));


        TrajectoryActionBuilder SampleGet = drive.actionBuilder(initialPoseSampleScore)

                //cycle1
                .strafeToLinearHeading(new Vector2d(36,-56.5),Math.toRadians(0));

        TrajectoryActionBuilder SampleGet1 = drive.actionBuilder(initialPoseSampleScorePush)

                //cycle1
                .strafeToLinearHeading(new Vector2d(36,-56.5),Math.toRadians(0));


        TrajectoryActionBuilder SampleScore = drive.actionBuilder(initialPoseSampleGet)
                .strafeTo(new Vector2d(0,-60))
                .splineToLinearHeading(new Pose2d(0,-36.5,Math.toRadians(270)),Math.toRadians(270));








        waitForStart();


        Actions.runBlocking(
                new ParallelAction(
                        robot.updatePID(),
                        new SequentialAction(
                                new ParallelAction(
                                        robot.setElevatorTarget(1500),
                                        new SequentialAction(
                                        robot.outtakeClawActionSpecimen()
                                                ),
                                        firstSample.build()

                                ),

                                robot.setElevatorTarget(40),
                                new SleepAction(0.5),
                                robot.resetClassAction(),
                                new ParallelAction(
                                        new SequentialAction(
                                       robot.intakeClawAction(),
                                                robot.setElevatorTarget(50)
                                                ),


                                        Push.build()

                                ),
                               new ParallelAction(
                                       new SequentialAction(
                                       robot.resetClassAction()
                                               ),
                                       SampleGet1.build()
                               ),


                                robot.intakeClawAction(),
                                new ParallelAction(
                                        new SequentialAction(
                                        robot.outtakeClawActionSpecimen(),
                                        robot.setElevatorTarget(1500)
                                                ),
                                        SampleScore.build()
                                ),
                                robot.setElevatorTarget(50),
                                new SleepAction(0.5),
                                robot.resetClassAction(),

                               new ParallelAction(
                                SampleGet.build(),
                                robot.setElevatorTarget(40)

                               ),
                                robot.intakeClawAction(),

                                new ParallelAction(
                                        new SequentialAction(
                                                robot.outtakeClawActionSpecimen(),
                                                robot.setElevatorTarget(1500)
                                        ),
                                SampleScore.build()
                                        ),

                               new ParallelAction(
                                SampleGet.build(),
                                robot.setElevatorTarget(40)
                               ),
                                robot.intakeClawAction(),
                                new ParallelAction(
                                        new SequentialAction(
                                                robot.outtakeClawActionSpecimen(),
                                                robot.setElevatorTarget(1500)
                                        ),

                                        SampleScore.build()

                                ),
                                robot.setElevatorTarget(50),
                                new SleepAction(0.5),
                                robot.setElevatorTarget(50),
                                SampleGet.build()



                        )

                )
        );

    }
}