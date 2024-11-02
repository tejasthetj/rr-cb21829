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

        Pose2d initialPose = new Pose2d(-30, -63, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder myTraj = drive.actionBuilder(initialPose)
                .setReversed(false)
                .splineTo(new Vector2d(-40, -26), Math.PI)
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-55, -55), Math.toRadians(225))
                .strafeToLinearHeading(new Vector2d(-58, -34), Math.PI / 2)
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-55, -55), Math.toRadians(225))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-60, -25, Math.toRadians(180)), Math.toRadians(180))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-55, -55), Math.toRadians(225));

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        myTraj.build()
                )
        );

    }
}
