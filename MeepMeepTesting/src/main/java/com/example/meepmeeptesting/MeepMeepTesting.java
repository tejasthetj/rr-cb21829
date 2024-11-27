package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(100, 100, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-30, -60, 90))
                        .lineToLinearHeading(new Pose2d(0,-33.5,Math.toRadians(270)))
                        .waitSeconds(0.5)
                        //first specimen
                        .strafeLeft(20)
                        .splineToSplineHeading(new Pose2d(46,-13,Math.toRadians(270)),Math.toRadians(0))
                        .forward(40)
                        .waitSeconds(0.5)
                        .strafeRight(2)
                        .splineToLinearHeading(new Pose2d(55,-11.5,Math.toRadians(270)),Math.toRadians(0))
                        .forward(40)
                        .back(20)
                        .waitSeconds(1)

                        //cycle1
                        .lineToLinearHeading(new Pose2d(36,-56.5,Math.toRadians(0)))
                        .waitSeconds(1)
                        .lineToLinearHeading(new Pose2d(0,-33.5,Math.toRadians(270)))
                        .waitSeconds(1)

                        //cycle2
                        .lineToLinearHeading(new Pose2d(36,-56.5,Math.toRadians(0)))
                        .waitSeconds(1)
                        .lineToLinearHeading(new Pose2d(0,-33.5,Math.toRadians(270)))
                        .waitSeconds(1)

                        //cycle3
                        .lineToLinearHeading(new Pose2d(36,-56.5,Math.toRadians(0)))
                        .waitSeconds(1)
                        .lineToLinearHeading(new Pose2d(0,-33.5,Math.toRadians(270)))
                        .waitSeconds(1)

                        //cycle4
                        .lineToLinearHeading(new Pose2d(36,-56.5,Math.toRadians(0)))
                        .waitSeconds(1)
                        .lineToLinearHeading(new Pose2d(0,-33.5,Math.toRadians(270)))
                        .waitSeconds(1)


                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}