package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(80, 80, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, -65, Math.toRadians(90)))
//                .setReversed(false)
//                .splineTo(new Vector2d(-48.0, -16), -Math.PI / 2)
//                .waitSeconds(1)
//                .strafeToLinearHeading(new Vector2d(-55, -55), Math.toRadians(225))
//                .build());

//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, -65, Math.toRadians(90)))
//                .setReversed(false)
//                .splineTo(new Vector2d(-48.0, -34), Math.PI / 2)
//                .waitSeconds(1)
//                .strafeToLinearHeading(new Vector2d(-55, -55), Math.toRadians(225))
//                .waitSeconds(3)
//                .strafeToLinearHeading(new Vector2d(-58, -34), Math.PI / 2)
//                .waitSeconds(1)
//                .strafeToLinearHeading(new Vector2d(-55, -55), Math.toRadians(225))
//                .waitSeconds(3)
//                .strafeToLinearHeading(new Vector2d(-60, -25), Math.toRadians(180))
//                .waitSeconds(1)
//                .strafeToLinearHeading(new Vector2d(-55, -55), Math.toRadians(225))
//                .build());

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-30, -63, Math.toRadians(90)))
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
                .strafeToLinearHeading(new Vector2d(-55, -55), Math.toRadians(225))
                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}