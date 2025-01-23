package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static double
            startingPositionX = -14.375,
            startingPositionY = -62,
            basketX = -55,
            basketY = -58,
            collectSampleY = -30,
            sample1X = -40,
            sample2X = -50,
            sample3X = -60,
            parkX = -20,
            parkY = -10;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(50, 60, Math.toRadians(180), Math.toRadians(180), 11.75)
                .setDimensions(14, 16.5)
                .build();

        Pose2d startPose = new Pose2d(startingPositionX, startingPositionY, Math.toRadians(180));

        myBot.runAction(myBot.getDrive().actionBuilder(startPose)
                .setTangent(270)
                .splineToLinearHeading(new Pose2d(basketX, basketY, Math.toRadians(225)), Math.toRadians(180))
                .waitSeconds(1.0)

                .setTangent(0)
                .splineToLinearHeading(new Pose2d(sample1X, collectSampleY, Math.toRadians(135)), Math.PI)
                .waitSeconds(0.4)

                .setTangent(270)
                .splineToLinearHeading(new Pose2d(basketX, basketY, Math.toRadians(225)), Math.toRadians(180))
                .waitSeconds(1.0)

                .setTangent(0)
                .splineToLinearHeading(new Pose2d(sample2X, collectSampleY, Math.toRadians(135)), Math.PI)
                .waitSeconds(0.4)

                .setTangent(270)
                .splineToLinearHeading(new Pose2d(basketX, basketY, Math.toRadians(225)), Math.toRadians(180))
                .waitSeconds(1.0)

                .setTangent(0)
                .splineToLinearHeading(new Pose2d(sample3X, collectSampleY, Math.toRadians(135)), Math.PI)
                .waitSeconds(0.4)

                .setTangent(270)
                .splineToLinearHeading(new Pose2d(basketX, basketY, Math.toRadians(225)), Math.toRadians(180))
                .waitSeconds(1.0)
                        .setTangent(45)
                .splineToLinearHeading(new Pose2d(parkX, parkY, Math.toRadians(0)), Math.toRadians(60))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
