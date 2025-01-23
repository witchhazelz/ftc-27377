package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
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

                .setTangent(0)
                .lineToX(sample1X)
                .setTangent(90)
                .lineToY(collectSampleY)
                .waitSeconds(0.4)

                .setTangent(270)
                .lineToX(basketX)
                .setTangent(90)
                .lineToY(basketY)
                .waitSeconds(1.0)

                .setTangent(0)
                .lineToX(sample2X)
                .setTangent(90)
                .lineToY(collectSampleY)
                .waitSeconds(0.4)

                .setTangent(270)
                .lineToX(basketX)
                .setTangent(90)
                .lineToY(basketY)
                .waitSeconds(1.0)

                .setTangent(0)
                .lineToX(sample3X)
                .setTangent(90)
                .lineToY(collectSampleY)
                .waitSeconds(0.4)

                .setTangent(270)
                .lineToX(basketX)
                .setTangent(90)
                .lineToY(basketY)
                .waitSeconds(1.0)

                .setTangent(45)
                .lineToX(parkX)
                .setTangent(90)
                .lineToY(parkY)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();


    }
}
