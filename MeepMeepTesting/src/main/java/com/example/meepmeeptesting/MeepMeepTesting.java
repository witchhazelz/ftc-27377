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

        Pose2d startPose = new Pose2d(startingPositionX, startingPositionY, Math.toRadians(270));

        myBot.runAction(myBot.getDrive().actionBuilder(startPose)
                // Move to basket facing it and wait
                .setTangent(Math.toRadians(180))
                .splineTo(new Vector2d(basketX, basketY), Math.toRadians(180))
                .waitSeconds(1.0)

                // Move to sample 1 facing forward (90 degrees) and wait
                .setTangent(Math.toRadians(90))
                .splineTo(new Vector2d(sample1X, collectSampleY), Math.toRadians(90))
                .waitSeconds(1.0)

                // Return to basket and wait
                .setTangent(Math.toRadians(180))
                .splineTo(new Vector2d(basketX, basketY), Math.toRadians(180))
                .waitSeconds(1.0)

                // Move to sample 2 facing forward (90 degrees) and wait
                .setTangent(Math.toRadians(90))
                .splineTo(new Vector2d(sample2X, collectSampleY), Math.toRadians(90))
                .waitSeconds(1.0)

                // Return to basket and wait
                .setTangent(Math.toRadians(180))
                .splineTo(new Vector2d(basketX, basketY), Math.toRadians(180))
                .waitSeconds(1.0)

                // Move to sample 3 facing forward (90 degrees) and wait
                .setTangent(Math.toRadians(90))
                .splineTo(new Vector2d(sample3X, collectSampleY), Math.toRadians(90))
                .waitSeconds(1.0)

                // Return to basket and wait
                .setTangent(Math.toRadians(270))
                .splineTo(new Vector2d(basketX, basketY), Math.toRadians(45))
                .waitSeconds(1.0)

                // Park in the final position with minimal turning
                .setTangent(Math.toRadians(90))
                .splineTo(new Vector2d(parkX, parkY), Math.toRadians(90))

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
