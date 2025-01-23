package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.subsystems.Common.robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.RobotActions;

@Autonomous(name = "Sample Side")
@Config()
public class BasketAuto extends AbstractAuto {
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

    @Override
    protected Pose2d getStartPose() {
        return new Pose2d(startingPositionX, startingPositionY, Math.toRadians(180));
    }

    @Override
    protected Action onRun() {
        TrajectoryActionBuilder builder = robot.drivetrain.actionBuilder(getStartPose());
        builder = scoreBasket(builder);
        builder = intakeFromSampleMark(builder, sample1X);

        builder = scoreBasket(builder);
        builder = intakeFromSampleMark(builder, sample2X);

        builder = scoreBasket(builder);
        builder = intakeFromSampleMark(builder, sample3X);

        builder = park(builder);

        return builder.build();
    }

    private TrajectoryActionBuilder scoreBasket(TrajectoryActionBuilder builder) {
        builder = builder
                .setTangent(270)
                .afterTime(0, RobotActions.setUpBasket())
                .splineToLinearHeading(new Pose2d(basketX, basketY, Math.toRadians(225)), Math.toRadians(180))
                .stopAndAdd(RobotActions.dropAndRetract());

        return builder;
    }

    private TrajectoryActionBuilder intakeFromSampleMark(TrajectoryActionBuilder builder, double x) {
        builder = builder
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(x, collectSampleY, Math.toRadians(135)), Math.PI)
                .afterTime(0, RobotActions.intakeSample())
                .waitSeconds(0.4);

        return builder;
    }

    private TrajectoryActionBuilder park(TrajectoryActionBuilder builder) {
        builder = builder
                .setTangent(45)
                .afterTime(0, RobotActions.park())
                .splineToLinearHeading(new Pose2d(parkX, parkY, Math.toRadians(0)), Math.toRadians(60));

        return builder;
    }
}
