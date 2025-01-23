package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.subsystems.Common.robot;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Common;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Disabled
public abstract class AbstractAuto extends LinearOpMode {

    protected final void update() {
        robot.readSensors();
        robot.run();
    }

    @Override
    public final void runOpMode() {
        robot = new Robot(hardwareMap);

        onInit();
        configure();
        Action action = onRun();

        if (isStopRequested()) return;

        waitForStart();

        resetRuntime();
        robot.drivetrain.pose = getStartPose();

        Actions.runBlocking(
                new ParallelAction(
                        action,
                        new org.firstinspires.ftc.teamcode.roadrunner.Actions.RunnableAction(() -> {
                            update();
                            return opModeIsActive();
                        })
                )
        );

        Common.AUTO_END_POSE = robot.drivetrain.pose;
    }

    protected void onInit() {}

    protected void configure() {}

    protected abstract Pose2d getStartPose();

    protected abstract Action onRun();
}