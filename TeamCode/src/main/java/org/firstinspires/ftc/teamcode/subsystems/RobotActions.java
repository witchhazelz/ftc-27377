package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.Common.robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SleepAction;

public class RobotActions {

    public static Action intakeSample(){
        return new SequentialAction(
                setPitch(6, 0.3),
                new ParallelAction(
                        setWrist(0.6,0.2),
                        setClaw(false, 0.2)
                ),
                setClaw(true, 0)
        );
    }

    public static Action setUpBasket() {
        return new SequentialAction(
                setWrist(0.4, 0.2),
                new ParallelAction(
                        setPitch(20, 0.4),
                        setLinear(20, 0)
                )
        );
    }

    public static Action dropAndRetract() {
        return new SequentialAction(
                new ParallelAction(
                        setWrist(0.6, 0.2),
                        setClaw(false, 0.2)
                ),
                new SleepAction(0.3),
                new ParallelAction(
                        setClaw(false, 0.5),
                        setWrist(0.4, 0.5),
                        setPitch(6, 0.5),
                        setLinear(0, 0)
                )
        );
    }

    public static Action park() {
        return new SequentialAction(
                setPitch(13, 0.2),
                setLinear(10, 0)
        );
    }

    private static Action setClaw(boolean isClosed, double sleepSeconds) {
        return new ParallelAction(
                new InstantAction(() -> robot.claw.setClaw(isClosed)),
                new SleepAction(sleepSeconds)
        );
    }

    private static Action setWrist(double target, double sleepSeconds) {
        return new ParallelAction(
                new InstantAction(() -> robot.wrist.setPosition(target)),
                new SleepAction(sleepSeconds)
        );
    }

    private static Action setPitch(double target, double sleepSeconds) {
        return new ParallelAction(
                new InstantAction(() -> robot.pitch.moveToPosition(target)),
                new SleepAction(sleepSeconds)
        );
    }

    private static Action setLinear(double target, double sleepSeconds) {
        return new ParallelAction(
                new InstantAction(() -> robot.linear.moveToPosition(target, target)),
                new SleepAction(sleepSeconds)
        );
    }
}
