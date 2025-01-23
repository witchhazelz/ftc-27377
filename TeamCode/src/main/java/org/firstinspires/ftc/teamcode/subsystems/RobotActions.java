package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SleepAction;

public class RobotActions {
    public static Robot robot;

    public static Action intakeSample(){
        return new SequentialAction(
                new InstantAction(() -> robot.pitch.moveToPosition(6)),
                new ParallelAction(
                        new InstantAction(()-> robot.wrist.setDOWN(0.6)),
                        new InstantAction(()->robot.claw.setOpened())
                ),
                new SleepAction(0.3),
                new InstantAction(()->robot.claw.setClamped())
        );
    }

    public static Action extendo() {
        return new SequentialAction(
                new InstantAction(() -> robot.wrist.setFORWARD(0.4)),
                new ParallelAction(
                        new InstantAction(() -> robot.pitch.moveToPosition(13)),
                        new InstantAction(() -> robot.linear.moveToPosition(20, 20))
                )
        );
    }

    public static Action dropAndRetract() {
        return new SequentialAction(
                new InstantAction(() -> robot.wrist.setDOWN(0.6)),
                new InstantAction(() -> robot.claw.setOpened()),
                new SleepAction(0.2),
                new ParallelAction(
                        new InstantAction(() -> robot.claw.setClamped()),
                        new InstantAction(() -> robot.wrist.setFORWARD(0.4)),
                        new InstantAction(() -> robot.pitch.moveToPosition(15)),
                        new InstantAction(() -> robot.linear.moveToPosition(0, 0))
                )
        );
    }

    public static Action park() {
        return new SequentialAction(
                new InstantAction(() -> robot.pitch.moveToPosition(13)),
                new InstantAction(() -> robot.linear.moveToPosition(10, 10))
        );
    }


}
