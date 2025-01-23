package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;

public final class Common {
    public static Pose2d AUTO_END_POSE = null;

    public static final double
            LEFT = Math.toRadians(180),
            FORWARD = Math.toRadians(90),
            RIGHT = Math.toRadians(0),
            BACKWARD = Math.toRadians(270);

    public static final double MAX_VOLTAGE = 13;

    public static Robot robot;
    public static MultipleTelemetry mTelemetry;

}