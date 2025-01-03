package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.BulkReader;

//@Config
public final class Robot {

    public final MecanumDrive drivetrain;
    public final Claw claw;
    public final Extension extension;
    public final Pitch pitch;

    public final BulkReader bulkReader;

    public Robot(HardwareMap hardwareMap, Pose2d startPose) {
        drivetrain = new MecanumDrive(hardwareMap, startPose);
        bulkReader = new BulkReader(hardwareMap);
        claw = new Claw(hardwareMap);
        extension = new Extension(hardwareMap);
        pitch = new Pitch(hardwareMap);
    }

    public void readSensors() {
        bulkReader.bulkRead();
        drivetrain.updatePoseEstimate();
    }

    public void run() {
        // read sensor data first
        readSensors();

        // control subsystems
        controlClaw();
        controlExtension();
        controlPitch();
        handleObstacles();

        // print telemetry for future debugging hehe
        printTelemetry();
    }

    private void controlClaw() {
        boolean isClawOpen = claw.isOpen();
        boolean isExtensionExtended = extension.isExtended();

        if (isClawOpen) {
            //if claw is open, check if extension needs to be retracted
            if (isExtensionExtended) {
                extension.retract();
            } else {
                // if extension is not extended, we can close claw (maybe?)
                claw.close();
            }
        } else {
            // if claw is closed, we may want to extend arm
            if (!isExtensionExtended) {
                extension.extend();
            }
        }

        // run claw subsystem
        claw.run();
    }

    private void controlExtension() {
        // additional logic for controlling the extension can be added here
        // but for now, we just run the extension subsystem, the current logic is pretty decent
        extension.run();
    }

    private void controlPitch() {
        if (!pitch.isAtTargetPosition()) {
            pitch.moveToPosition();
        }

        // run pitch subsystem
        pitch.run();
    }

    private void handleObstacles() {
        if (bulkReader.isObstacleDetected()) {
            // if obstacle is detected, stop all movements
            drivetrain.stop();
            claw.stop();
            extension.stop();
            pitch.stop();
        } else {
            // if no obstacles, continue driving
            drivetrain.drive();
        }
    }

    public void printTelemetry() {
        drivetrain.printTelemetry();
        claw.printTelemetry();
        extension.printTelemetry();
        pitch.printTelemetry();
    }
}