package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.BulkReader;

@Config
public final class Robot {

    public final MecanumDrive drivetrain;
    public final Claw claw;
    public final SprocketPitch sprocketPitch;
    public final Extension extension;

    private final BulkReader bulkReader;

    // Slow mode flag
    private boolean slowModeEnabled = false;

    public Robot(HardwareMap hardwareMap, Pose2d startPose) {
        drivetrain = new MecanumDrive(hardwareMap, startPose);
        bulkReader = new BulkReader(hardwareMap);
        claw = new Claw(hardwareMap);
        sprocketPitch= new SprocketPitch(hardwareMap);
        extension = new Extension(hardwareMap);
    }

    public void readSensors() {
        bulkReader.bulkRead();
        drivetrain.updatePoseEstimate();
    }

    public void run() {
        // Check for slow mode request
        if (requestingSlowMode()) {
            enableSlowMode();
        } else {
            disableSlowMode();
        }

        // Control the claw, pitch sprocket, and extension based on your logic
        claw.run(slowModeEnabled);
        sprocketPitch.run(slowModeEnabled);
        extension.run(slowModeEnabled);
    }

    public boolean requestingSlowMode() {
        // Example condition for slow mode:
        // You can replace this with your actual logic, such as checking a button press
        return claw.isGrabbing() || sprocketPitch.isAdjusting() || extension.isExtending();
    }

    private void enableSlowMode() {
        slowModeEnabled = true;
        drivetrain.setMaxSpeed(0.5); // Set max speed to 50% of normal speed
    }

    private void disableSlowMode() {
        slowModeEnabled = false;
        drivetrain.setMaxSpeed(1.0); // Reset max speed to 100%
    }

    public void printTelemetry() {
        drivetrain.printTelemetry();
        claw.printTelemetry();
        sprocketPitch.printTelemetry();
        extension.printTelemetry();
    }
}
