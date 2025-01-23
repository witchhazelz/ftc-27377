package org.firstinspires.ftc.teamcode.subsystems;

//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.BulkReader;


//@Config
public final class Robot {

    //public final MecanumDrive drivetrain;
    public final MecanumDrive drivetrain;
    public final Claw claw;
    public final Linear linear;
    public final Pitch pitch;
    public final Wrist wrist;

   // Wrist wrist = new Wrist();

    public final BulkReader bulkReader;

    public Robot(HardwareMap hardwareMap) {
        //drivetrain = new MecanumDrive(hardwareMap, startPose);
        wrist =  new Wrist();
        drivetrain = new MecanumDrive(hardwareMap,new Pose2d(0,0,0));
        bulkReader = new BulkReader(hardwareMap);
        claw = new Claw(hardwareMap);
        linear = new Linear();
        pitch = new Pitch(hardwareMap);

        linear.LinearSlides(hardwareMap);
    }

   public void readSensors() {
        bulkReader.bulkRead();
        drivetrain.updatePoseEstimate();
    }

    public void run() {
        // read sensor data first
        //readSensors();

        wrist.run();
        claw.run();
        linear.run();
        pitch.run();
        // print telemetry for future debugging hehe
        printTelemetry();
    }



    public void printTelemetry() {
        drivetrain.printTelemetry();
        claw.printTelemetry();
        linear.printTelemetry();
        pitch.printTelemetry();
    }
}