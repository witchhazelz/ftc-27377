package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.util.SimpleServoPivot;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {

    public static final double DEPOSIT_ANGLE = 0.5;// angle_intial
    public static final double CLAMP_ANGLE = 0.0;//angle_activated

    //private final SimpleServoPivot claw;
    private final Servo claw;
    private boolean isClosed;

//    public Claw(HardwareMap hardwareMap) {
//        claw = new SimpleServoPivot(DEPOSIT_ANGLE, CLAMP_ANGLE, SimpleServoPivot.getGoBildaServo(hardwareMap, "claw"));
//        setClamped(false);
//    }

    public Claw(HardwareMap hardwareMap){
        claw = hardwareMap.get(Servo.class,"clawServo");
    }
//    public void toggleClaw() {
//        claw.toggle();
//        claw.run();
//    } //put through robot class
//
//    public void setClamped(boolean clamped) {
//        claw.setActivated(clamped);
//        claw.run();
//    }

    public void setClawOpen(){
        claw.setPosition(DEPOSIT_ANGLE);
    }

    public void setClawClosed(){
        claw.setPosition(CLAMP_ANGLE);
    }

    public void setClamped() {
        isClosed = true;
    }

    public void setOpened(){
        isClosed = false;
    }

    public void setClaw(boolean isActivated) {
        isClosed = isActivated;
    }

    public void run(){
        claw.setPosition(isClosed ? CLAMP_ANGLE : DEPOSIT_ANGLE);

    }
//
//    public boolean getClamped() {
//        return claw.isActivated();
//    }
//
//    public void run() {
//        claw.run();
//    }
//
//    public double getCurrentAngle() {
//        return claw.isActivated() ? DEPOSIT_ANGLE : CLAMP_ANGLE;
//    }
//
//    public boolean isActivated() {
//        return claw.isActivated();
//    }

    public void printTelemetry(){

    }

//    public void stop() {
//        claw.setActivated(true);
//    }
//    public void close()
//    {
//        claw.setActivated(true);
//    }
//    public boolean isOpen()
//    {
//        return !(claw.isActivated());
//    }
}
