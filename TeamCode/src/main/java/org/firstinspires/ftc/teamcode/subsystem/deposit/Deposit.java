package org.firstinspires.ftc.teamcode.subsystem.deposit;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Deposit {

    private Servo lArm, rArm, wrist, finger;

    ElapsedTime timer = new ElapsedTime();

    static final double INCREMENT = 0.005 ;     // amount to slew servo each CYCLE_MS cycle
    double currentPos;

    boolean amithere = false;
    //Arm armposs
    public static double armTransfer = 0.95, armDeposit90 = 0.45, armDeposit30 = 0.6, armPreTransfer = 0.75;

    //Wrist armpos
    public static double wrist30degree = 0.85, wristTransfer = 0.187, wrist90degree = 0.7;

    //Finger armpos
    public static double bothPixels = 0.02, onePixel = 0.25, zeroPixel = 0.35;

    public Deposit(HardwareMap hardwareMap) {
        lArm = hardwareMap.servo.get("lArm");
        rArm = hardwareMap.servo.get("rArm");
        lArm.setDirection(Servo.Direction.REVERSE);

        wrist = hardwareMap.servo.get("wrist");
        finger = hardwareMap.servo.get("finger");
    }

    public void setArm(double armpos) {
        lArm.setPosition(armpos); rArm.setPosition(armpos);
    }

    public void setSlowArm(double armpos){
        amithere = false;
        currentPos = lArm.getPosition();

        if (currentPos < armpos) {
            // Keep stepping up until we hit the max value.
            currentPos += INCREMENT ;
        }
        else if (currentPos > armpos){
            // Keep stepping down until we hit the min value.
            currentPos -= INCREMENT ;
        }
        else {
            currentPos = currentPos;
            amithere = true;
        }

        // Set the servo to the new armpos and pause;
        lArm.setPosition(currentPos);
        rArm.setPosition(currentPos);
    }

    public void setWrist(double wristpos) {
        wrist.setPosition(wristpos);
    }

    public void setFinger(double fingerpos) {
        finger.setPosition(fingerpos);
    }

    public boolean isAtTarget() {
        return amithere;
    }
}
