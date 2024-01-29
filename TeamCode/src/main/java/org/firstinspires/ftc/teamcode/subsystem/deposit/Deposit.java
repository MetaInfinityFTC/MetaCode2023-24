package org.firstinspires.ftc.teamcode.subsystem.deposit;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Deposit {

    private Servo lArm, rArm, wrist, finger;

    //Arm Positions
    public static double armTransfer = 0.9, armDeposit90 = 0.22, armDeposit30 = 0.42, armPreTransfer = 0.5;

    //Wrist Position
    public static double wrist30degree = 0.82, wristTransfer = 0.12, wrist90degree = 0.5;

    //Finger Position
    public static double bothPixels = 0, onePixel = 0.5, zeroPixel = 0.59;

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

    public void setWrist(double wristpos) {
        wrist.setPosition(wristpos);
    }

    public void setFinger(double fingerpos) {
        finger.setPosition(fingerpos);
    }
}
