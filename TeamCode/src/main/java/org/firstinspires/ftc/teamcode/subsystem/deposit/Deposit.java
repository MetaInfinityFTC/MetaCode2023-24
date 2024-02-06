package org.firstinspires.ftc.teamcode.subsystem.deposit;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Deposit {

    private Servo lArm, rArm, wrist, finger;

    //Arm Positions
    public static double armTransfer = 0.88, armDeposit90 = 0.60, armDeposit30 = 0.67, armPreTransfer = 0.75;

    //Wrist Position
    public static double wrist30degree = 0.79, wristTransfer = 0.18, wrist90degree = 0.65;

    //Finger Position
    public static double bothPixels = 0.5, onePixel = 0.5, zeroPixel = 0;

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
