package org.firstinspires.ftc.teamcode.subsystem.deposit;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Deposit {

    private Servo lArm, rArm, wrist, finger;

    //Arm Positions
    public static double armTransfer = 0, armDeposit = 0;

    //Wrist Position
    public static double wristTranfer = 0, wristDeposit = 0;

    //Finger Position
    public static double bothPixels = 0, onePixel = 0, retracted = 0;

    //TODO: I really dont know how you want pixel amount selection to work, or how you want this controlled in tele

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
