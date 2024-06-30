package org.firstinspires.ftc.teamcode.subsystem.deposit;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class NewDeposit {
    private Servo lArm, rArm, wrist, lclaw, rclaw;

    ElapsedTime timer = new ElapsedTime();

    double currentPos;

    public static double armTransfer = 0.0, armDeposit = 0.0, armPreTransfer = 0.0;
    public static double wrist30degree = 0.0, wristTransfer = 0.0, wrist90degree = 0.0;
    public static double pixel = 0.0, noPixel = 0.0;

    public NewDeposit(HardwareMap hardwareMap) {
        lArm = hardwareMap.servo.get("lArm");
        rArm = hardwareMap.servo.get("rArm");
        lArm.setDirection(Servo.Direction.REVERSE);

        wrist = hardwareMap.servo.get("wrist");

        lclaw = hardwareMap.servo.get("lclaw");
        rclaw = hardwareMap.servo.get("rclaw");
    }

    public void setArm(double armpos) {
        lArm.setPosition(armpos); rArm.setPosition(armpos);
    }
}
