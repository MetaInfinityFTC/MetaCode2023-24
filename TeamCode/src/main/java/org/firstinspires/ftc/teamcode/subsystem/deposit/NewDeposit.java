package org.firstinspires.ftc.teamcode.subsystem.deposit;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.Subsystem;

@Config
public class NewDeposit implements Subsystem {
    public static Servo lArm, rArm, wrist, lclaw, rclaw;

    ElapsedTime timer = new ElapsedTime();


    public static double armTransfer = 0.0, armDeposit = 0.0, armPreTransfer = 0.0;
    public static double wrist30degree = 0.0, wristTransfer = 0.0, wrist90degree = 0.0;
    public static double pixel = 0.0, noPixel = 0.0;

    public static double lGrab = 0.0;
    public static double lDrop = 0.0;
    public static double rGrab = 0.0;
    public static double rDrop = 0.0;

    public NewDeposit(HardwareMap hardwareMap) {
        lArm = hardwareMap.servo.get("lArm");
        rArm = hardwareMap.servo.get("rArm");
        lArm.setDirection(Servo.Direction.REVERSE);

        wrist = hardwareMap.servo.get("wrist");

        lclaw = hardwareMap.servo.get("lclaw");
        rclaw = hardwareMap.servo.get("rclaw");
    }

    @Override
    public void init() {
        setArm(armPreTransfer);
        setWrist(wristTransfer);
        clawDrop();
    }

    @Override
    public void update() {

    }

    public void outtake(){
        setArm(armDeposit);
        setWrist(wrist30degree);
        clawGrab();
    }

    public void transfer(){
        setArm(armTransfer);
        setWrist(wristTransfer);
    }

    public void setArm(double armpos) {
        lArm.setPosition(armpos); rArm.setPosition(armpos);
    }

    public void setWrist(double w){
        wrist.setPosition(w);
    }

    public void grabL(){
        setClawL(lGrab);
    }
    public void dropL(){
        setClawL(lDrop);
    }

    public void setClawL(double l){
        lclaw.setPosition(l);
    }

    public void grabR(){
        setClawR(rGrab);
    }
    public void dropR(){
        setClawR(rDrop);
    }

    public void setClawR(double r){
        rclaw.setPosition(r);
    }

    public void clawGrab(){
        grabL();
        grabR();
    }

    public void clawDrop(){
        dropL();
        dropR();
    }

    public void setClaw(double c){
        setClawL(c);
        setClawR(c);
    }

    public void collapse(){
        setArm(armPreTransfer);
        setWrist(wristTransfer);
        clawDrop();
    }
}
