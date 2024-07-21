package org.firstinspires.ftc.teamcode.subsystem.deposit;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.Subsystem;

@Config
public class NewDeposit implements Subsystem {
    public static Servo lArm, rArm, wrist, lclaw, rclaw, swivel;

    ElapsedTime timer = new ElapsedTime();


    public static double armTransfer = 0.57, armDeposit = 0.07, armPreTransfer = 0.45;
    public static double wrist30degree = 0.9, wristTransfer = 0.46;

    public static double lGrab = 0.55;
    public static double lDrop = 1;
    public static double rGrab = 0.45;
    public static double rDrop = 0.2;

    public static double swivelTransfer = 0.285;
    public static double swivelFlat = 0;
    public static double swivelVert = 0;

    public static double armHighDeposit = 0.17;
    public static double wrist90Degree = 1;

    public int swivelPos = 0;


    public NewDeposit(HardwareMap hardwareMap) {
        lArm = hardwareMap.servo.get("lArm");
        rArm = hardwareMap.servo.get("rArm");
        lArm.setDirection(Servo.Direction.REVERSE);

        wrist = hardwareMap.servo.get("wrist");

        lclaw = hardwareMap.servo.get("lClaw");
        rclaw = hardwareMap.servo.get("rClaw");
        swivel = hardwareMap.get(Servo.class, "swivel");
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
        if(swivel.getPosition() < 0.25 || swivel.getPosition() > 0.75)
            setClawL(lGrab);
        else setClawR(rGrab);
    }
    public void dropL(){
        if(swivel.getPosition() < 0.25 || swivel.getPosition() > 0.75)
            setClawL(lDrop);
        else setClawR(rDrop);
    }

    public void setClawL(double l){
        lclaw.setPosition(l);
    }

    public void grabR(){
        if(!(swivel.getPosition() < 0.25 || swivel.getPosition() > 0.75))
            setClawL(lGrab);
        else setClawR(rGrab);
    }
    public void dropR(){
        if(!(swivel.getPosition() < 0.25 || swivel.getPosition() > 0.75))
            setClawL(lDrop);
        else setClawR(rDrop);
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

    public void setSwivel(double pos){
        swivel.setPosition(pos);
    }

    public void setSwivel(){
        setSwivel(getSwivelValue(swivelPos));
    }

    public void setSwivel(int pos){
        setSwivel(getSwivelValue(pos));
    }

    public static double getSwivelValue(int pos){
        return swivelFlat + ((pos + 6 * 100) % 6) * Math.abs(swivelFlat - swivelVert) * 2 / 3;
    }

}
