package org.firstinspires.ftc.teamcode.subsystem.intake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystem.Subsystem;

@Config
public class NewActiveIntake implements Subsystem {
    private final Servo linkage;
    private final DcMotor intake;
    private ColorSensor distanceSensors;

    public static double ground = 0.0, retracted = 0.0, stackHigh = 0.0, stackMid = 0.0;
    public static double on = 1.0, off = 0.0;


    public boolean onePixel = false;
    public boolean twoPixel = true;

    public NewActiveIntake(HardwareMap hardwareMap) {
        linkage = hardwareMap.servo.get("linkage");
        // linkage.setDirection(Servo.Direction.REVERSE); idk if u have to flip or not
        intake = hardwareMap.dcMotor.get("intake");
        // intake.setDirection(DcMotor.Direction.REVERSE); idk if u have to flip or not
        distanceSensors = hardwareMap.colorSensor.get("distanceSensors");
    }

    public void setLinkage(double pos) { linkage.setPosition(pos); }
    public void setIntake(double power) { intake.setPower(power); }

    public void transfer() {
        setLinkage(ground);
        setIntake(on);
    }

    public void ground(){
        setLinkage(ground);
    }

    public void mid(){
        setLinkage(stackMid);
    }

    public void high(){
        setLinkage(stackHigh);
    }

    public void detectPixels() {
        //distanceSensors
    }

    @Override
    public void update() {

    }

    @Override
    public void init() {

    }
}

