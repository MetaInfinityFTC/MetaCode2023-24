package org.firstinspires.ftc.teamcode.subsystem.intake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystem.Subsystem;

@Config
public class NewActiveIntake implements Subsystem {
    private final Servo linkage;
    private final DcMotor intake;
    private RevColorSensorV3 pxSensor1, pxSensor2;

    public static double ground = 0.0, retracted = 0.0, stackHigh = 0.0, stackMid = 0.0;
    public static double on = 1.0, off = 0.0;


    public boolean onePixel = false;
    public boolean twoPixel = false;

    public double pxThresh = 0.0;

    public NewActiveIntake(HardwareMap hardwareMap) {
        linkage = hardwareMap.servo.get("linkage");
        // linkage.setDirection(Servo.Direction.REVERSE); idk if u have to flip or not
        intake = hardwareMap.dcMotor.get("intake");
        // intake.setDirection(DcMotor.Direction.REVERSE); idk if u have to flip or not
        pxSensor1 = hardwareMap.get(RevColorSensorV3.class, "one");
        pxSensor2 = hardwareMap.get(RevColorSensorV3.class, "two");
    }

    public void setLinkage(double pos) { linkage.setPosition(pos); }
    public void setIntake(double power) { intake.setPower(power); }

    public void transfer() {
        setLinkage(retracted);
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
        if (t(get1st()) && t(get2nd())) {
            twoPixel = true;
            onePixel = true;
        } else if (t(get1st()) || t(get2nd())) {
            onePixel = true;
            twoPixel = false;
        } else {
            onePixel = false;
            twoPixel = false;
        }
    }

    @Override
    public void update() {
        detectPixels();
    }

    @Override
    public void init() {

    }

    public double get1st() {
        return pxSensor1.getDistance(DistanceUnit.CM);
    }

    public double get2nd() {
        return pxSensor2.getDistance(DistanceUnit.CM);
    }

    public boolean t (double d) {
        if (d > pxThresh) {
            return true;
        } else {
            return false;
        }
    }
}

