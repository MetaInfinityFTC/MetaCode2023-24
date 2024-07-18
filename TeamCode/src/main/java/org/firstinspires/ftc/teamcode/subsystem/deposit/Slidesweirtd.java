package org.firstinspires.ftc.teamcode.subsystem.deposit;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystem.Subsystem;

@Config
public class Slidesweirtd implements Subsystem {

    public static double p = 0, i = 0, d = 0, f = 0;
    public static double tolerance = 3;
    public PIDFController controller;

    public static DcMotor left, right;

    public double upperLimit = 450, lowerLimit = 0;

    public int row = 1;

    public static double row0 = 0;
    public static double row1 = 0;
    public static double row2 = 0;
    public static double row3 = 0;
    public static double row4 = 0;
    public static double row5 = 0;
    public static double row6 = 0;
    public static double row7 = 0;
    public static double row8 = 0;
    public static double row9 = 0;
    public static double row10 = 0;

    public static double hangHeight = 200;

    public static int rows = 10;


    public Slidesweirtd(HardwareMap hardwareMap) {
        left = hardwareMap.dcMotor.get("lSlide");
        right = hardwareMap.dcMotor.get("rSlide");
        right.setDirection(DcMotorSimple.Direction.REVERSE);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        controller = new PIDFController(p, i, d, f);
        controller.setTolerance(tolerance);
    }

    public double pidTarget = 0;


    public void setPidTarget(double target) {
        //base encoder code
        pidTarget = Range.clip(target, lowerLimit, upperLimit);
    }



    @Override
    public void update() {
        if(getPos() < 0){
            left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        controller.setPIDF(p, i, d, f);
        controller.setTolerance(tolerance);
        double cmd = controller.calculate(getPos(), pidTarget);
        setPower(cmd);
    }

    public void toHangPosition(){
        setPidTarget(hangHeight);
    }

    @Override
    public void init() {
        setPosition(0);
    }

    //DO NOT USE THIS

//    public void manual(double inputPower) {
//        if ((left.getCurrentPosition() > lowerLimit && inputPower < 0) || (left.getCurrentPosition() < upperLimit && inputPower > 0)) {
//            setPidTarget(inputPower);
//        } else {
//            setPower(f);
//        }
//    }

    public void setPower(double power) {
        left.setPower(power);
        right.setPower(power);
    }

    public double getRowPosition(int row){
        if(row == 0) return row0;

        row = ((row + rows*100)%rows)+1;

        switch (row){
            case 1:
                return row1;
            case 2:
                return row2;
            case 3:
                return row3;
            case 4:
                return row4;
            case 5:
                return row5;
            case 6:
                return row6;
            case 7:
                return row7;
            case 8:
                return row8;
            case 9:
                return row9;
            case 10:
                return row10;
        }

        return row0;
    }

    public void setPosition(){
        setPosition(row);
    }

    public void setPosition(int row){
        setPidTarget(getRowPosition(row));
    }

    public double getPos() {
        return right.getCurrentPosition();
    }

    public boolean isAtTarget() {
        return controller.atSetPoint();
    }

}
