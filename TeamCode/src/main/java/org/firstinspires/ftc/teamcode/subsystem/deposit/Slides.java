package org.firstinspires.ftc.teamcode.subsystem.deposit;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Slides {

    public static double p = 0.03, i = 0, d = 0, f = 0.01;
    public static double tolerance = 3;
    PIDFController controller;

    private DcMotor left, right;

    public double upperLimit = 450, lowerLimit = 0;

    public Slides(HardwareMap hardwareMap) {
        left = hardwareMap.dcMotor.get("lSlide");
        right = hardwareMap.dcMotor.get("rSlide");
        left.setDirection(DcMotorSimple.Direction.REVERSE);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        controller = new PIDFController(p, i, d, f);
        controller.setTolerance(tolerance);
    }

    public static double pidTarget = 0;


    public void setPidTarget(double target) {
        //base encoder code
        pidTarget = target;
    }

    public void updatePID() {
        controller.setPIDF(p, i, d, f);
        controller.setTolerance(tolerance);
        double cmd = controller.calculate(left.getCurrentPosition(), pidTarget);
        setPower(cmd);
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
    public double getPos() {
        return right.getCurrentPosition();
    }

    public boolean isAtTarget() {
        return controller.atSetPoint();
    }

}
