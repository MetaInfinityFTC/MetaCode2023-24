package org.firstinspires.ftc.teamcode.subsystem.deposit;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Slides {

    public static double p = 0.045, i = 0, d = 0, f = 0.001;
    PIDFController controller;

    private DcMotor left, right;

    public double upperLimit = 0, lowerLimit = 0;

    public Slides(HardwareMap hardwareMap) {
        left = hardwareMap.dcMotor.get("lSlide");
        right = hardwareMap.dcMotor.get("rSlide");
        left.setDirection(DcMotorSimple.Direction.REVERSE);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        controller = new PIDFController(p, i, d, f);
    }

    private double pidTarget = 0;

    public void setPidTarget(double pidTarget) {
        this.pidTarget = pidTarget;
    }

    public void updatePID() {
        double cmd = controller.calculate(left.getCurrentPosition(), pidTarget);
        setPower(cmd);
        //TODO: remove when done tuning
    }

    public void manual(double inputPower) {
        if ((left.getCurrentPosition() > lowerLimit && inputPower < 0) || (left.getCurrentPosition() < upperLimit && inputPower > 0)) {
            setPidTarget(inputPower);
        } else {
            setPower(f);
        }
    }

    public void setPower(double power) {
        left.setPower(power);
        right.setPower(power);
    }
    public double getPos() {
        return right.getCurrentPosition();
    }

}
