package org.firstinspires.ftc.teamcode.subsystem.deposit;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Slides {

    public enum Side {
        red, blue
    }

    Side side = Side.blue;

    public static double p = 0, i = 0, d = 0, f = 0;
    PIDFController controller;

    private DcMotor left, right;

    public Slides(HardwareMap hardwareMap, Side side) {
        left = hardwareMap.dcMotor.get("lSlide");
        right = hardwareMap.dcMotor.get("rSlide");
        left.setDirection(DcMotorSimple.Direction.REVERSE);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        controller = new PIDFController(p, i, d, f);
        this.side = side;
    }

    private double pidTarget = 0;

    public void updatePID() {
        double cmd = controller.calculate(left.getCurrentPosition(), pidTarget);
        setPower(cmd);
        //TODO: remove when done tuning
        controller.setPIDF(p, i, d, f);
    }

    public void manual() {
        //TODO: Vikram do this cause idk what yall want me to do
    }

    public void setPower(double power) {
        left.setPower(power);
        right.setPower(power);
    }

}
