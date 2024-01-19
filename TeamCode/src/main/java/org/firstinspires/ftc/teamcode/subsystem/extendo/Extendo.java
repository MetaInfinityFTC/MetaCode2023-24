package org.firstinspires.ftc.teamcode.subsystem.extendo;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Extendo {

    private DcMotor left, right;
    public static double p = 0, i = 0, d = 0, f = 0;
    PIDFController controller = new PIDFController(p, i, d, f);

    //TODO: tune extension vals
    //enum to house states with better names
    public enum Extension_States {
        retracted(0.0), first(0.0), second(0.0), extended(0.0);
        private double numVal;
        Extension_States(double numVal) { this.numVal = numVal;}
        public double getNumVal() { return numVal; }
    }

    Extension_States state = Extension_States.retracted;

    //TODO: Set motor names to whatever they are in the config, maybe swap direction
    public Extendo(HardwareMap hardwareMap) {
        left = hardwareMap.dcMotor.get("leftEx"); right = hardwareMap.dcMotor.get("rightEx");
        left.setDirection(DcMotorSimple.Direction.REVERSE);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void update() {
        double output = controller.calculate(right.getCurrentPosition(), state.getNumVal());
        left.setPower(output); right.setPower(output);
        //TODO: Remove controller reset when tuning is done
        controller.setPIDF(p, i, d, f);
    }

    public void setState(Extension_States state) {
        this.state = state;
    }

    public double getPos() {
        return right.getCurrentPosition();
    }
}
