package org.firstinspires.ftc.teamcode.subsystem.extendo;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Extendo {

    private DcMotor left, right;
    public static double p = 0.02, i = 0, d = 0, f = 0.0005;
    PIDFController controller = new PIDFController(p, i, d, f);

    //enum to house states with better names
    public enum Extension_States {
        retracted(0), first(500), second(800), extended(1050), closespike(200), midspike(400), farspike(900);
        private double numVal;
        Extension_States(double numVal) { this.numVal = numVal;}
        public double getNumVal() { return numVal; }
    }

    Extension_States state = Extension_States.retracted;

    public Extendo(HardwareMap hardwareMap) {
        left = hardwareMap.dcMotor.get("leftEx"); right = hardwareMap.dcMotor.get("rightEx");
        left.setDirection(DcMotorSimple.Direction.REVERSE);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        controller.setTolerance(3);
    }

    public void update() {
        double output = controller.calculate(right.getCurrentPosition(), state.getNumVal());
        left.setPower(output); right.setPower(output);
    }

    public void setState(Extension_States state) {
        this.state = state;
    }

    public double getPos() {
        return right.getCurrentPosition();
    }

    public boolean isAtTarget() {
        return controller.atSetPoint();
    }
}
