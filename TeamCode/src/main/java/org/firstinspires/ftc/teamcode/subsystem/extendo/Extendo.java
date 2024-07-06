package org.firstinspires.ftc.teamcode.subsystem.extendo;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystem.Subsystem;

@Config
public class Extendo implements Subsystem {

    private DcMotor left, right;
    public static double p = 0.04, i = 0.004, d = 0.00051, f = 0.005;
    public static double tolerance = 3;

    PIDFController controller = new PIDFController(p, i, d, f);

    Extension_States state;

    //enum to house states with better names
    public enum Extension_States {
        retracted(0), hung(400),  first(500), second(950), extended(1050), closespike(75), midspike(400), farspike(800);
        private double ticks;
        Extension_States(double ticks) { this.ticks = ticks;}
        public double getTicks() { return ticks; }
    }


    public Extendo(HardwareMap hardwareMap) {
        left = hardwareMap.dcMotor.get("leftEx"); right = hardwareMap.dcMotor.get("rightEx");
        left.setDirection(DcMotorSimple.Direction.REVERSE);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        controller.setTolerance(tolerance);
    }


    @Override
    public void update() {
        controller.setPIDF(p,i,d,f);
        controller.setTolerance(tolerance);
        setPower(controller.calculate(right.getCurrentPosition(), state.getTicks()));
    }

    public void setPower(double power){
        left.setPower(power);
        right.setPower(power);
    }

    @Override
    public void init() {
        setState(Extension_States.retracted);
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


    //RTP
    public void extendosetPidTarget(int slidePOS, double motorPower) {
        //base encoder code
        left.setTargetPosition(slidePOS);
        right.setTargetPosition(slidePOS);
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left.setPower(motorPower);
        right.setPower(motorPower);
    }

}
