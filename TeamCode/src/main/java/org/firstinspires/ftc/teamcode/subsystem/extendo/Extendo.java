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
    public static double p = 0.03, i = 0, d = 0.0001, f = 0.0004;
    public static double tolerance = 3;

    PIDFController controller = new PIDFController(p, i, d, f);

    Extension_States state;

    //enum to house states with better names
    public enum Extension_States {
        retracted(0), hung(1730),  first(1800), second(2550), extended(3700), closespike(1800), midspike(2550), farspike(3700);
        private double ticks;
        Extension_States(double ticks) { this.ticks = ticks;}
        public double getTicks() { return ticks; }
    }


    public Extendo(HardwareMap hardwareMap) {
        left = hardwareMap.dcMotor.get("leftEx");
        left.setDirection(DcMotorSimple.Direction.REVERSE);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        controller.setTolerance(tolerance);
        setState(Extension_States.retracted);
    }


    @Override
    public void update() {
        controller.setPIDF(p,i,d,f);
        controller.setTolerance(tolerance);
        setPower(controller.calculate(left.getCurrentPosition(), state.getTicks()));
    }

    public void setPower(double power){
        left.setPower(power);
    }

    @Override
    public void init() {
        setState(Extension_States.retracted);
    }

    public void setState(Extension_States state) {
        this.state = state;
    }

    public double getPos() {
        return left.getCurrentPosition();
    }

    public boolean isAtTarget() {
        return controller.atSetPoint();
    }


    //RTP
    public void extendosetPidTarget(int slidePOS, double motorPower) {
        //base encoder code
        left.setTargetPosition(slidePOS);
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left.setPower(motorPower);
    }

}
