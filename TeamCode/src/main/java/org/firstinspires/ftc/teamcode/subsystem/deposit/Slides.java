package org.firstinspires.ftc.teamcode.subsystem.deposit;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystem.Subsystem;

@Config
public class Slides implements Subsystem {
    private PIDController controller;
    public static double p = 0.01, i = 0, d = 0.0; // d = dampener (dampens arm movement and is scary). ignore i
    public static double f = .001;  // prevents arm from falling from gravity


    public static double LiftTarget = 0; // target position

    //public static int START_POS = 0;
    public static int LOW = 0;
    public static int MID = 0;
    public static int HIGH = 0;

    private DcMotorEx llift;
    private DcMotorEx rlift;

    public int row = 0;

    public static double row0 = 0;
    public static double row1 = 100;
    public static double row2 = 200;
    public static double row3 = 300;
    public static double row4 = 400;
    public static double row5 = 500;
    public static double row6 = 600;
    public static double row7 = 700;
    public static double row8 = 800;

    public static int rows = 8;


    public Slides(HardwareMap hardwareMap) {
        controller = new PIDController(p, i, d);

        llift = hardwareMap.get(DcMotorEx.class,"lSlide");
        rlift = hardwareMap.get(DcMotorEx.class,"rSlide");

        llift.setDirection(DcMotorEx.Direction.FORWARD);
        rlift.setDirection(DcMotorEx.Direction.REVERSE);

        llift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rlift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        llift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rlift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void update() {
        controller.setPID(p, i, d);

        int larmPos = llift.getCurrentPosition();
        int rarmPos = rlift.getCurrentPosition();

        double Lpid = controller.calculate(larmPos, LiftTarget);
        double Rpid = controller.calculate(rarmPos, LiftTarget);

        // double Lff = Math.cos(Math.toRadians(LiftTarget / ticks_in_degree)) * f; //* (12/voltageSensor.getVoltage()
        // double Rff = Math.cos(Math.toRadians(LiftTarget / ticks_in_degree)) * f; // * (12/voltageSensor.getVoltage()

        double Lpower = Lpid + f;
        double Rpower = Rpid + f;

        llift.setPower(Lpower);
        rlift.setPower(Rpower);

    }
    public void setPidTarget(double target) {
        //base encoder code
        LiftTarget = target;
    }

    @Override
    public void init() {
        setPosition();;
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
        return rlift.getCurrentPosition();
    }

}