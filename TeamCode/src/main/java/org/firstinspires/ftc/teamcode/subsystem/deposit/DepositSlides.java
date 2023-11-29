package org.firstinspires.ftc.teamcode.subsystem.deposit;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PController;
import com.arcrobotics.ftclib.controller.PDController;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
public class DepositSlides{

    static DcMotorEx leftSlide;
    static DcMotorEx rightSlide;
    public static double kP = 0.03;
    public static double kI = 0;
    public static double kD = 0.0001;
    public static double kF = 0.0002;
    public static int liftTargetPos = 0;
    public static PIDController pid;

    // Creates a PIDFController with gains kP, kI, kD, and kF
    public void init(DcMotorEx slide1) {
        leftSlide = slide1;
        //rightSlide = slide2;
        pid = new PIDController(kP, kI, kD);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        //liftTargetPos = 0;
    }
    /*
    CODE TO TUNE
    @Override
    public void init() {
        leftSlide = hardwareMap.get(DcMotorEx.class, "slide");
        //rightSlide = slide2;
        pid = new PIDController(kP, kI, kD);
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        //liftTargetPos = 0;
    }*/

    public void loop() {
        pid.setPID(kP, kI, kD);
        double pidPower = pid.calculate(leftSlide.getCurrentPosition(), liftTargetPos);
        double ffPower = kF * leftSlide.getCurrentPosition();
        leftSlide.setPower(ffPower + pidPower);
        double currentpos = leftSlide.getCurrentPosition();
        //rightSlide.setPower(ffPower + pidPower);
    }

    public void changeHeight(int pos) {
        if(liftTargetPos>650)
            liftTargetPos = 650;
        else if(liftTargetPos < 0)
            liftTargetPos = 20;
        else
        liftTargetPos = pos;
    }

    public void reset() {
        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftTargetPos = 0;
    }

}

