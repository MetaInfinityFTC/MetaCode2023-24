package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import android.transition.Slide;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit;
import org.firstinspires.ftc.teamcode.subsystem.deposit.DepositSlides;
import org.firstinspires.ftc.teamcode.subsystem.intake.ActiveIntake;

@Config
@TeleOp(name = "\uD83D\uDC80\\")
public class Teleop extends LinearOpMode {
    //get our analog input from the hardwareMap
    AnalogInput analogInput;// = hardwareMap.get(AnalogInput.class, "miniaxon");
    DcMotorEx intake;
    Servo intakeservo;
    Servo bucket;
    DepositSlides SlidePID;

    ActiveIntake Intake;

    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;
    DcMotorEx slide;

    // get the voltage of our analog line
// divide by 3.3 (the max voltage) to get a value between 0 and 1
// multiply by 360 to convert it to 0 to 360 degrees
    double position;
    public void runOpMode(){
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        slide = (DcMotorEx) hardwareMap.dcMotor.get("slide");

        SlidePID = new DepositSlides();
        Intake = new ActiveIntake();

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        intakeservo = hardwareMap.servo.get("intakeservo");
        intakeservo = hardwareMap.servo.get("max");
        analogInput = hardwareMap.analogInput.get("miniaxon");
        intake = (DcMotorEx) hardwareMap.dcMotor.get("intake");

        SlidePID.init(slide);
        Intake.initintake(intake);

        double power;

        waitForStart();

        while(opModeIsActive()) {
            //set intake linkage to height
            intakeservo.setPosition(0.8);

            //let user change height of slides
            if(gamepad1.right_bumper)
                SlidePID.changeHeight(slide.getCurrentPosition() + 50);
            else if(gamepad1.left_bumper)
                SlidePID.changeHeight(slide.getCurrentPosition() - 50);

            //sets power of intake
            power = gamepad1.right_trigger - gamepad1.left_trigger;
            Intake.Intake(intake, power);

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
        }
    }
}
