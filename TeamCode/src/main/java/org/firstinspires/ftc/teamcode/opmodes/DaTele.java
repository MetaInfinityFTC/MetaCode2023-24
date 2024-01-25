package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.armDeposit;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.armTransfer;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.bothPixels;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.onePixel;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.retracted;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.wristDeposit;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.wrist30degree;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.wrist90degree;
import static org.firstinspires.ftc.teamcode.subsystem.intake.Virtual4Bar.clawClose;
import static org.firstinspires.ftc.teamcode.subsystem.intake.Virtual4Bar.v4bGround;
import static org.firstinspires.ftc.teamcode.subsystem.intake.Virtual4Bar.v4bStackHigh;
import static org.firstinspires.ftc.teamcode.subsystem.intake.Virtual4Bar.v4bStackMid;
import static org.firstinspires.ftc.teamcode.subsystem.intake.Virtual4Bar.v4bTransfer;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit;
import org.firstinspires.ftc.teamcode.subsystem.deposit.Slides;
import org.firstinspires.ftc.teamcode.subsystem.extendo.Extendo;
import org.firstinspires.ftc.teamcode.subsystem.intake.Virtual4Bar;

@Config
@TeleOp(name = "TeleOp")
public class DaTele extends LinearOpMode {
    Deposit deposit;
    Slides slides;
    Extendo extendo;
    Virtual4Bar virtual4Bar;
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;
    Servo LeftHang;
    Servo RightHang;
    Servo Drone;

    public static double clawpos = 0.5;
    public static double v4bpos = 1.0;
    public static double armpos = 0.5;
    public static double wristpos = 0.5;
    public static double fingerpos = 0.5;



    public void runOpMode() {
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        LeftHang = hardwareMap.servo.get("LeftHang");
        RightHang = hardwareMap.servo.get("RightHang");
        Drone = hardwareMap.servo.get("Drone");

        extendo = new Extendo(hardwareMap);
        slides = new Slides(hardwareMap);
        deposit = new Deposit(hardwareMap);
        virtual4Bar = new Virtual4Bar(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
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

            deposit.setWrist(wristpos);
            deposit.setArm(armpos);
            deposit.setFinger(fingerpos);
            virtual4Bar.setClaw(clawpos);
            virtual4Bar.setV4b(v4bpos);

            /*if(gamepad2.left_bumper)
                deposit.setArm(armTransfer);
                //deposit.setWrist(wristTransfer);
            if(gamepad2.right_bumper)
                deposit.setArm(armDeposit);
                //deposit.setWrist(wristDeposit);
            if(gamepad2.a)
                deposit.setFinger(retracted);
            if(gamepad2.b)
                deposit.setFinger(bothPixels);
            if(gamepad2.x)
                deposit.setFinger(onePixel);

            if(gamepad2.dpad_left)
                virtual4Bar.setV4b(v4bGround);
            if(gamepad2.dpad_right)
                virtual4Bar.setV4b(v4bTransfer);
            if(gamepad2.dpad_up)
                virtual4Bar.setV4b(v4bStackHigh);
            if(gamepad2.dpad_down)
                virtual4Bar.setV4b(v4bStackMid);

            if(gamepad2.y)
                virtual4Bar.setClaw(clawClose);*/

        }
    }
}
