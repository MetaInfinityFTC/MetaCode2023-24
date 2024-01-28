package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.armDeposit30;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.armDeposit90;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.armPreTransfer;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.armTransfer;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.bothPixels;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.onePixel;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.wristTransfer;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.zeroPixel;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.wristTransfer;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.wrist30degree;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.wrist90degree;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.Slides.pidTarget;
import static org.firstinspires.ftc.teamcode.subsystem.extendo.Extendo.Extension_States.extended;
import static org.firstinspires.ftc.teamcode.subsystem.extendo.Extendo.Extension_States.first;
import static org.firstinspires.ftc.teamcode.subsystem.extendo.Extendo.Extension_States.retracted;
import static org.firstinspires.ftc.teamcode.subsystem.extendo.Extendo.Extension_States.second;
import static org.firstinspires.ftc.teamcode.subsystem.intake.Virtual4Bar.clawClose;
import static org.firstinspires.ftc.teamcode.subsystem.intake.Virtual4Bar.clawOpen;
import static org.firstinspires.ftc.teamcode.subsystem.intake.Virtual4Bar.v4bGround;
import static org.firstinspires.ftc.teamcode.subsystem.intake.Virtual4Bar.v4bPreTransfer;
import static org.firstinspires.ftc.teamcode.subsystem.intake.Virtual4Bar.v4bStackHigh;
import static org.firstinspires.ftc.teamcode.subsystem.intake.Virtual4Bar.v4bStackMid;
import static org.firstinspires.ftc.teamcode.subsystem.intake.Virtual4Bar.v4bTransfer;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    Servo drone;

    ElapsedTime timer;

    public void runOpMode() {
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        LeftHang = hardwareMap.servo.get("LeftHang");
        RightHang = hardwareMap.servo.get("RightHang");
        drone = hardwareMap.servo.get("drone");

        extendo = new Extendo(hardwareMap);
        slides = new Slides(hardwareMap);
        deposit = new Deposit(hardwareMap);
        virtual4Bar = new Virtual4Bar(hardwareMap);

        timer = new ElapsedTime();

        LeftHang.setPosition(0);
        RightHang.setPosition(1);

        drone.setPosition(0);

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

            //update PID loop for Extendo and Outtake Slides
            extendo.update();
            slides.updatePID();

            //extendo control
            if(gamepad1.dpad_up)
                extendo.setState(extended);
            if(gamepad1.dpad_right)
                extendo.setState(second);
            if(gamepad1.dpad_left)
                extendo.setState(first);
            if(gamepad1.dpad_down)
                extendo.setState(retracted);

            //outtake slides control
            slides.setPidTarget(pidTarget+gamepad2.left_stick_x*10);
            if(gamepad2.dpad_down)
                slides.setPidTarget(0);

            //TODO Changing things with multiple commands to finite state machine tonight/tomorrow
            //intake v4b+claw control
            // open claw
            if(gamepad2.a)
                virtual4Bar.setClaw(clawOpen);
            //grab, bring slides back, claw up, and ready for transfer
            if(gamepad2.b) {
                virtual4Bar.setClaw(clawClose);
                extendo.setState(retracted);
                deposit.setWrist(wristTransfer);
                deposit.setArm(armPreTransfer);
                virtual4Bar.setV4b(v4bTransfer);
            }
            //transfer
            if(gamepad2.x) {
                deposit.setFinger(bothPixels);
                deposit.setArm(armTransfer);
            }
            //get claw ready to grab
            if(gamepad2.y){
                deposit.setArm(armPreTransfer);
                virtual4Bar.setV4b(v4bGround);
                virtual4Bar.setClaw(clawOpen);
            }

            //deposit control
            //after deposit, use this
            if(gamepad2.left_bumper){
                deposit.setWrist(wristTransfer);
                deposit.setArm(armTransfer);
            }
            //use this to go to low deposit position, rare
            if(gamepad2.start){
                deposit.setWrist(wrist90degree);
                deposit.setArm(armDeposit90);
            }
            //use this to go to high deposit position, more common
            if(gamepad2.left_bumper){
                deposit.setWrist(wrist30degree);
                deposit.setArm(armDeposit30);
            }
            //drop one pixel
            if(gamepad2.dpad_left)
                deposit.setFinger(onePixel);
            //drop two pixel
            if(gamepad2.dpad_left)
                deposit.setFinger(zeroPixel);

            //hang
            if(gamepad1.x){
                LeftHang.setPosition(0.5);
                RightHang.setPosition(0.5);
            }

            //drone
            if(gamepad1.a)
                drone.setPosition(0.3);

        }
    }
}
