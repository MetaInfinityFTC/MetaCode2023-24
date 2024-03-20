package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.armDeposit30;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.armDeposit90;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.armPreTransfer;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.armTransfer;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.bothPixels;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.onePixel;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.wristTransfer;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.zeroPixel;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.wrist30degree;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.wrist90degree;
import static org.firstinspires.ftc.teamcode.subsystem.extendo.Extendo.Extension_States.extended;
import static org.firstinspires.ftc.teamcode.subsystem.extendo.Extendo.Extension_States.first;
import static org.firstinspires.ftc.teamcode.subsystem.extendo.Extendo.Extension_States.hung;
import static org.firstinspires.ftc.teamcode.subsystem.extendo.Extendo.Extension_States.retracted;
import static org.firstinspires.ftc.teamcode.subsystem.extendo.Extendo.Extension_States.second;
import static org.firstinspires.ftc.teamcode.subsystem.intake.Virtual4Bar.clawClose;
import static org.firstinspires.ftc.teamcode.subsystem.intake.Virtual4Bar.clawOpen;
import static org.firstinspires.ftc.teamcode.subsystem.intake.Virtual4Bar.v4bGround;
import static org.firstinspires.ftc.teamcode.subsystem.intake.Virtual4Bar.v4bStackHigh;
import static org.firstinspires.ftc.teamcode.subsystem.intake.Virtual4Bar.v4bStackMid;
import static org.firstinspires.ftc.teamcode.subsystem.intake.Virtual4Bar.v4bTransfer;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.Blinkdin;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.AbstractedMachine;
import org.firstinspires.ftc.teamcode.subsystem.AbstractedMachineRTP;
import org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit;
import org.firstinspires.ftc.teamcode.subsystem.deposit.Slides;
import org.firstinspires.ftc.teamcode.subsystem.extendo.Extendo;
import org.firstinspires.ftc.teamcode.subsystem.intake.Virtual4Bar;
import org.firstinspires.ftc.teamcode.telecontrol.DriveControl;

@Config
@TeleOp(name = "DaTele")
public class DaTele extends LinearOpMode {
    Deposit deposit;
    Slides slides;
    Extendo extendo;
    Virtual4Bar virtual4Bar;
    Servo LeftHang;
    Servo RightHang;
    Servo drone;
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;

    DcMotor left;
    DcMotor right;

    ElapsedTime timer;

    Blinkdin led;

    enum States {
        NEUTRAL, INTAKE, PRE_INTAKE, DEPOSIT30, TRANSFER, GRAB, DROP_ONE_PIXEL, DROP_TWO_PIXEL, TILL_TRANSFER, TILL_DEPO, CLAW_OPEN, WRIST90, CLAWINITIAL_OPEN, PRE_PRE_INTAKE, SECONDPIXEL, SETWRIST, EXTEND, REARANGE, HIGHSTACK, MIDSTACK, DRONE, HANG, PRE_HANG, SECOND, DEPOSIT90
    }

    public void runOpMode() {
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        left = hardwareMap.dcMotor.get("lSlide");
        right = hardwareMap.dcMotor.get("rSlide");
        left.setDirection(DcMotorSimple.Direction.REVERSE);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        LeftHang = hardwareMap.servo.get("LeftHang");
        RightHang = hardwareMap.servo.get("RightHang");
        drone = hardwareMap.servo.get("drone");

        extendo = new Extendo(hardwareMap);
        slides = new Slides(hardwareMap);
        deposit = new Deposit(hardwareMap);
        virtual4Bar = new Virtual4Bar(hardwareMap);

        led = new Blinkdin(hardwareMap.get(RevBlinkinLedDriver.class, "led"));

        led.changePattern(RevBlinkinLedDriver.BlinkinPattern.LAWN_GREEN);
        led.update();

        timer = new ElapsedTime();

        LeftHang.setPosition(0.25);
        RightHang.setPosition(0.9);

        drone.setPosition(0);

        double slidescontrol = 0;

        StateMachine transferMachine = AbstractedMachineRTP.getTransferMachine(virtual4Bar, extendo, deposit);

        StateMachine globalMachine = new StateMachineBuilder()
                .state(States.NEUTRAL)
                .loop(() -> {
                    virtual4Bar.setV4b(v4bTransfer);
                    virtual4Bar.setClaw(0.34);
                    setPidTarget(0, 1);
                    extendo.setState(retracted);
                    deposit.setArm(armTransfer);
                    deposit.setWrist(wristTransfer);
                    deposit.setFinger(zeroPixel);
                })
                .transition(() -> gamepad1.a, States.PRE_INTAKE)
                .transition(() -> gamepad1.b, States.SECONDPIXEL)
                .transition(() -> gamepad1.right_bumper, States.REARANGE)
                .transition(() -> gamepad1.y, States.DRONE)
                .transition(() -> gamepad1.x, States.PRE_HANG)

                .state(States.REARANGE)
                .onEnter(() -> {
                    deposit.setArm(armDeposit90);
                    deposit.setWrist(wrist90degree);
                    deposit.setFinger(bothPixels);
                })
                .transition(() -> gamepad1.a, States.PRE_INTAKE)
                .transition(() -> gamepad1.left_bumper, States.NEUTRAL)
                .transition(() -> gamepad1.b, States.TRANSFER)

                .state(States.PRE_HANG)
                .onEnter(() -> {
                    deposit.setArm(armPreTransfer);
                    LeftHang.setPosition(0.8);
                    RightHang.setPosition(0.5);
                })
                .transitionTimed(0.5)
                .state(States.HANG)
                .onEnter(() -> {
                    virtual4Bar.setV4b(0.7);
                    extendo.setState(hung);
                })
                .transition(() -> gamepad1.a, States.PRE_INTAKE)
                .transition(() -> gamepad1.left_bumper, States.NEUTRAL)
                .transition(() -> gamepad1.b, States.TRANSFER)

                .state(States.DRONE)
                .onEnter(() -> {
                    drone.setPosition(0.5);
                })
                .transitionTimed(.5, States.NEUTRAL)

                .state(States.PRE_INTAKE)
                .onEnter(() -> {
                    deposit.setFinger(zeroPixel);
                    deposit.setArm(armPreTransfer);
                    virtual4Bar.setClaw(clawClose);
                })
                .transitionTimed(0.5) // putting deposit out before v4b
                .state(States.PRE_PRE_INTAKE)
                .onEnter(() -> {
                    virtual4Bar.setV4b(0.8);
                })
                .transitionTimed(1)
                .state(States.CLAWINITIAL_OPEN)
                .onEnter(() -> {
                    virtual4Bar.setV4b(v4bGround);
                    virtual4Bar.setClaw(clawOpen);
                })
                .transition(() -> gamepad1.b, States.TRANSFER)
                .transition(() -> gamepad1.left_bumper,  States.MIDSTACK)
                .transition(() -> gamepad1.right_bumper,  States.HIGHSTACK)

                .state(States.MIDSTACK)
                .onEnter(() -> {
                    virtual4Bar.setV4b(v4bStackMid);
                    virtual4Bar.setClaw(clawOpen);
                })
                .transition(() -> gamepad1.b, States.TRANSFER)
                .transition(() -> gamepad1.left_bumper,  States.CLAWINITIAL_OPEN)
                .transition(() -> gamepad1.right_bumper,  States.HIGHSTACK)

                .state(States.HIGHSTACK)
                .onEnter(() -> {
                    virtual4Bar.setV4b(v4bStackHigh);
                    virtual4Bar.setClaw(clawOpen);
                })
                .transition(() -> gamepad1.b, States.TRANSFER)
                .transition(() -> gamepad1.left_bumper,  States.CLAWINITIAL_OPEN)
                .transition(() -> gamepad1.right_bumper,  States.MIDSTACK)

                // abstracted transfer machine
                .state(States.TRANSFER)
                .onEnter(transferMachine::start)
                .loop(transferMachine::update)
                .onExit(() -> {
                    transferMachine.stop();
                    transferMachine.reset();
                })
                .transition(() -> !transferMachine.isRunning()) // transition once transfer is finished

                .state(States.TILL_DEPO)
                .transition(() -> gamepad1.x) //press x to go to deposit
                .transition(() -> gamepad1.a, States.PRE_INTAKE)
                .transition(() -> gamepad1.left_bumper, States.NEUTRAL)

                .state(States.DEPOSIT90)
                .onEnter(() -> {
                    deposit.setFinger(bothPixels);
                    deposit.setArm(armDeposit90);
                })
                .transitionTimed(1)
                .state(States.WRIST90)
                .onEnter(() -> {
                    virtual4Bar.setClaw(clawClose);
                    deposit.setWrist(wrist90degree);
                })
                .transition(() -> gamepad1.right_bumper, States.DEPOSIT30)
                .transition(() -> gamepad1.y, States.DROP_TWO_PIXEL)
                .transition(() -> gamepad1.start, States.DROP_ONE_PIXEL)
                .transition(() -> gamepad1.a, States.PRE_INTAKE)
                .transition(() -> gamepad1.left_bumper, States.NEUTRAL)
                .transition(() -> gamepad1.b, States.TRANSFER)

                .state(States.DEPOSIT30)
                .onEnter(() -> {
                    virtual4Bar.setClaw(clawClose);
                    deposit.setWrist(wrist30degree);
                    deposit.setArm(armDeposit30);
                })
                .transition(() -> gamepad1.x, States.DEPOSIT90) // x to go back to deposit 90 position
                .transition(() -> gamepad1.y, States.DROP_TWO_PIXEL) // y to drop both pixel/the remaining pixel
                .transition(() -> gamepad1.start, States.DROP_ONE_PIXEL) //start to just drop one pixel
                .transition(() -> gamepad1.a, States.PRE_INTAKE)
                .transition(() -> gamepad1.left_bumper, States.NEUTRAL)
                .transition(() -> gamepad1.b, States.TRANSFER)

                .state(States.DROP_ONE_PIXEL)
                .onEnter(() -> {
                    deposit.setFinger(onePixel);
                })
                .transition(() -> gamepad1.y, States.DROP_TWO_PIXEL) // on y drop second pixel
                .transition(() -> gamepad1.x, States.DEPOSIT90) // x to go back to deposit 90 position

                .state(States.DROP_TWO_PIXEL)
                .onEnter(() -> deposit.setFinger(onePixel))
                .transitionTimed(0.4, States.SECOND) // going back to neutral state

                .state(States.SECOND)
                .onEnter(() -> deposit.setFinger(zeroPixel))
                .transitionTimed(0.4, States.SETWRIST)

                .state(States.SETWRIST)
                .onEnter(() -> deposit.setWrist(wristTransfer))
                .transitionTimed(0.3, States.NEUTRAL)

                .build();

        waitForStart();

        globalMachine.start();

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

            globalMachine.update();

            telemetry.addData("State: ", globalMachine.getState());
            telemetry.update();
            //update PID loop for Extendo and Outtake Slides
            extendo.update();

            //extendo control
            if(gamepad1.dpad_up)
                extendo.extendosetPidTarget(1050, 1);
            if(gamepad1.dpad_right)
                setPidTarget(left.getCurrentPosition()+40, 1);
            if(gamepad1.dpad_left)
                setPidTarget(left.getCurrentPosition()-40, 1);
            if(gamepad1.dpad_down)
                extendo.extendosetPidTarget(0, 1);
        }
    }
    public void setPidTarget(double slidePOS, double motorPower) {
        //base encoder code
        left.setTargetPosition((int) slidePOS);
        right.setTargetPosition((int) slidePOS);
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left.setPower(motorPower);
        right.setPower(motorPower);
    }
}