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
import static org.firstinspires.ftc.teamcode.subsystem.deposit.Slides.pidTarget;
import static org.firstinspires.ftc.teamcode.subsystem.extendo.Extendo.Extension_States.extended;
import static org.firstinspires.ftc.teamcode.subsystem.extendo.Extendo.Extension_States.first;
import static org.firstinspires.ftc.teamcode.subsystem.extendo.Extendo.Extension_States.retracted;
import static org.firstinspires.ftc.teamcode.subsystem.extendo.Extendo.Extension_States.second;
import static org.firstinspires.ftc.teamcode.subsystem.intake.Virtual4Bar.clawClose;
import static org.firstinspires.ftc.teamcode.subsystem.intake.Virtual4Bar.clawOpen;
import static org.firstinspires.ftc.teamcode.subsystem.intake.Virtual4Bar.v4bGround;
import static org.firstinspires.ftc.teamcode.subsystem.intake.Virtual4Bar.v4bTransfer;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.AbstractedMachine;
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

    ElapsedTime timer;

    enum States {
        NEUTRAL, INTAKE, PRE_INTAKE, DEPOSIT30, TRANSFER, GRAB, DROP_ONE_PIXEL, DROP_TWO_PIXEL, TILL_TRANSFER, TILL_DEPO, CLAW_OPEN, WRIST90, DEPOSIT90
    }

    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        DriveControl driveControl = new DriveControl(drive);

        LeftHang = hardwareMap.servo.get("LeftHang");
        RightHang = hardwareMap.servo.get("RightHang");
        drone = hardwareMap.servo.get("drone");

        extendo = new Extendo(hardwareMap);
        slides = new Slides(hardwareMap);
        deposit = new Deposit(hardwareMap);
        virtual4Bar = new Virtual4Bar(hardwareMap);

        timer = new ElapsedTime();

        LeftHang.setPosition(0.2);
        RightHang.setPosition(0.8);

        drone.setPosition(0);

        StateMachine transferMachine = AbstractedMachine.getTransferMachine(virtual4Bar, extendo, deposit);

        StateMachine globalMachine = new StateMachineBuilder()
                .state(States.NEUTRAL)
                .loop(() -> {
                    virtual4Bar.setV4b(v4bTransfer);
                    slides.setPidTarget(0);
                    extendo.setState(retracted);
                    deposit.setArm(armPreTransfer);
                })
                .transition(() -> gamepad2.a)
                .state(States.PRE_INTAKE)
                .onEnter(() -> {
                    deposit.setArm(armPreTransfer);
                })
                .transitionTimed(.2) // putting deposit out before v4b
                .state(States.INTAKE)
                .onEnter(() -> {
                    virtual4Bar.setV4b(v4bGround);
                })
                .transitionTimed(0.4)
                .state(States.CLAW_OPEN)
                .onEnter(() -> {
                    virtual4Bar.setClaw(clawOpen);
                })
                .transition(() -> gamepad2.b)
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
                .transition(() -> gamepad2.x) //press x to go to deposit
                .transition(() -> gamepad2.a, States.PRE_INTAKE)
                .transition(() -> gamepad2.left_bumper, States.NEUTRAL)

                .state(States.DEPOSIT90)
                .onEnter(() -> {
                    deposit.setArm(armDeposit90);
                })
                .transitionTimed(0.4)
                .state(States.WRIST90)
                .onEnter(() -> {
                    deposit.setWrist(wrist90degree);
                })
                .transition(() -> gamepad2.right_bumper, States.DEPOSIT30)
                .transition(() -> gamepad2.y, States.DROP_TWO_PIXEL)
                .transition(() -> gamepad2.start, States.DROP_ONE_PIXEL)
                .transition(() -> gamepad2.a, States.PRE_INTAKE)
                .transition(() -> gamepad2.left_bumper, States.NEUTRAL)
                .transition(() -> gamepad2.b, States.TRANSFER)

                .state(States.DEPOSIT30)
                .onEnter(() -> {
                    virtual4Bar.setClaw(clawClose);
                    deposit.setWrist(wrist30degree);
                    deposit.setArm(armDeposit30);
                })
                .transition(() -> gamepad2.x, States.DEPOSIT90) // x to go back to deposit 90 position
                .transition(() -> gamepad2.y, States.DROP_TWO_PIXEL) // y to drop both pixel/the remaining pixel
                .transition(() -> gamepad2.start, States.DROP_ONE_PIXEL) //start to just drop one pixel
                .transition(() -> gamepad2.a, States.PRE_INTAKE)
                .transition(() -> gamepad2.left_bumper, States.NEUTRAL)
                .transition(() -> gamepad2.b, States.TRANSFER)

                .state(States.DROP_ONE_PIXEL)
                .onEnter(() -> {
                    deposit.setFinger(onePixel);
                })
                .transition(() -> gamepad2.y, States.DROP_TWO_PIXEL) // on y drop second pixel

                .state(States.DROP_TWO_PIXEL)
                .onEnter(() -> deposit.setFinger(zeroPixel))
                .transitionTimed(.3, States.NEUTRAL) // going back to neutral state

                .build();

        waitForStart();

        globalMachine.start();

        while (opModeIsActive()) {
            driveControl.update(gamepad1, gamepad2);
            globalMachine.update();

            telemetry.addData("State: ", globalMachine.getState());
            telemetry.update();
            //update PID loop for Extendo and Outtake Slides
            extendo.update();
            slides.updatePID();

            //hang
            if(gamepad1.x){
                LeftHang.setPosition(0.5);
                RightHang.setPosition(0.5);
            }

            //drone
            if(gamepad1.a)
                drone.setPosition(0.3);

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
        }
    }
}