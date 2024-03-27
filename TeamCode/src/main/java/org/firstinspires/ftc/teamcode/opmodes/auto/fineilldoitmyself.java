package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.armDeposit90;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.armPreTransfer;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.armTransfer;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.bothPixels;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.wrist90degree;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.wristTransfer;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.zeroPixel;
import static org.firstinspires.ftc.teamcode.subsystem.extendo.Extendo.Extension_States.closespike;
import static org.firstinspires.ftc.teamcode.subsystem.extendo.Extendo.Extension_States.extended;
import static org.firstinspires.ftc.teamcode.subsystem.extendo.Extendo.Extension_States.farspike;
import static org.firstinspires.ftc.teamcode.subsystem.extendo.Extendo.Extension_States.midspike;
import static org.firstinspires.ftc.teamcode.subsystem.extendo.Extendo.Extension_States.retracted;
import static org.firstinspires.ftc.teamcode.subsystem.intake.Virtual4Bar.clawClose;
import static org.firstinspires.ftc.teamcode.subsystem.intake.Virtual4Bar.clawOpen;
import static org.firstinspires.ftc.teamcode.subsystem.intake.Virtual4Bar.v4bGround;
import static org.firstinspires.ftc.teamcode.subsystem.intake.Virtual4Bar.v4bPreTransfer;
import static org.firstinspires.ftc.teamcode.subsystem.intake.Virtual4Bar.v4bStackHigh;
import static org.firstinspires.ftc.teamcode.subsystem.intake.Virtual4Bar.v4bStackMid;
import static org.firstinspires.ftc.teamcode.subsystem.intake.Virtual4Bar.v4bTransfer;
import static org.firstinspires.ftc.teamcode.vision.processors.PropProcessor.Location.MIDDLE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.AbstractedMachine;
import org.firstinspires.ftc.teamcode.subsystem.AbstractedMachineRTP;
import org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit;
import org.firstinspires.ftc.teamcode.subsystem.deposit.Slides;
import org.firstinspires.ftc.teamcode.subsystem.extendo.Extendo;
import org.firstinspires.ftc.teamcode.subsystem.intake.Virtual4Bar;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.processors.PropProcessor;
import org.firstinspires.ftc.teamcode.vision.processors.RedPropProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name="\uD83D\uDC80\\Fine, I'll Do It Myself.", preselectTeleOp = "DaTele")

public class fineilldoitmyself extends LinearOpMode {

    private PropProcessor.Location location = MIDDLE;
    private RedPropProcessor redPropProcessor;
    private VisionPortal visionPortal;

    Deposit deposit;
    Slides slides;
    Extendo extendo;
    Virtual4Bar virtual4Bar;

    Servo LeftHang;
    Servo RightHang;
    Servo drone;
    SampleMecanumDrive drive;

    DcMotor left;
    DcMotor right;

    DcMotor leftextendo;
    DcMotor rightextendo;

    boolean trasnferring = false;

    double v4bStackHeight = v4bStackHigh, ticker = 1, cycle = 0;

    public enum states {
        initial, grab, drop, end
    }

    @Override
    public void runOpMode(){
        drive = new SampleMecanumDrive(hardwareMap);
        redPropProcessor = new RedPropProcessor(telemetry);
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), redPropProcessor);

        LeftHang = hardwareMap.servo.get("LeftHang");
        RightHang = hardwareMap.servo.get("RightHang");
        drone = hardwareMap.servo.get("drone");

        left = hardwareMap.dcMotor.get("lSlide");
        right = hardwareMap.dcMotor.get("rSlide");

        leftextendo = hardwareMap.dcMotor.get("lSlide");
        rightextendo = hardwareMap.dcMotor.get("rSlide");

        leftextendo.setDirection(DcMotorSimple.Direction.REVERSE);
        leftextendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightextendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        extendo = new Extendo(hardwareMap);
        slides = new Slides(hardwareMap);
        deposit = new Deposit(hardwareMap);
        virtual4Bar = new Virtual4Bar(hardwareMap);

        deposit.setArm(armPreTransfer);
        deposit.setFinger(bothPixels);
        deposit.setWrist(wristTransfer);
        virtual4Bar.setClaw(clawClose);
        virtual4Bar.setV4b(v4bPreTransfer);

        LeftHang.setPosition(0.25);
        RightHang.setPosition(0.85);

        drone.setPosition(0);

        Pose2d startpose = new Pose2d(14.75, -61.5, Math.toRadians(-90));
        drive.setPoseEstimate(startpose);

        StateMachine transferMachine = AbstractedMachineRTP.getTransferMachine(virtual4Bar, extendo, deposit);
        StateMachine dropMachine = AbstractedMachineRTP.dropMachine(deposit);

        TrajectorySequence middlePurple = drive.trajectorySequenceBuilder(startpose)
                .UNSTABLE_addTemporalMarkerOffset(0.8,() -> {
                    deposit.setWrist(wrist90degree);
                    deposit.setArm(armDeposit90);
                    setPidTarget(0, 0.5);
                    extendo.extendosetPidTarget(400, 1);
                    virtual4Bar.setV4b(v4bStackHigh);
                })
                .lineToSplineHeading(new Pose2d(46, -31, Math.toRadians(-195)))
                .addTemporalMarker(() -> {
                    //place yellow & purple
                    virtual4Bar.setClaw(clawOpen);
                    deposit.setFinger(zeroPixel);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    deposit.setWrist(wristTransfer);
                    deposit.setArm(armPreTransfer);
                    extendo.extendosetPidTarget(0, 1);
                    virtual4Bar.setClaw(clawClose);
                    virtual4Bar.setV4b(v4bStackHigh);
                    setPidTarget(0, 0.5);
                }).build();
        TrajectorySequence grab = drive.trajectorySequenceBuilder(middlePurple.end())
                .addTemporalMarker(()->{
                    setPidTarget(0, 1);
                    virtual4Bar.setClaw(clawClose);
                    virtual4Bar.setV4b(v4bStackHeight);})
                .lineToSplineHeading(new Pose2d(20, -33 - (2.5 * cycle), Math.toRadians(-180)))
                .addTemporalMarker(()-> {
                    extendo.extendosetPidTarget(1050, 1);
                    virtual4Bar.setClaw(clawOpen);
                })
                .lineTo(new Vector2d(-20.5, -33  - (2.5 * cycle)))
                .waitSeconds(0.2)
                .addTemporalMarker(()-> {transferMachine.start(); trasnferring = true;})
                .waitSeconds(0.3)
                .lineTo(new Vector2d(20, -33 - (2.5 * cycle)))
                .splineToConstantHeading(new Vector2d(44.5, -29), Math.toRadians(0))
                .build();

        StateMachine master = new StateMachineBuilder()
                .state(states.initial)
                .onEnter(()-> drive.followTrajectorySequenceAsync(middlePurple))
                .transition(()->!drive.isBusy(), states.grab)
                .state(states.grab)
                .onEnter(()-> drive.followTrajectorySequenceAsync(grab))
                .transition(()-> !drive.isBusy() && !transferMachine.isRunning(), () -> {
                    trasnferring = false; transferMachine.stop(); transferMachine.reset();
                    deposit.setWrist(wrist90degree); deposit.setArm(armDeposit90); setPidTarget(-200, 1);
                })
                .state(states.drop)
                .onEnter(dropMachine::start)
                .loop(dropMachine::update)
                .transition(()->!dropMachine.isRunning() && ticker < 2, states.grab, ()-> {dropMachine.stop(); dropMachine.reset();})
                .transition(()->!dropMachine.isRunning() && ticker > 1, states.end, ()-> {dropMachine.stop(); dropMachine.reset();})
                .onExit(()->{ticker+=1; v4bStackHeight = v4bStackMid; cycle++;})
                .state(states.end)
                .onEnter(()-> {
                    deposit.setWrist(wristTransfer);
                    deposit.setArm(armPreTransfer);
                    extendo.extendosetPidTarget(0, 1);
                    virtual4Bar.setClaw(clawClose);
                    virtual4Bar.setV4b(v4bTransfer);
                    extendo.extendosetPidTarget(0, 0.5);
                    setPidTarget(0, 1);})

                .build();


        while(!isStarted()){
            location = redPropProcessor.getLocation();
            telemetry.update();
        }
        waitForStart();
        master.start();
        visionPortal.close();
        while(opModeIsActive()){
            master.update();
            if (trasnferring) {
                transferMachine.update();
            }
            drive.update();
        }
    }
    public void setPidTarget(int slidePOS, double motorPower) {
        //base encoder code
        left.setTargetPosition(slidePOS);
        right.setTargetPosition(slidePOS);
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left.setPower(motorPower);
        right.setPower(motorPower);
    }

}