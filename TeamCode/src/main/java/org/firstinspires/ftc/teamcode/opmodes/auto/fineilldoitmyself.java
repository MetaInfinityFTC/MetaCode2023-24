package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.armDeposit90;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.armPreTransfer;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.onePixel;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.wrist90degree;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.wristTransfer;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.zeroPixel;
import static org.firstinspires.ftc.teamcode.subsystem.intake.Virtual4Bar.clawClose;
import static org.firstinspires.ftc.teamcode.subsystem.intake.Virtual4Bar.clawOpen;
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

    boolean transferring = false;

    double v4bStackHeight = v4bStackHigh, ticker = 1;

    public enum states {
        initial, grab, drop, grab1, end2, grab2, drop1, grab3, drop2, park, grab3side, end
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
        deposit.setFinger(onePixel);
        deposit.setWrist(wristTransfer);
        virtual4Bar.setClaw(clawClose);
        virtual4Bar.setV4b(v4bPreTransfer);

        LeftHang.setPosition(0.25);
        RightHang.setPosition(0.9);

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
                    extendo.extendosetPidTarget(355, 1);
                    virtual4Bar.setV4b(v4bStackHigh);
                })
                .lineToSplineHeading(new Pose2d(47, -30, Math.toRadians(-195)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //place yellow & purple
                    virtual4Bar.setClaw(clawOpen);
                    deposit.setFinger(zeroPixel);
                })
                .waitSeconds(0.4)
                .addTemporalMarker(() -> {
                    deposit.setWrist(wristTransfer);
                    deposit.setArm(armPreTransfer);
                    extendo.extendosetPidTarget(0, 1);
                    virtual4Bar.setClaw(clawClose);
                    virtual4Bar.setV4b(0.7);
                    setPidTarget(0, 0.5);
                })
                .turn(Math.toRadians(15))
                .build();
        TrajectorySequence grab1 = drive.trajectorySequenceBuilder(middlePurple.end())
                .addTemporalMarker(()->{
                    setPidTarget(0, 1);
                    virtual4Bar.setClaw(clawOpen);
                    virtual4Bar.setV4b(v4bStackHeight);})
                .splineToConstantHeading(new Vector2d(20, -33), Math.toRadians(180))
                .addTemporalMarker(()-> {
                    extendo.extendosetPidTarget(1050, 1);
                    virtual4Bar.setClaw(clawOpen);
                })
                .lineToSplineHeading(new Pose2d(-25, -33, Math.toRadians(-180)))
                .addTemporalMarker(()-> {transferMachine.start(); transferring = true;})
                .waitSeconds(0.7)
                .lineTo(new Vector2d(20, -33))
                .splineToConstantHeading(new Vector2d(44.75, -40), Math.toRadians(0))
                .build();

        TrajectorySequence grab2 = drive.trajectorySequenceBuilder(grab1.end())
                .addTemporalMarker(()->{
                    setPidTarget(0, 1);
                    virtual4Bar.setV4b(v4bStackHeight);})
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> virtual4Bar.setClaw(clawOpen))
                .splineToConstantHeading(new Vector2d(20, -34.5), Math.toRadians(180))
                .addTemporalMarker(()-> {
                    extendo.extendosetPidTarget(1050, 1);
                    virtual4Bar.setClaw(clawOpen);
                })
                .lineToSplineHeading(new Pose2d(-23.5, -34.5, Math.toRadians(-180)))
                .addTemporalMarker(()-> {transferMachine.start(); transferring = true;})
                .waitSeconds(0.7)
                .lineTo(new Vector2d(20, -34.5))
                .splineToConstantHeading(new Vector2d(44.75, -28), Math.toRadians(0))
//                .addTemporalMarker(dropMachine::start)

                .lineTo(new Vector2d(44.75, -45))

                .build();

        TrajectorySequence grab3 = drive.trajectorySequenceBuilder(grab2.end())
                .addTemporalMarker(()->{
                    setPidTarget(0, 1);
                    virtual4Bar.setV4b(v4bStackHeight);})
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> virtual4Bar.setClaw(clawOpen))

                .setTangent(Math.toRadians(135))
                .splineToConstantHeading(new Vector2d(20, -12), Math.toRadians(180))
                .addTemporalMarker(()-> {
                    extendo.extendosetPidTarget(1050, 1);
                    virtual4Bar.setClaw(clawOpen);
                })

                // stack
                .lineTo(new Vector2d(-23.5, -12))
                .addTemporalMarker(() -> {
                    transferMachine.start();
                    transferring = true;
                })

                .waitSeconds(0.4)

                .lineTo(new Vector2d(20, -12))
                .splineToConstantHeading(new Vector2d(45, -30), Math.toRadians(0))
//                .waitSeconds(0.5)
                .lineToSplineHeading(new Pose2d(45, -57, Math.toRadians(-180))) // board
                .waitSeconds(.3)

                .addTemporalMarker(dropMachine::start)
                .build();

        StateMachine master = new StateMachineBuilder()
                .state(states.initial)
                .onEnter(()-> drive.followTrajectorySequenceAsync(middlePurple))
                .transition(()->!drive.isBusy(), states.grab1)

                .state(states.grab1)
                .onEnter(()-> drive.followTrajectorySequenceAsync(grab1))
                .transition(()-> !drive.isBusy() && !transferMachine.isRunning(), states.drop, () -> {
                    transferring = false; transferMachine.stop(); transferMachine.reset();
                    deposit.setWrist(wrist90degree); deposit.setArm(armDeposit90); setPidTarget(-200, 1);
                })
                //.waitState(0.5)

                .state(states.drop)
                .onEnter(dropMachine::start)
                .loop(dropMachine::update)
                .transition(()-> dropMachine.getState() ==  AbstractedMachineRTP.Drop.RESET && ticker < 2, states.grab2)
                .onExit(()->{
                    dropMachine.stop(); dropMachine.reset();
                    ticker+=1; v4bStackHeight = v4bStackMid;
                })

                // ~~~~~~~~~~~~~~~~

                .state(states.grab2)
                .onEnter(()-> drive.followTrajectorySequenceAsync(grab2))
                .transition(()-> !drive.isBusy() && !transferMachine.isRunning(), states.drop1, () -> {
                    transferring = false; transferMachine.stop(); transferMachine.reset();
                    deposit.setWrist(wrist90degree); deposit.setArm(armDeposit90); setPidTarget(-200, 1);
                })
                //.waitState(0.5)

                .state(states.drop1)
                // drop machine is started in the grab2 path
//                .onEnter(dropMachine::start)
                .loop(dropMachine::update)
                .transition(()->dropMachine.getState() ==  AbstractedMachineRTP.Drop.RESET && ticker < 3, states.park)
                .onExit(()->{
                    ticker+=1; v4bStackHeight = v4bStackHigh;
                    dropMachine.stop(); dropMachine.reset();
                })

                // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

                // THIRD PIXEL
                .state(states.grab3side)
                .onEnter(() -> drive.followTrajectorySequenceAsync(grab3))
                .loop(() -> {
                    transferMachine.update();
                    dropMachine.update();
                })
                .transition(() -> dropMachine.getState() == AbstractedMachineRTP.Drop.RESET && !drive.isBusy())
                .onExit(() -> {
                    dropMachine.stop();
                    dropMachine.reset();
                })

                .state(states.end)
                .onEnter(() -> {
                    deposit.setWrist(wristTransfer);
                    deposit.setArm(armPreTransfer);
                    extendo.extendosetPidTarget(0, 1);
                    virtual4Bar.setClaw(clawClose);
                    virtual4Bar.setV4b(v4bTransfer);
                    extendo.extendosetPidTarget(0, 0.5);
                    setPidTarget(0, 1);
                })
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
            if (transferring) {
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