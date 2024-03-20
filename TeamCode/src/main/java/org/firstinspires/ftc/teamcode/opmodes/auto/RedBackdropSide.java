package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.armDeposit90;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.armPreTransfer;
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
import static org.firstinspires.ftc.teamcode.subsystem.intake.Virtual4Bar.v4bPreTransfer;
import static org.firstinspires.ftc.teamcode.subsystem.intake.Virtual4Bar.v4bStackHigh;
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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.AbstractedMachine;
import org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit;
import org.firstinspires.ftc.teamcode.subsystem.deposit.Slides;
import org.firstinspires.ftc.teamcode.subsystem.extendo.Extendo;
import org.firstinspires.ftc.teamcode.subsystem.intake.Virtual4Bar;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.processors.PropProcessor;
import org.firstinspires.ftc.teamcode.vision.processors.RedPropProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name="\uD83D\uDC80\\RedBackdrop", preselectTeleOp = "DaTele")
public class RedBackdropSide extends LinearOpMode {
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

    boolean trasnferring = false;

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
        left.setDirection(DcMotorSimple.Direction.REVERSE);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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
        RightHang.setPosition(0.9);

        drone.setPosition(0);

        Pose2d startpose = new Pose2d(14.75, -61.5, Math.toRadians(-90));
        drive.setPoseEstimate(startpose);

        StateMachine transferMachine = AbstractedMachine.getTransferMachine(virtual4Bar, extendo, deposit);

        TrajectorySequence leftPurple = drive.trajectorySequenceBuilder(startpose)
                .UNSTABLE_addTemporalMarkerOffset(0.8,() -> {
                    deposit.setWrist(wrist90degree);
                    deposit.setArm(armDeposit90);
                    setPidTarget(-100, 0.5);
                    extendo.setState(farspike);
                    virtual4Bar.setV4b(0.92);
                })
                .lineToSplineHeading(new Pose2d(43, -30, Math.toRadians(-180)))
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    virtual4Bar.setClaw(clawOpen);
                })
                .waitSeconds(0.5)
                .lineToSplineHeading(new Pose2d(43, -27, Math.toRadians(-180)))
                .addTemporalMarker(() -> {
                    deposit.setFinger(zeroPixel);
                })
                .forward(4)
                .addTemporalMarker(() -> {
                    deposit.setWrist(wristTransfer);
                    deposit.setArm(armPreTransfer);
                    extendo.setState(retracted);
                    virtual4Bar.setClaw(clawClose);
                    virtual4Bar.setV4b(v4bTransfer);
                    setPidTarget(0, 0.5);
                })
                .strafeLeft(32)
                .build();

        TrajectorySequence middlePurple = drive.trajectorySequenceBuilder(startpose)
                .UNSTABLE_addTemporalMarkerOffset(0.8,() -> {
                    deposit.setWrist(wrist90degree);
                    deposit.setArm(armDeposit90);
                    setPidTarget(0, 0.5);
                    extendo.setState(midspike);
                    virtual4Bar.setV4b(0.92);
                })
                .lineToSplineHeading(new Pose2d(46, -29, Math.toRadians(-195)))
                .addTemporalMarker(() -> {
                    //place yellow & purple
                    virtual4Bar.setClaw(clawOpen);
                    deposit.setFinger(zeroPixel);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    deposit.setWrist(wristTransfer);
                    deposit.setArm(armPreTransfer);
                    extendo.setState(retracted);
                    virtual4Bar.setClaw(clawClose);
                    virtual4Bar.setV4b(v4bStackHigh);
                    setPidTarget(0, 0.5);
                })
                .splineToSplineHeading(new Pose2d(20, -35, Math.toRadians(-180)), Math.toRadians(180))
                .addTemporalMarker(()->extendo.setState(extended))
                .lineTo(new Vector2d(-15, -35))
                .waitSeconds(0.35)
                .addTemporalMarker(() -> {
                    trasnferring = true;
                    transferMachine.start();
                })
                .lineTo(new Vector2d(20, -35))
                .splineToConstantHeading(new Vector2d(45, -30), Math.toRadians(0))
                .addTemporalMarker(() -> {
                    trasnferring = false;
                    transferMachine.reset();
                    transferMachine.stop();
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    //drop
                })
                .splineToConstantHeading(new Vector2d(20, -35), Math.toRadians(180))
                .lineTo(new Vector2d(-15, -35))
                .waitSeconds(0.35)
                .addTemporalMarker(() -> {
                    //grab
                })
                .lineTo(new Vector2d(20, -35))
                .splineToConstantHeading(new Vector2d(45, -30), Math.toRadians(0))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    //drop
                })
                .splineToConstantHeading(new Vector2d(20, -10.5), Math.toRadians(180))
                .lineTo(new Vector2d(-15, -10.5))
                .waitSeconds(0.35)
                .addTemporalMarker(() -> {
                    //grab
                })
                .lineTo(new Vector2d(20, -10.5))
                .splineToConstantHeading(new Vector2d(45, -30), Math.toRadians(0))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    //drop
                })
                .splineToConstantHeading(new Vector2d(20, -10.5), Math.toRadians(180))
                .lineTo(new Vector2d(-15, -10.5))
                .waitSeconds(0.35)
                .addTemporalMarker(() -> {
                    //grab
                })
                .lineTo(new Vector2d(20, -10.5))
                .splineToConstantHeading(new Vector2d(45, -30), Math.toRadians(0))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    //drop
                })
                .lineToSplineHeading(new Pose2d(45, -57, Math.toRadians(-180)))
                .build();

        TrajectorySequence rightPurple = drive.trajectorySequenceBuilder(startpose)
                .UNSTABLE_addTemporalMarkerOffset(1,() -> {
                    deposit.setWrist(wrist90degree);
                    deposit.setArm(armDeposit90);
                    setPidTarget(-100, 0.5);
                    extendo.setState(closespike);
                })
                .lineToSplineHeading(new Pose2d(43, -36, Math.toRadians(-180)))
                .addTemporalMarker(() -> {
                    deposit.setFinger(zeroPixel);
                    virtual4Bar.setV4b(0.92);
                })
                .waitSeconds(1)
                .forward(4)
                .strafeRight(8)
                .addTemporalMarker(() -> {
                    virtual4Bar.setClaw(clawOpen);
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    setPidTarget(0, 0.5);
                    virtual4Bar.setClaw(clawClose);
                    virtual4Bar.setV4b(v4bTransfer);
                    extendo.setState(retracted);
                    deposit.setWrist(wristTransfer);
                    deposit.setArm(armPreTransfer);
                })
                .strafeLeft(25)
                .build();

        while(!isStarted()){
            location = redPropProcessor.getLocation();
            telemetry.update();
        }
        waitForStart();
        switch(location){
            case LEFT:
                drive.followTrajectorySequenceAsync(leftPurple);
                break;
            case MIDDLE:
                drive.followTrajectorySequenceAsync(middlePurple);
                break;
            case RIGHT:
                drive.followTrajectorySequenceAsync(rightPurple);
                break;
        }
        while(opModeIsActive()){
            drive.update();
            extendo.update();
            if (trasnferring) {
                transferMachine.update();
            }
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

