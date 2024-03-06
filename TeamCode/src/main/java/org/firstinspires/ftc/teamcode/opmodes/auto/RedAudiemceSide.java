package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.armDeposit90;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.armPreTransfer;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.bothPixels;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.wrist90degree;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.wristTransfer;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.zeroPixel;
import static org.firstinspires.ftc.teamcode.subsystem.extendo.Extendo.Extension_States.retracted;
import static org.firstinspires.ftc.teamcode.subsystem.intake.Virtual4Bar.clawClose;
import static org.firstinspires.ftc.teamcode.subsystem.intake.Virtual4Bar.clawOpen;
import static org.firstinspires.ftc.teamcode.subsystem.intake.Virtual4Bar.v4bPreTransfer;
import static org.firstinspires.ftc.teamcode.subsystem.intake.Virtual4Bar.v4bTransfer;
import static org.firstinspires.ftc.teamcode.vision.processors.RedPropProcessor.Location.MIDDLE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
import org.firstinspires.ftc.teamcode.vision.processors.RedPropProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name="\uD83D\uDC80\\RedAudience")
public class RedAudiemceSide extends LinearOpMode {
    private RedPropProcessor.Location location = MIDDLE;
    private RedPropProcessor redPropProcessor;
    private VisionPortal visionPortal;

    private DcMotor slide;
    private DcMotorEx intake;
    private Servo intakeservo, bucket;

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

    @Override
    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);
        redPropProcessor = new RedPropProcessor(telemetry);
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), redPropProcessor);

        LeftHang = hardwareMap.servo.get("LeftHang");
        RightHang = hardwareMap.servo.get("RightHang");
        drone = hardwareMap.servo.get("drone");

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

        left = hardwareMap.dcMotor.get("lSlide");
        right = hardwareMap.dcMotor.get("rSlide");
        left.setDirection(DcMotorSimple.Direction.REVERSE);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        StateMachine transferMachine = AbstractedMachine.getTransferMachine(virtual4Bar, extendo, deposit);

        Pose2d startpose = new Pose2d(-36, -58, Math.toRadians(-90));
        drive.setPoseEstimate(startpose);

        TrajectorySequence leftPurple = drive.trajectorySequenceBuilder(startpose)
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    virtual4Bar.setV4b(0.92);
                })
                .lineToSplineHeading(new Pose2d(-51.5, -8.5, Math.toRadians(-90)))
                //add claw drop
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    virtual4Bar.setClaw(clawOpen);
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    virtual4Bar.setClaw(clawClose);
                    virtual4Bar.setV4b(v4bTransfer);
                })
                .lineToSplineHeading(new Pose2d(-20, -6, Math.toRadians(-180)))
                //claw in
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    deposit.setArm(0.85);
                })
                .waitSeconds(12)
                //.waitSeconds(20) -> timer for other team to finish auto
                .lineTo(new Vector2d(20, -4))
                //depo arm out now
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    deposit.setArm(armDeposit90);
                    deposit.setWrist(wrist90degree);
                    setPidTarget(-100, 1);
                })
                .splineTo(new Vector2d(43, -21), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    deposit.setFinger(zeroPixel);
                })
                .waitSeconds(1)
                .forward(3)
                .addTemporalMarker(() -> {
                    deposit.setWrist(wristTransfer);
                    deposit.setArm(armPreTransfer);
                    extendo.setState(retracted);
                    virtual4Bar.setClaw(clawClose);
                    virtual4Bar.setV4b(v4bTransfer);
                    setPidTarget(0, 0.5);
                })
                .lineTo(new Vector2d(42, -3))
                .build();

        TrajectorySequence middlePurple = drive.trajectorySequenceBuilder(startpose)
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
                    virtual4Bar.setV4b(0.92);
                })
                .lineToSplineHeading(new Pose2d(-67, -19, Math.toRadians(-90)))                //claw goes out here
                .turn(Math.toRadians(90))
                //add claw drop
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    virtual4Bar.setClaw(clawOpen);
                })
                .waitSeconds(0.3)
                //.strafeLeft(3)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    virtual4Bar.setClaw(clawClose);
                    virtual4Bar.setV4b(v4bTransfer);
                })
                .lineTo(new Vector2d(-55,-2))
                .lineToSplineHeading(new Pose2d(-20, -2, Math.toRadians(180)))
                .strafeLeft(4)
                //claw in
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    deposit.setArm(0.85);
                })
                .waitSeconds(12)
                //.waitSeconds(20) -> timer for other team to finish auto
                .lineTo(new Vector2d(10, -10))
                .lineToSplineHeading(new Pose2d(20, -10, Math.toRadians(180)))
                //depo arm out now
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    deposit.setArm(armDeposit90);
                    deposit.setWrist(wrist90degree);
                    setPidTarget(-50, 1);
                })
                .splineTo(new Vector2d(43, -29), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    deposit.setFinger(zeroPixel);
                })
                .waitSeconds(1)
                .forward(4)
                //place yellow pixel, depo arm in, etc.
                .addTemporalMarker(() -> {
                    deposit.setWrist(wristTransfer);
                    deposit.setArm(armPreTransfer);
                    extendo.setState(retracted);
                    virtual4Bar.setClaw(clawClose);
                    virtual4Bar.setV4b(v4bTransfer);
                    setPidTarget(0, 0.5);
                })
                .lineTo(new Vector2d(42, -3))
                .build();

        TrajectorySequence rightPurple = drive.trajectorySequenceBuilder(startpose)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    virtual4Bar.setV4b(0.92);
                })
                .lineToSplineHeading(new Pose2d(-46, -11, Math.toRadians(-90)))
                //claw goes out here
                .strafeLeft(5)
                .turn(Math.toRadians(45))
                //add claw drop
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    virtual4Bar.setClaw(clawOpen);
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    virtual4Bar.setClaw(clawClose);
                    virtual4Bar.setV4b(v4bTransfer);
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    deposit.setArm(0.85);
                })
                .back(10)
                .lineToSplineHeading(new Pose2d(-20, -4, Math.toRadians(-180)))
                .waitSeconds(12)
                //.waitSeconds(20) -> timer for other team to finish auto
                .lineTo(new Vector2d(20, -12))
                //depo arm out now
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    deposit.setArm(armDeposit90);
                    deposit.setWrist(wrist90degree);
                    setPidTarget(-100, 1);
                })
                .splineTo(new Vector2d(43, -37), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    deposit.setFinger(zeroPixel);
                })
                .waitSeconds(1)
                .forward(2)
                //place yellow pixel, depo arm in, etc.
                .addTemporalMarker(() -> {
                    deposit.setWrist(wristTransfer);
                    deposit.setArm(armPreTransfer);
                    extendo.setState(retracted);
                    virtual4Bar.setClaw(clawClose);
                    virtual4Bar.setV4b(v4bTransfer);
                    setPidTarget(0, 0.5);
                })
                .forward(2)
                .lineTo(new Vector2d(42, -3))
                .build();

        while (!isStarted()) {
            location = redPropProcessor.getLocation();
            telemetry.update();
        }
        waitForStart();
        switch (location) {
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
