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
import static org.firstinspires.ftc.teamcode.vision.NewBluePropProcessor.Location.MIDDLE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit;
import org.firstinspires.ftc.teamcode.subsystem.deposit.Slides;
import org.firstinspires.ftc.teamcode.subsystem.extendo.Extendo;
import org.firstinspires.ftc.teamcode.subsystem.intake.ActiveIntake;
import org.firstinspires.ftc.teamcode.subsystem.intake.Virtual4Bar;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.NewBluePropProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name="\uD83D\uDC80\\BlueAudience")
public class BlueAudiemceSide extends LinearOpMode {
    private NewBluePropProcessor.Location location = MIDDLE;
    private NewBluePropProcessor bluePropProcessor;
    private VisionPortal visionPortal;

    SampleMecanumDrive drive;

    Deposit deposit;
    Slides slides;
    Extendo extendo;
    Virtual4Bar virtual4Bar;

    Servo LeftHang;
    Servo RightHang;
    Servo drone;

    @Override
    public void runOpMode(){
        drive = new SampleMecanumDrive(hardwareMap);
        bluePropProcessor = new NewBluePropProcessor(telemetry);
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), bluePropProcessor);
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

        Pose2d startpose = new Pose2d(-36-8.25, 58, Math.toRadians(90));
        drive.setPoseEstimate(startpose);

        TrajectorySequence leftPurple = drive.trajectorySequenceBuilder(startpose)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    virtual4Bar.setV4b(0.92);
                })
                .lineToSplineHeading(new Pose2d(-44, 11, Math.toRadians(90)))
                //claw goes out here
                .strafeRight(2)
                .turn(Math.toRadians(-45))
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
                .waitSeconds(12)
                .back(5)
                .lineToSplineHeading(new Pose2d(-20, 4, Math.toRadians(-180)))
                //.waitSeconds(20) -> timer for other team to finish auto
                .lineTo(new Vector2d(20, 12))
                //depo arm out now
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    deposit.setArm(armDeposit90);
                    deposit.setWrist(wrist90degree);
                    slides.setPidTarget(-20, 0.5);
                })
                .splineTo(new Vector2d(45, 42), Math.toRadians(0))
                .waitSeconds(1)
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
                    slides.setPidTarget(0, 0.5);
                })
                .forward(2)
                .lineTo(new Vector2d(42, 3))
                .build();

        TrajectorySequence middlePurple = drive.trajectorySequenceBuilder(startpose)
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
                    virtual4Bar.setV4b(0.92);
                })
                .lineToSplineHeading(new Pose2d(-64, 19, Math.toRadians(90)))                //claw goes out here
                .turn(Math.toRadians(-90))
                //add claw drop
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    virtual4Bar.setClaw(clawOpen);
                })
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    virtual4Bar.setClaw(clawClose);
                    virtual4Bar.setV4b(v4bTransfer);
                })
                .lineTo(new Vector2d(-55,4))
                .lineToSplineHeading(new Pose2d(-20, 2, Math.toRadians(180)))
                //claw in
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    deposit.setArm(0.85);
                })
                .waitSeconds(12)
                //.waitSeconds(20) -> timer for other team to finish auto
                .lineTo(new Vector2d(10, 12))
                .lineToSplineHeading(new Pose2d(20, 12, Math.toRadians(180)))
                //depo arm out now
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    deposit.setArm(armDeposit90);
                    deposit.setWrist(wrist90degree);
                    slides.setPidTarget(-20, 0.5);
                })
                .splineTo(new Vector2d(45, 31), Math.toRadians(0))
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    deposit.setFinger(zeroPixel);
                })
                .waitSeconds(0.5)
                .forward(2)
                //place yellow pixel, depo arm in, etc.
                .addTemporalMarker(() -> {
                    deposit.setWrist(wristTransfer);
                    deposit.setArm(armPreTransfer);
                    extendo.setState(retracted);
                    virtual4Bar.setClaw(clawClose);
                    virtual4Bar.setV4b(v4bTransfer);
                    slides.setPidTarget(0, 0.5);
                })
                .lineTo(new Vector2d(42, 3))
                .build();

        TrajectorySequence rightPurple = drive.trajectorySequenceBuilder(startpose)
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    virtual4Bar.setV4b(0.92);
                })
                .lineToSplineHeading(new Pose2d(-37, 10, Math.toRadians(90)))
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
                .lineToSplineHeading(new Pose2d(-20, 6, Math.toRadians(-180)))
                //claw in
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    deposit.setArm(0.85);
                })
                .waitSeconds(12)
                //.waitSeconds(20) -> timer for other team to finish auto
                .lineTo(new Vector2d(20, 4))
                //depo arm out now
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    deposit.setArm(armDeposit90);
                    deposit.setWrist(wrist90degree);
                    slides.setPidTarget(-20, 0.5);
                })
                .splineTo(new Vector2d(45, 24), Math.toRadians(0))
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    deposit.setFinger(zeroPixel);
                })
                .waitSeconds(0.5)
                .forward(2)
                .addTemporalMarker(() -> {
                    deposit.setWrist(wristTransfer);
                    deposit.setArm(armPreTransfer);
                    extendo.setState(retracted);
                    virtual4Bar.setClaw(clawClose);
                    virtual4Bar.setV4b(v4bTransfer);
                    slides.setPidTarget(0, 0.5);
                })
                .lineTo(new Vector2d(42, 3))
                .build();

        while(!isStarted()){
            location = bluePropProcessor.getLocation();
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
        }

    }
}
