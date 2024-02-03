package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.armDeposit90;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.armPreTransfer;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.bothPixels;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.wrist90degree;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.wristTransfer;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.zeroPixel;
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

        LeftHang.setPosition(0.2);
        RightHang.setPosition(0.9);

        drone.setPosition(0);

        Pose2d startpose = new Pose2d(-33, 58, Math.toRadians(90));
        drive.setPoseEstimate(startpose);

        TrajectorySequence leftPurple = drive.trajectorySequenceBuilder(startpose)
                .lineToSplineHeading(new Pose2d(-45, 12, Math.toRadians(90)))
                //claw goes out here
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    virtual4Bar.setV4b(0.97);
                })
                .strafeRight(5)
                .turn(Math.toRadians(-45))
                .forward(3)
                //add claw drop
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    virtual4Bar.setClaw(clawOpen);
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    virtual4Bar.setClaw(clawClose);
                    virtual4Bar.setV4b(v4bTransfer);
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    deposit.setArm(0.8);
                })
                .back(10)
                .lineToSplineHeading(new Pose2d(-20, 4, Math.toRadians(180)))
                //claw in
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    virtual4Bar.setClaw(clawClose);
                    virtual4Bar.setV4b(v4bTransfer);
                })
                //.waitSeconds(20) -> timer for other team to finish auto
                .lineTo(new Vector2d(20, 12))
                //depo arm out now
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    deposit.setArm(armDeposit90);
                    deposit.setWrist(wrist90degree);
                })
                .splineTo(new Vector2d(44, 24), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    deposit.setFinger(zeroPixel);
                })
                .waitSeconds(1)
                //place yellow pixel, depo arm in, etc.
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    deposit.setArm(armPreTransfer);
                    deposit.setWrist(wristTransfer);
                    deposit.setFinger(zeroPixel);
                })
                .strafeLeft(30)
                .build();

        TrajectorySequence middlePurple = drive.trajectorySequenceBuilder(startpose)
                .lineToSplineHeading(new Pose2d(-45, 12, Math.toRadians(90)))
                //claw goes out here
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    virtual4Bar.setV4b(0.97);
                })
                .strafeRight(5)
                .turn(Math.toRadians(45))
                .lineTo(new Vector2d(-25, 9))
                //add claw drop
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    virtual4Bar.setClaw(clawOpen);
                })
                .lineToSplineHeading(new Pose2d(-20, 4, Math.toRadians(180)))
                //claw in
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    virtual4Bar.setClaw(clawClose);
                    virtual4Bar.setV4b(v4bTransfer);
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    deposit.setArm(0.8);
                })
                //.waitSeconds(20) -> timer for other team to finish auto
                .lineTo(new Vector2d(20, 12))
                //depo arm out now
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    deposit.setArm(armDeposit90);
                    deposit.setWrist(wrist90degree);
                })
                .splineTo(new Vector2d(45, 32), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    deposit.setFinger(zeroPixel);
                })
                .waitSeconds(1)
                //place yellow pixel, depo arm in, etc.
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    deposit.setArm(armPreTransfer);
                    deposit.setWrist(wristTransfer);
                    deposit.setFinger(zeroPixel);
                })
                .strafeLeft(20)
                .build();

        TrajectorySequence rightPurple = drive.trajectorySequenceBuilder(startpose)
                .lineTo(new Vector2d(-45, 12))
                //claw goes out here
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    virtual4Bar.setV4b(0.9);
                })
                .strafeRight(5)
                .turn(Math.toRadians(45))
                //add claw drop
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    virtual4Bar.setClaw(clawOpen);
                })
                .waitSeconds(1)
                .lineToSplineHeading(new Pose2d(-20, 12, Math.toRadians(180)))
                //claw in
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    virtual4Bar.setClaw(clawClose);
                    virtual4Bar.setV4b(v4bTransfer);
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    deposit.setArm(0.8);
                })
                //.waitSeconds(20) -> timer for other team to finish auto
                .lineTo(new Vector2d(20, 4))
                //depo arm out now
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    deposit.setArm(armDeposit90);
                    deposit.setWrist(wrist90degree);
                })
                .splineTo(new Vector2d(45, 38), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    deposit.setFinger(zeroPixel);
                })
                .waitSeconds(1)
                //place yellow pixel, depo arm in, etc.
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    deposit.setArm(armPreTransfer);
                    deposit.setWrist(wristTransfer);
                    deposit.setFinger(zeroPixel);
                })
                .strafeLeft(10)
                .build();

        while(!isStarted()){
            location = bluePropProcessor.getLocation();
            telemetry.update();
        }
        waitForStart();
        switch(location){
            case LEFT:
                drive.followTrajectorySequence(leftPurple);
                break;
            case MIDDLE:
                drive.followTrajectorySequence(middlePurple);
                break;
            case RIGHT:
                drive.followTrajectorySequence(rightPurple);
                break;
        }


    }
}
