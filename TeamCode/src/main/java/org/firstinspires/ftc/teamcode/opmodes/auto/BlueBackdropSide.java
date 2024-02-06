package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.armDeposit90;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.armPreTransfer;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.bothPixels;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.wrist90degree;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.wristTransfer;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.zeroPixel;
import static org.firstinspires.ftc.teamcode.subsystem.extendo.Extendo.Extension_States.closespike;
import static org.firstinspires.ftc.teamcode.subsystem.extendo.Extendo.Extension_States.farspike;
import static org.firstinspires.ftc.teamcode.subsystem.extendo.Extendo.Extension_States.midspike;
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
import org.firstinspires.ftc.teamcode.vision.NewBluePropProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name="\uD83D\uDC80\\blueBackdrop")
public class BlueBackdropSide extends LinearOpMode {
    private NewBluePropProcessor.Location location = MIDDLE;
    private NewBluePropProcessor bluePropProcessor;
    private VisionPortal visionPortal;
    Deposit deposit;
    Slides slides;
    Extendo extendo;
    Virtual4Bar virtual4Bar;

    Servo LeftHang;
    Servo RightHang;
    Servo drone;
    SampleMecanumDrive drive;

    @Override
    public void runOpMode(){
        drive = new SampleMecanumDrive(hardwareMap);
        bluePropProcessor = new NewBluePropProcessor(telemetry);
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), bluePropProcessor);
        Pose2d startpose = new Pose2d(14.75-8.25, 61.5, Math.toRadians(90));
        drive.setPoseEstimate(startpose);

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

        TrajectorySequence leftPurple = drive.trajectorySequenceBuilder(startpose)
                .UNSTABLE_addTemporalMarkerOffset(1,() -> {
                    deposit.setWrist(wrist90degree);
                    deposit.setArm(armDeposit90);
                })
                .lineToSplineHeading(new Pose2d(46.5, 40, Math.toRadians(180)))
                .addTemporalMarker(() -> deposit.setFinger(zeroPixel))
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> {
                    virtual4Bar.setV4b(0.97);
                    extendo.setState(closespike);
                    deposit.setWrist(wristTransfer);
                    deposit.setArm(armPreTransfer);
                })
                .strafeLeft(10)
                .addTemporalMarker(() -> virtual4Bar.setClaw(clawOpen))
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    virtual4Bar.setClaw(clawClose);
                    virtual4Bar.setV4b(v4bTransfer);
                    extendo.setState(retracted);
                })
                .strafeRight(25)
                .build();

        TrajectorySequence middlePurple = drive.trajectorySequenceBuilder(startpose)
                .UNSTABLE_addTemporalMarkerOffset(1,() -> {
                    deposit.setWrist(wrist90degree);
                    deposit.setArm(armDeposit90);
                })
                .lineToSplineHeading(new Pose2d(46.5, 32, Math.toRadians(180)))
                .addTemporalMarker(() -> deposit.setFinger(zeroPixel))
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    virtual4Bar.setV4b(0.97);
                    extendo.setState(midspike);
                    deposit.setWrist(wristTransfer);
                    deposit.setArm(armPreTransfer);
                })
                .strafeLeft(11)
                .addTemporalMarker(() -> virtual4Bar.setClaw(clawOpen))
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    virtual4Bar.setClaw(clawClose);
                    virtual4Bar.setV4b(v4bTransfer);
                    extendo.setState(retracted);
                })
                .strafeRight(35)
                .build();

        TrajectorySequence rightPurple = drive.trajectorySequenceBuilder(startpose)
                .UNSTABLE_addTemporalMarkerOffset(1,() -> {
                    deposit.setWrist(wrist90degree);
                    deposit.setArm(armDeposit90);
                })
                .lineToSplineHeading(new Pose2d(46.5, 26, Math.toRadians(180)))
                .addTemporalMarker(() -> deposit.setFinger(zeroPixel))
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> {
                    virtual4Bar.setV4b(0.97);
                    extendo.setState(farspike);
                    deposit.setWrist(wristTransfer);
                    deposit.setArm(armPreTransfer);
                })
                .waitSeconds(2)
                .strafeRight(2)
                .addTemporalMarker(() -> virtual4Bar.setClaw(clawOpen))
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    virtual4Bar.setClaw(clawClose);
                    virtual4Bar.setV4b(v4bTransfer);
                    extendo.setState(retracted);
                })
                .strafeRight(30)
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
