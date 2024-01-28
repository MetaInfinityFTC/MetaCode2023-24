package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.armDeposit90;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.armPreTransfer;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.armTransfer;
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
import static org.firstinspires.ftc.teamcode.subsystem.intake.Virtual4Bar.v4bTransfer;
import static org.firstinspires.ftc.teamcode.vision.NewRedPropProcessor.Location.LEFT;
import static org.firstinspires.ftc.teamcode.vision.NewRedPropProcessor.Location.MIDDLE;

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
import org.firstinspires.ftc.teamcode.vision.NewRedPropProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Mat;

@Autonomous(name="\uD83D\uDC80\\RedBackdrop", preselectTeleOp = "DaTele")
public class RedBackdropSide extends LinearOpMode {
    private NewRedPropProcessor.Location location = LEFT;
    private NewRedPropProcessor redPropProcessor;
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
        redPropProcessor = new NewRedPropProcessor(telemetry);
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
        virtual4Bar.setV4b(v4bTransfer);

        LeftHang.setPosition(0.2);
        RightHang.setPosition(0.9);

        drone.setPosition(0);

        Pose2d startpose = new Pose2d(14.75, -61.5, Math.toRadians(-90));
        drive.setPoseEstimate(startpose);

        TrajectorySequence leftPurple = drive.trajectorySequenceBuilder(startpose)
                .UNSTABLE_addTemporalMarkerOffset(1,() -> {
                    deposit.setWrist(wrist90degree);
                    deposit.setArm(armDeposit90);
                })
                .lineToSplineHeading(new Pose2d(45, -30, Math.toRadians(-180)))
                .addTemporalMarker(() -> deposit.setFinger(zeroPixel))
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> {
                    virtual4Bar.setV4b(0.9);
                    extendo.setState(farspike);
                    deposit.setWrist(wristTransfer);
                    deposit.setArm(armPreTransfer);
                })
                .waitSeconds(2)
                .addTemporalMarker(() -> virtual4Bar.setClaw(clawOpen))
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    virtual4Bar.setClaw(clawClose);
                    virtual4Bar.setV4b(v4bTransfer);
                    extendo.setState(retracted);
                })
                .strafeLeft(30)
                .build();

        TrajectorySequence middlePurple = drive.trajectorySequenceBuilder(startpose)
                .UNSTABLE_addTemporalMarkerOffset(1,() -> {
                    deposit.setWrist(wrist90degree);
                    deposit.setArm(armDeposit90);
                })
                .lineToSplineHeading(new Pose2d(45, -34.5, Math.toRadians(-180)))
                .addTemporalMarker(() -> deposit.setFinger(zeroPixel))
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> {
                    virtual4Bar.setV4b(0.9);
                    extendo.setState(midspike);
                    deposit.setWrist(wristTransfer);
                    deposit.setArm(armPreTransfer);
                })
                .strafeRight(8)
                .addTemporalMarker(() -> virtual4Bar.setClaw(clawOpen))
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    virtual4Bar.setClaw(clawClose);
                    virtual4Bar.setV4b(v4bTransfer);
                    extendo.setState(retracted);
                })
                .strafeLeft(30)
                .build();

        TrajectorySequence rightPurple = drive.trajectorySequenceBuilder(startpose)
                .UNSTABLE_addTemporalMarkerOffset(1,() -> {
                    deposit.setWrist(wrist90degree);
                    deposit.setArm(armDeposit90);
                })
                .lineToSplineHeading(new Pose2d(45, -42.5, Math.toRadians(-180)))
                .addTemporalMarker(() -> deposit.setFinger(zeroPixel))
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> {
                    virtual4Bar.setV4b(0.9);
                    extendo.setState(closespike);
                    deposit.setWrist(wristTransfer);
                    deposit.setArm(armPreTransfer);
                })
                .strafeRight(12)
                .addTemporalMarker(() -> virtual4Bar.setClaw(clawOpen))
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    virtual4Bar.setClaw(clawClose);
                    virtual4Bar.setV4b(v4bTransfer);
                    extendo.setState(retracted);
                })
                .strafeLeft(30)
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
        }
    }
}

