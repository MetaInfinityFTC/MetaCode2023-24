package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.armDeposit90;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.armPreTransfer;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.wrist90degree;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.wristTransfer;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.zeroPixel;
import static org.firstinspires.ftc.teamcode.subsystem.extendo.Extendo.Extension_States.retracted;
import static org.firstinspires.ftc.teamcode.subsystem.intake.Virtual4Bar.clawClose;
import static org.firstinspires.ftc.teamcode.subsystem.intake.Virtual4Bar.clawOpen;
import static org.firstinspires.ftc.teamcode.subsystem.intake.Virtual4Bar.v4bTransfer;
import static org.firstinspires.ftc.teamcode.vision.processors.PropProcessor.Location.MIDDLE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.processors.BluePropProcessor;
import org.firstinspires.ftc.teamcode.vision.processors.PropProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name="CRIBlueMiddle")
public class CRIBlueMiddleSide extends LinearOpMode {
    private PropProcessor.Location location = MIDDLE;
    private BluePropProcessor bluePropProcessor;

    SampleMecanumDrive drive;

    @Override
    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);
        bluePropProcessor = new BluePropProcessor(telemetry);

        Pose2d startpose = new Pose2d(-36, 58, Math.toRadians(90));
        drive.setPoseEstimate(startpose);
        TrajectorySequence leftPurple = drive.trajectorySequenceBuilder(startpose)
                .UNSTABLE_addTemporalMarkerOffset(1.0, () -> {
                })
                // .lineToSplineHeading(new Pose2d(-42.5, 11, Math.toRadians(90)))
                .back(5)
                .turn(Math.toRadians(25))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                })
                .back(5)
                .lineToSplineHeading(new Pose2d(-20, 47, Math.toRadians(-180)))
                .lineTo(new Vector2d(20, 47))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                })
                .splineTo(new Vector2d(43, 40), Math.toRadians(0))
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                })
                .waitSeconds(1)
                .forward(2)
                .addTemporalMarker(() -> {
                })
                .forward(4)
                .lineTo(new Vector2d(42, 10))
                .build();

        TrajectorySequence middlePurple = drive.trajectorySequenceBuilder(startpose)
                .UNSTABLE_addTemporalMarkerOffset(1.0, () -> {
                })
                .back(5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                })
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                })
                .lineToSplineHeading(new Pose2d(-20, 47, Math.toRadians(180)))
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                })
                .lineTo(new Vector2d(10, 47))
                .lineToSplineHeading(new Pose2d(20, 47, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                })
                .splineTo(new Vector2d(43, 35), Math.toRadians(0))
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                })
                .waitSeconds(0.5)
                .forward(4)
                .addTemporalMarker(() -> {
                })
                .lineTo(new Vector2d(42, 10))
                .build();

        TrajectorySequence rightPurple = drive.trajectorySequenceBuilder(startpose)
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                })
                .back(5)
                .turn(Math.toRadians(-25))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                })
                .lineToSplineHeading(new Pose2d(-20, 47, Math.toRadians(-180)))
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                })
                .lineTo(new Vector2d(20, 47))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                })
                .splineTo(new Vector2d(43, 30), Math.toRadians(0))
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                })
                .waitSeconds(0.5)
                .forward(4)
                .addTemporalMarker(() -> {
                })
                .lineTo(new Vector2d(42, 10))
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
        }
    }
}
