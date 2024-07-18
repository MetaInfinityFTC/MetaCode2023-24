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
import static org.firstinspires.ftc.teamcode.vision.processors.PropProcessor.Location.MIDDLE;


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
import org.firstinspires.ftc.teamcode.subsystem.AbstractedMachineRTP;
import org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit;
import org.firstinspires.ftc.teamcode.subsystem.deposit.Slides;
import org.firstinspires.ftc.teamcode.subsystem.extendo.Extendo;
import org.firstinspires.ftc.teamcode.subsystem.intake.Virtual4Bar;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.processors.PropProcessor;
import org.firstinspires.ftc.teamcode.vision.processors.RedPropProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name="RedMiddle", preselectTeleOp = "TeleCRI")
public class CRIRedMiddleSide extends LinearOpMode {
    private PropProcessor.Location location = MIDDLE;
    private RedPropProcessor redPropProcessor;

    SampleMecanumDrive drive;

    @Override
    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);
        redPropProcessor = new RedPropProcessor(telemetry);

        Pose2d startpose = new Pose2d(-36, -58, Math.toRadians(-90));
        drive.setPoseEstimate(startpose);

        TrajectorySequence leftPurple = drive.trajectorySequenceBuilder(startpose)
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                })
                .lineToSplineHeading(new Pose2d(-51.5, -8.5, Math.toRadians(-90)))
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                })
                .lineToSplineHeading(new Pose2d(-20, -6, Math.toRadians(-180)))
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                })
                .waitSeconds(12)
                .lineTo(new Vector2d(20, -4))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                })
                .splineTo(new Vector2d(43, -21), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                })
                .waitSeconds(1)
                .forward(3)
                .addTemporalMarker(() -> {
                })
                .lineTo(new Vector2d(42, -3))
                .build();

        TrajectorySequence middlePurple = drive.trajectorySequenceBuilder(startpose)
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
                })
                .lineToSplineHeading(new Pose2d(-67, -19, Math.toRadians(-90)))                //claw goes out here
                .turn(Math.toRadians(90))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                })
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                })
                .lineTo(new Vector2d(-55,-2))
                .lineToSplineHeading(new Pose2d(-20, -2, Math.toRadians(180)))
                .strafeLeft(4)
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                })
                .waitSeconds(12)
                .lineTo(new Vector2d(10, -10))
                .lineToSplineHeading(new Pose2d(20, -10, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                })
                .splineTo(new Vector2d(43, -29), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                })
                .waitSeconds(1)
                .forward(4)
                .addTemporalMarker(() -> {
                })
                .lineTo(new Vector2d(42, -3))
                .build();

        TrajectorySequence rightPurple = drive.trajectorySequenceBuilder(startpose)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                })
                .lineToSplineHeading(new Pose2d(-46, -11, Math.toRadians(-90)))
                .strafeLeft(5)
                .turn(Math.toRadians(45))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                })
                .back(10)
                .lineToSplineHeading(new Pose2d(-20, -4, Math.toRadians(-180)))
                .waitSeconds(12)
                .lineTo(new Vector2d(20, -12))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                })
                .splineTo(new Vector2d(43, -37), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                })
                .waitSeconds(1)
                .forward(2)
                .addTemporalMarker(() -> {
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
        }
    }
}
