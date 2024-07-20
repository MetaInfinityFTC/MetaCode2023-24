package org.firstinspires.ftc.teamcode.opmodes.auto;


import static org.firstinspires.ftc.teamcode.subsystem.deposit.NewDeposit.armDeposit;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.NewDeposit.armPreTransfer;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.NewDeposit.wrist30degree;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.NewDeposit.wristTransfer;
import static org.firstinspires.ftc.teamcode.subsystem.extendo.Extendo.Extension_States.closespike;
import static org.firstinspires.ftc.teamcode.subsystem.extendo.Extendo.Extension_States.extended;
import static org.firstinspires.ftc.teamcode.subsystem.extendo.Extendo.Extension_States.midspike;
import static org.firstinspires.ftc.teamcode.subsystem.extendo.Extendo.Extension_States.retracted;
import static org.firstinspires.ftc.teamcode.subsystem.extendo.Extendo.Extension_States.second;
import static org.firstinspires.ftc.teamcode.vision.processors.PropProcessor.Location.MIDDLE;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.Drone;
import org.firstinspires.ftc.teamcode.subsystem.Hang;
import org.firstinspires.ftc.teamcode.subsystem.deposit.NewDeposit;
import org.firstinspires.ftc.teamcode.subsystem.deposit.Slides;
import org.firstinspires.ftc.teamcode.subsystem.extendo.Extendo;
import org.firstinspires.ftc.teamcode.subsystem.intake.NewActiveIntake;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.processors.PropProcessor;
import org.firstinspires.ftc.teamcode.vision.processors.RedPropProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name="RedMiddle", preselectTeleOp = "TeleCRI")
public class CRIRedMiddleSide extends LinearOpMode {
    private PropProcessor.Location location = MIDDLE;
    private RedPropProcessor redPropProcessor;

    private VisionPortal visionPortal;
    NewDeposit deposit;
    Slides slides;
    Extendo extendo;
    Hang hang;
    NewActiveIntake intake;

    Drone drone;
    SampleMecanumDrive drive;

    DcMotor left;
    DcMotor right;

    boolean transferring = false;

    @Override
    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);
        redPropProcessor = new RedPropProcessor(telemetry);
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), redPropProcessor);

        extendo = new Extendo(hardwareMap);
        slides = new Slides(hardwareMap);
        deposit = new NewDeposit(hardwareMap);
        hang = new Hang(hardwareMap);
        drone = new Drone(hardwareMap);
        intake = new NewActiveIntake(hardwareMap);

        deposit.setArm(armPreTransfer);
        deposit.clawGrab();
        deposit.setWrist(wristTransfer);

        hang.down();

        drone.init();

        Pose2d startpose = new Pose2d(-36, -58, Math.toRadians(-90));
        drive.setPoseEstimate(startpose);

        TrajectorySequence leftPurple = drive.trajectorySequenceBuilder(startpose)
                .back(5)
                .turn(Math.toRadians(205))
                .waitSeconds(1.0)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    extendo.setState(second);
                })
                .waitSeconds(1.5)
                .addTemporalMarker(() -> {
                    intake.setIntake(1);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    intake.setIntake(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    extendo.setState(retracted);
                })
                .lineToSplineHeading(new Pose2d(-20, -47, Math.toRadians(-180)))
                .lineTo(new Vector2d(20, -47))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    deposit.setArm(armDeposit);
                    deposit.setWrist(wrist30degree);
                })
                .splineTo(new Vector2d(43, -30), Math.toRadians(0))
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    deposit.clawDrop();
                })
                .waitSeconds(0.5)
                .forward(2)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    extendo.setState(retracted);
                    deposit.setWrist(wristTransfer);
                    deposit.setArm(armPreTransfer);
                })
                .forward(4)
                .lineTo(new Vector2d(42, -10))
                .build();

        TrajectorySequence middlePurple = drive.trajectorySequenceBuilder(startpose)
                .back(4)
                .turn(Math.toRadians(180))
                .strafeLeft(8)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    extendo.setState(second);
                })
                .waitSeconds(1.5)
                .addTemporalMarker(() -> {
                    intake.setIntake(1);
                })
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    intake.setIntake(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    extendo.setState(retracted);
                })
                .lineToSplineHeading(new Pose2d(-20, -35, Math.toRadians(180)))
                .lineTo(new Vector2d(10, -47))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    deposit.setArm(armDeposit);
                    deposit.setWrist(wrist30degree);
                })
                .lineToSplineHeading(new Pose2d(20, -35, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                })
                .splineToConstantHeading(new Vector2d(42, -30), Math.toRadians(0))
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    deposit.clawDrop();
                })
                .waitSeconds(0.5)
                .forward(4)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    extendo.setState(retracted);
                    deposit.setWrist(wristTransfer);
                    deposit.setArm(armPreTransfer);
                })
                .lineTo(new Vector2d(42, -10))
                .build();

        TrajectorySequence rightPurple = drive.trajectorySequenceBuilder(startpose)
                .back(5)
                .turn(Math.toRadians(155))
                .waitSeconds(1.0)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    extendo.setState(second);
                })
                .waitSeconds(1.5)
                .addTemporalMarker(() -> {
                    intake.setIntake(1);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    intake.setIntake(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    extendo.setState(retracted);
                })
                .lineToSplineHeading(new Pose2d(-20, -47, Math.toRadians(-180)))
                .lineTo(new Vector2d(20, -47))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    deposit.setArm(armDeposit);
                    deposit.setWrist(wrist30degree);
                })
                .splineTo(new Vector2d(43, -40), Math.toRadians(0))
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    deposit.clawDrop();
                })
                .waitSeconds(0.5)
                .forward(4)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    extendo.setState(retracted);
                    deposit.setWrist(wristTransfer);
                    deposit.setArm(armPreTransfer);
                })
                .lineTo(new Vector2d(42, -10))
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
        while (opModeIsActive()) {
            drive.update();
            extendo.update();

        }

    }
}
