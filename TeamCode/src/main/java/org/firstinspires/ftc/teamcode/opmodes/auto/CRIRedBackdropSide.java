package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.subsystem.deposit.NewDeposit.armDeposit;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.NewDeposit.armPreTransfer;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.NewDeposit.wrist30degree;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.NewDeposit.wristTransfer;
import static org.firstinspires.ftc.teamcode.subsystem.extendo.Extendo.Extension_States.closespike;
import static org.firstinspires.ftc.teamcode.subsystem.extendo.Extendo.Extension_States.extended;
import static org.firstinspires.ftc.teamcode.subsystem.extendo.Extendo.Extension_States.farspike;
import static org.firstinspires.ftc.teamcode.subsystem.extendo.Extendo.Extension_States.midspike;
import static org.firstinspires.ftc.teamcode.subsystem.extendo.Extendo.Extension_States.retracted;
import static org.firstinspires.ftc.teamcode.subsystem.extendo.Extendo.Extension_States.second;
import static org.firstinspires.ftc.teamcode.vision.processors.PropProcessor.Location.MIDDLE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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

@Autonomous(name="RedBackdrop", preselectTeleOp = "TeleCRI")
public class CRIRedBackdropSide extends LinearOpMode {
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

        Pose2d startpose = new Pose2d(12.5, -61.5, Math.toRadians(-90));
        drive.setPoseEstimate(startpose);

        TrajectorySequence leftPurple = drive.trajectorySequenceBuilder(startpose)
                .lineToSplineHeading(new Pose2d(44.5, -25, Math.toRadians(-180)))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    deposit.setArm(armDeposit);
                    deposit.setWrist(wrist30degree);
                    extendo.setState(extended);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    intake.setIntake(1);
                    deposit.clawDrop();
                })
                .forward(4)
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    intake.setIntake(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    deposit.setWrist(wristTransfer);
                    deposit.setArm(armPreTransfer);
                    extendo.setState(retracted);
                })
                .strafeLeft(32)
                .back(12)
                .build();

        TrajectorySequence middlePurple = drive.trajectorySequenceBuilder(startpose)
                .lineToSplineHeading(new Pose2d(45, -32, Math.toRadians(-180)))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    deposit.setArm(armDeposit);
                    deposit.setWrist(wrist30degree);
                    extendo.setState(midspike);
                })
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    deposit.clawDrop();
                })
                .waitSeconds(0.5)
                .strafeRight(12)
                .addTemporalMarker(() -> {
                    intake.setIntake(1);
                })
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    intake.setIntake(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    deposit.setWrist(wristTransfer);
                    deposit.setArm(armPreTransfer);
                    extendo.setState(retracted);
                })
                .strafeLeft(35)
                .back(10)
                .build();

        TrajectorySequence rightPurple = drive.trajectorySequenceBuilder(startpose)
                .lineToSplineHeading(new Pose2d(46, -38, Math.toRadians(-180)))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    deposit.setArm(armDeposit);
                    deposit.setWrist(wrist30degree);
                    extendo.setState(closespike);
                })
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    deposit.clawDrop();
                })
                .waitSeconds(0.5)
                .forward(2)
                .strafeRight(8)
                .addTemporalMarker(() -> {
                    intake.setIntake(1);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    intake.setIntake(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    deposit.setWrist(wristTransfer);
                    deposit.setArm(armPreTransfer);
                    extendo.setState(retracted);
                })
                .strafeLeft(28)
                .back(12)
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

