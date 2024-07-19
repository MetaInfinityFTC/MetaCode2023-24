package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.subsystem.deposit.NewDeposit.armDeposit;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.NewDeposit.armPreTransfer;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.NewDeposit.wrist30degree;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.NewDeposit.wristTransfer;
import static org.firstinspires.ftc.teamcode.subsystem.extendo.Extendo.Extension_States.midspike;
import static org.firstinspires.ftc.teamcode.subsystem.extendo.Extendo.Extension_States.retracted;
import static org.firstinspires.ftc.teamcode.vision.processors.PropProcessor.Location.MIDDLE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.Drone;
import org.firstinspires.ftc.teamcode.subsystem.Hang;
import org.firstinspires.ftc.teamcode.subsystem.deposit.NewDeposit;
import org.firstinspires.ftc.teamcode.subsystem.deposit.Slides;
import org.firstinspires.ftc.teamcode.subsystem.extendo.Extendo;
import org.firstinspires.ftc.teamcode.subsystem.intake.NewActiveIntake;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.processors.BluePropProcessor;
import org.firstinspires.ftc.teamcode.vision.processors.PropProcessor;

@Autonomous(name="CRIBlueMiddle", preselectTeleOp = "TeleCRI")
public class CRIBlueMiddleSide extends LinearOpMode {
    private PropProcessor.Location location = MIDDLE;
    private BluePropProcessor bluePropProcessor;
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
        bluePropProcessor = new BluePropProcessor(telemetry);

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

        Pose2d startpose = new Pose2d(-36, 58, Math.toRadians(90));
        drive.setPoseEstimate(startpose);

        TrajectorySequence leftPurple = drive.trajectorySequenceBuilder(startpose)
                .back(5)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    extendo.setState(midspike);
                })
                .turn(Math.toRadians(25))
                .waitSeconds(0.5)
                .addTemporalMarker(0, () -> {
                    intake.setIntake(1);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(0, () -> {
                    intake.setIntake(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    extendo.setState(retracted);
                })
                .lineToSplineHeading(new Pose2d(-20, 47, Math.toRadians(-180)))
                .lineTo(new Vector2d(20, 47))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    deposit.setArm(armDeposit);
                    deposit.setWrist(wrist30degree);
                })
                .splineTo(new Vector2d(43, 40), Math.toRadians(0))
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    deposit.clawDrop();
                })
                .waitSeconds(0.5)
                .forward(2)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    setPidTarget(0, 0.5);
                    extendo.setState(retracted);
                    deposit.setWrist(wristTransfer);
                    deposit.setArm(armPreTransfer);
                })
                .forward(4)
                .lineTo(new Vector2d(42, 10))
                .build();

        TrajectorySequence middlePurple = drive.trajectorySequenceBuilder(startpose)
                .back(5)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    extendo.setState(midspike);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(0, () -> {
                    intake.setIntake(1);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(0, () -> {
                    intake.setIntake(0);
                })
                .lineToSplineHeading(new Pose2d(-20, 47, Math.toRadians(180)))
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    extendo.setState(retracted);
                })
                .lineTo(new Vector2d(10, 47))
                .lineToSplineHeading(new Pose2d(20, 47, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    deposit.setArm(armDeposit);
                    deposit.setWrist(wrist30degree);
                })
                .splineTo(new Vector2d(43, 35), Math.toRadians(0))
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    deposit.clawDrop();
                })
                .waitSeconds(0.5)
                .forward(4)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    setPidTarget(0, 0.5);
                    extendo.setState(retracted);
                    deposit.setWrist(wristTransfer);
                    deposit.setArm(armPreTransfer);
                })
                .lineTo(new Vector2d(42, 10))
                .build();

        TrajectorySequence rightPurple = drive.trajectorySequenceBuilder(startpose)
                .back(5)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    extendo.setState(midspike);
                })
                .turn(Math.toRadians(-25))
                .waitSeconds(0.5)
                .addTemporalMarker(0, () -> {
                    intake.setIntake(1);
                })
                .waitSeconds(0.5)
                .addTemporalMarker(0, () -> {
                    intake.setIntake(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    extendo.setState(retracted);
                })
                .lineToSplineHeading(new Pose2d(-20, 47, Math.toRadians(-180)))
                .lineTo(new Vector2d(20, 47))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    deposit.setArm(armDeposit);
                    deposit.setWrist(wrist30degree);
                })
                .splineTo(new Vector2d(43, 30), Math.toRadians(0))
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    deposit.clawDrop();
                })
                .waitSeconds(0.5)
                .forward(4)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    setPidTarget(0, 0.5);
                    extendo.setState(retracted);
                    deposit.setWrist(wristTransfer);
                    deposit.setArm(armPreTransfer);
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
