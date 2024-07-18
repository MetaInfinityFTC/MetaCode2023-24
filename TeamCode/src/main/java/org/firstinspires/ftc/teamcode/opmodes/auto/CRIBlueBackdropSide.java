package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.bothPixels;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.zeroPixel;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.NewDeposit.armDeposit;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.NewDeposit.armPreTransfer;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.NewDeposit.wrist30degree;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.NewDeposit.wristTransfer;
import static org.firstinspires.ftc.teamcode.subsystem.extendo.Extendo.Extension_States.closespike;
import static org.firstinspires.ftc.teamcode.subsystem.extendo.Extendo.Extension_States.extended;
import static org.firstinspires.ftc.teamcode.subsystem.extendo.Extendo.Extension_States.farspike;
import static org.firstinspires.ftc.teamcode.subsystem.extendo.Extendo.Extension_States.midspike;
import static org.firstinspires.ftc.teamcode.subsystem.extendo.Extendo.Extension_States.retracted;
import static org.firstinspires.ftc.teamcode.subsystem.intake.Virtual4Bar.clawClose;
import static org.firstinspires.ftc.teamcode.subsystem.intake.Virtual4Bar.clawOpen;
import static org.firstinspires.ftc.teamcode.subsystem.intake.Virtual4Bar.v4bPreTransfer;
import static org.firstinspires.ftc.teamcode.subsystem.intake.Virtual4Bar.v4bTransfer;
import static org.firstinspires.ftc.teamcode.vision.processors.PropProcessor.Location.MIDDLE;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.Drone;
import org.firstinspires.ftc.teamcode.subsystem.Hang;
import org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit;
import org.firstinspires.ftc.teamcode.subsystem.deposit.NewDeposit;
import org.firstinspires.ftc.teamcode.subsystem.deposit.Slides;
import org.firstinspires.ftc.teamcode.subsystem.extendo.Extendo;
import org.firstinspires.ftc.teamcode.subsystem.intake.NewActiveIntake;
import org.firstinspires.ftc.teamcode.subsystem.intake.Virtual4Bar;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.processors.BluePropProcessor;
import org.firstinspires.ftc.teamcode.vision.processors.PropProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name="blueBackdrop", preselectTeleOp = "TeleCRI")
public class CRIBlueBackdropSide extends LinearOpMode {
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
    public void runOpMode(){
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

        Pose2d startpose = new Pose2d(14.75-8.25, 61.5, Math.toRadians(90));
        drive.setPoseEstimate(startpose);

        TrajectorySequence leftPurple = drive.trajectorySequenceBuilder(startpose)
                .lineToSplineHeading(new Pose2d(45, 38, Math.toRadians(-180)))
                .addTemporalMarker(() -> {
                    deposit.setArm(armDeposit);
                    deposit.setWrist(wrist30degree);
                    extendo.setState(closespike);
                })
                .waitSeconds(1)
                .forward(2)
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    deposit.clawDrop();
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    setPidTarget(0, 0.5);
                    extendo.setState(retracted);
                    deposit.setWrist(wristTransfer);
                    deposit.setArm(armPreTransfer);
                })
                .strafeLeft(10)
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    intake.setIntake(1);
                })
                .waitSeconds(0.3)
                .strafeRight(25)
                .build();

        TrajectorySequence middlePurple = drive.trajectorySequenceBuilder(startpose)
                .lineToSplineHeading(new Pose2d(45, 29.5, Math.toRadians(-180)))
                .addTemporalMarker(() -> {
                    deposit.setArm(armDeposit);
                    deposit.setWrist(wrist30degree);
                    extendo.setState(closespike);
                })
                .waitSeconds(1.0)
                .addTemporalMarker(() -> {
                    deposit.clawDrop();
                })
                .forward(2)
                .waitSeconds(1.0)
                .strafeLeft(4)
                .addTemporalMarker(() -> {
                    intake.setIntake(1);
                })
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    setPidTarget(0, 0.5);
                    extendo.setState(retracted);
                    deposit.setWrist(wristTransfer);
                    deposit.setArm(armPreTransfer);
                })
                .strafeRight(32)
                .build();

        TrajectorySequence rightPurple = drive.trajectorySequenceBuilder(startpose)
                .lineToSplineHeading(new Pose2d(45, 24, Math.toRadians(-180)))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    deposit.setArm(armDeposit);
                    deposit.setWrist(wrist30degree);
                    extendo.setState(extended);
                })
                .waitSeconds(1.0)
                .addTemporalMarker(() -> {
                    deposit.clawDrop();
                })
                .waitSeconds(0.5)
                .forward(2)
                .addTemporalMarker(() -> {
                    intake.setIntake(1);
                })
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    setPidTarget(0, 0.5);
                    extendo.setState(retracted);
                    deposit.setWrist(wristTransfer);
                    deposit.setArm(armPreTransfer);
                })
                .strafeRight(33)
                .back(10)
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
