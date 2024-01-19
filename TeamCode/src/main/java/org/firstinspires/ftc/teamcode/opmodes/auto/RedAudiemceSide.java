package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.vision.NewRedPropProcessor.Location.MIDDLE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.intake.ActiveIntake;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.NewRedPropProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name="\uD83D\uDC80\\RedAudience")
public class RedAudiemceSide extends LinearOpMode {
    private NewRedPropProcessor.Location location = MIDDLE;
    private NewRedPropProcessor redPropProcessor;
    private VisionPortal visionPortal;

    private DcMotor slide;
    private DcMotorEx intake;
    private Servo intakeservo, bucket;

    NormalizedColorSensor colorSensor;

    private ActiveIntake Intake;
    SampleMecanumDrive drive;

    @Override
    public void runOpMode(){
        drive = new SampleMecanumDrive(hardwareMap);
        redPropProcessor = new NewRedPropProcessor(telemetry);
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), redPropProcessor);
        Pose2d startpose = new Pose2d(-38, -58.75, Math.toRadians(90));
        drive.setPoseEstimate(startpose);

        initHardware();

        TrajectorySequence leftPurple = drive.trajectorySequenceBuilder(startpose)
                .lineTo(new Vector2d(-48.5, -42))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    Intake.Intake(intake, 0.4);
                })
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    Intake.Intake(intake, 0);
                })
                .strafeRight(8)
                .lineToSplineHeading(new Pose2d(-43, -6, Math.toRadians(180)))
                .back(40)
                .waitSeconds(5)
                .back(24)
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    myGoToHeightPOS(-270, 1);
                })
                .lineToSplineHeading(new Pose2d(55.5,-23.5, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    bucket.setPosition(0.95);
                })
                .waitSeconds(0.4)
                .forward(5)
                //.strafeRight(8)
                .addTemporalMarker(() -> {
                    bucket.setPosition(0.47);
                })
                .splineToConstantHeading(new Vector2d(60, -8), Math.toRadians(0))
                .addTemporalMarker(() -> {
                    myGoToHeightPOS(0, 1);
                })
                .waitSeconds(1)
                .build();

        TrajectorySequence middlePurple = drive.trajectorySequenceBuilder(startpose)
                .lineTo(new Vector2d(-30, -32))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    Intake.Intake(intake, 0.4);
                })
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    Intake.Intake(intake, 0);
                })
                .strafeLeft(24)
                .lineToSplineHeading(new Pose2d(-43, -6, Math.toRadians(180)))
                .back(40)
                .waitSeconds(10)
                .back(24)
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    myGoToHeightPOS(-270, 1);
                })
                .lineToSplineHeading(new Pose2d(55.5,-34, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    bucket.setPosition(0.95);
                })
                .waitSeconds(0.4)
                .forward(5)
                //.strafeRight(8)
                .addTemporalMarker(() -> {
                    bucket.setPosition(0.47);
                })
                .splineToConstantHeading(new Vector2d(60, -8), Math.toRadians(0))
                .addTemporalMarker(() -> {
                    myGoToHeightPOS(0, 1);
                })
                .waitSeconds(1)
                .build();

        TrajectorySequence rightPurple = drive.trajectorySequenceBuilder(startpose)
                .lineToSplineHeading(new Pose2d(-35, -43, Math.toRadians(45)))
                .splineToConstantHeading(new Vector2d(-30, -39), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    Intake.Intake(intake, 0.4);
                })
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    Intake.Intake(intake, 0);
                })
                .back(2)
                .lineToSplineHeading(new Pose2d(-43, -6, Math.toRadians(180)))
                .back(40)
                .waitSeconds(5)
                .back(24)
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    myGoToHeightPOS(-270, 1);
                })
                .lineToSplineHeading(new Pose2d(55.5,-37, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    bucket.setPosition(0.95);
                })
                .waitSeconds(0.4)
                .forward(7)
                //.strafeRight(8)
                .addTemporalMarker(() -> {
                    bucket.setPosition(0.47);
                })
                .splineToConstantHeading(new Vector2d(60, -8), Math.toRadians(0))
                .addTemporalMarker(() -> {
                    myGoToHeightPOS(0, 1);
                })
                .waitSeconds(1)
                .build();

        while(!isStarted()){
            location = redPropProcessor.getLocation();
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
    public void initHardware(){
        slide = hardwareMap.get(DcMotor.class, "slide");
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeservo = hardwareMap.servo.get("intakeservo");
        intake = (DcMotorEx) hardwareMap.dcMotor.get("intake");
        Intake = new ActiveIntake();
        bucket = hardwareMap.servo.get("bucket");
        double power;
        bucket.setPosition(0.47);
        intakeservo.setPosition(0.65);
    }
    public void myGoToHeightPOS(int slidePOS, double motorPower) {
        //to find slide position and motor position
        telemetry.addData("leftSlidePOS", slide.getCurrentPosition());
        telemetry.addData("motorPower", motorPower);
        telemetry.update();
        //base encoder code
        slide.setTargetPosition(slidePOS);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(motorPower);
    }



}
