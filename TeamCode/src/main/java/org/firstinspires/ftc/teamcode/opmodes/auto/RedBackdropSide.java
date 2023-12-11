package org.firstinspires.ftc.teamcode.opmodes.auto;

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
import org.firstinspires.ftc.teamcode.subsystem.intake.ActiveIntake;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.NewRedPropProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Mat;

@Autonomous(name="\uD83D\uDC80\\RedBackdrop")
public class RedBackdropSide extends LinearOpMode {
    private NewRedPropProcessor.Location location = MIDDLE;
    private NewRedPropProcessor redPropProcessor;
    private VisionPortal visionPortal;

    private DcMotor slide;
    private DcMotorEx intake;
    private Servo intakeservo, bucket;

    private ActiveIntake Intake;
    SampleMecanumDrive drive;

    @Override
    public void runOpMode(){
        drive = new SampleMecanumDrive(hardwareMap);
        redPropProcessor = new NewRedPropProcessor(telemetry);
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), redPropProcessor);
        Pose2d startpose = new Pose2d(10, -58, Math.toRadians(90));
        drive.setPoseEstimate(startpose);


        initHardware();

        TrajectorySequence leftPurple = drive.trajectorySequenceBuilder(startpose)
                .lineToSplineHeading(new Pose2d(15, -43, Math.toRadians(135)))
                .splineToConstantHeading(new Vector2d(10, -36), Math.toRadians(180))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    Intake.Intake(intake, 0.6);
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    Intake.Intake(intake, 0);
                })
                .addTemporalMarker(() -> {
                    myGoToHeightPOS(-250, 1);
                })
                .lineToSplineHeading(new Pose2d(56,-25, Math.toRadians(180)))
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
                .waitSeconds(0.5)
                .build();

        TrajectorySequence middlePurple = drive.trajectorySequenceBuilder(startpose)
                .forward(2)
                .build();

        TrajectorySequence rightPurple = drive.trajectorySequenceBuilder(startpose)
                .forward(2)
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
        intakeservo.setPosition(0.68);
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
