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

@Autonomous(name="\uD83D\uDC80\\RedBackdrop ")
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

        initHardware();

        TrajectorySequence leftPurple = drive.trajectorySequenceBuilder(new Pose2d(10, -58, Math.toRadians(90)))
                .splineTo(new Vector2d(10, -38), Math.toRadians(135))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    Intake.Intake(intake, -0.5);
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    myGoToHeightPOS(200, 1);
                })
                .lineToSplineHeading(new Pose2d(50,-29, Math.toRadians(180)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    bucket.setPosition(0.84);
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    bucket.setPosition(0.47);
                })
                .strafeRight(5)
                .splineToConstantHeading(new Vector2d(60, -12), Math.toRadians(0))
                .build();

        TrajectorySequence middlePurple = drive.trajectorySequenceBuilder(new Pose2d(10, -58, Math.toRadians(90)))
                .forward(40)
                .waitSeconds(2)
                .turn(Math.toRadians(-90))
                .forward(35)
                .build();

        TrajectorySequence rightPurple = drive.trajectorySequenceBuilder(new Pose2d(10, -58, Math.toRadians(90)))
                .strafeRight(10)
                .forward(20)
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
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeservo = hardwareMap.servo.get("intakeservo");
        intake = (DcMotorEx) hardwareMap.dcMotor.get("intake");
        Intake = new ActiveIntake();
        bucket = hardwareMap.servo.get("bucket");
        double power;
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
