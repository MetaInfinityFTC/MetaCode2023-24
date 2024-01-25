package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Blinkdin;
import org.firstinspires.ftc.teamcode.subsystem.intake.ActiveIntake;

@Config
@Disabled
@TeleOp(name = "\uD83D\uDC80\\tele")
public class AntiquatedTeleOp extends LinearOpMode {
    //get our analog input from the hardwareMap
    AnalogInput analogInput;// = hardwareMap.get(AnalogInput.class, "miniaxon");
    DcMotorEx intake;
    Servo intakeservo;
    Servo bucket;

    ActiveIntake Intake;

    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;
    DcMotorEx slide;

    Blinkdin led;

    // get the voltage of our analog line
// divide by 3.3 (the max voltage) to get a value between 0 and 1
// multiply by 360 to convert it to 0 to 360 degrees
    double position;

    public void runOpMode() {
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        slide = (DcMotorEx) hardwareMap.dcMotor.get("slide");

        Intake = new ActiveIntake();

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        intakeservo = hardwareMap.servo.get("intakeservo");
        analogInput = hardwareMap.analogInput.get("miniaxon");
        intake = (DcMotorEx) hardwareMap.dcMotor.get("intake");

        bucket = hardwareMap.servo.get("bucket");

        Intake.initintake(intake);

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        bucket.setPosition(0.47);
        intakeservo.setPosition(0.82);

        led = new Blinkdin(hardwareMap.get(RevBlinkinLedDriver.class, "led"));

        double power;

        led.changePattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_LAVA_PALETTE);
        led.update();

        waitForStart();

        while (opModeIsActive()) {
            //set intake linkage to height
            intakeservo.setPosition(0.83);

            //let user change height of slides
            if (gamepad2.dpad_right) {
                myGoToHeightPOS(650, 1);
            }
            //SlidePID.changeHeight(slide.getCurrentPosition() + 50);
            else if (gamepad2.dpad_down) {
                //SlidePID.changeHeight(slide.getCurrentPosition() - 50);
                myGoToHeightPOS(0, 0.4);
            }
            else if (gamepad1.x)
                myGoToHeightPOS(40,1);
            else if (gamepad2.dpad_left)
                myGoToHeightPOS(200,1);
            else if (gamepad2.dpad_up)
                myGoToHeightPOS(500,1);

            //flip bucket
            if (gamepad2.left_bumper) {
                bucket.setPosition(0.47);
            }
            else if (gamepad2.right_bumper) {
                bucket.setPosition(0.84);
            }

            //change Blinkdin Color

            if(gamepad2.y) {
                led.changePattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
            }
            else if(gamepad2.a) {
                led.changePattern(RevBlinkinLedDriver.BlinkinPattern.LAWN_GREEN);
            }
            else if(gamepad2.x) {
                led.changePattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
            }
            else if(gamepad2.b) {
                led.changePattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_WHITE);
            }


            //sets power of intake
            power = gamepad1.right_trigger - gamepad1.left_trigger;
            Intake.Intake(intake, power*0.7);
            if(gamepad1.right_trigger>0.5)
                myGoToHeightPOS(40,1);

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            telemetry.addData("position", slide.getCurrentPosition());
            telemetry.update();

            led.update();
        }
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
