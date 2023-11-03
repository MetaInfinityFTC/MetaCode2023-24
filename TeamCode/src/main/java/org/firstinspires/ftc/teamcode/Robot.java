package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {

    public DcMotorEx frontLeftMotor;
    public DcMotorEx frontRightMotor;
    public DcMotorEx backLeftMotor;
    public DcMotorEx backRightMotor;

    public DcMotorEx leftSlideMotor;
    public DcMotorEx rightSlideMotor;

    public DcMotorEx intakeMotor;

    public Servo frontLeftServo;
    public Servo frontRightServo;
    public Servo backLeftServo;
    public Servo backRightServo;

    public AnalogInput xPod;
    public AnalogInput yPod;


    private HardwareMap hardwareMap;
    public boolean isEnabled = false;
    private static Robot instance = null;

    public static Robot getInstance() {
        if (instance == null) {
            instance = new Robot();
        }

        instance.isEnabled = true;
        return instance;
    }

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;

        // define motors & servos in initialization

    }

}
