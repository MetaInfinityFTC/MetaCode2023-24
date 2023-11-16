package org.firstinspires.ftc.teamcode.subsystem.intake;

import com.qualcomm.robotcore.hardware.AnalogInput;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ActiveIntake extends LinearOpMode{
    //get our analog input from the hardwareMap
    AnalogInput analogInput = hardwareMap.get(AnalogInput.class, "miniaxon");
    Servo intakeservo = hardwareMap.servo.get("intakeservo");

    // get the voltage of our analog line
// divide by 3.3 (the max voltage) to get a value between 0 and 1
// multiply by 360 to convert it to 0 to 360 degrees
    double position = analogInput.getVoltage() / 3.3;


    @Override
    public void runOpMode() throws InterruptedException {
        //intakeservo.setPosition(0);
        telemetry.addData("angle", analogInput);
    }
}
