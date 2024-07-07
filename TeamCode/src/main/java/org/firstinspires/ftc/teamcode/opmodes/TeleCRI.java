package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcontroller.external.samples.RobotTeleopPOV_Linear;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystem.Drive;
import org.firstinspires.ftc.teamcode.telecontrol.DepositControl;
import org.firstinspires.ftc.teamcode.telecontrol.DriveControl;
import org.firstinspires.ftc.teamcode.telecontrol.IntakeControl;

@TeleOp
@Config
public class TeleCRI extends LinearOpMode {
    Robot r;
    IntakeControl ic;
    DriveControl dc;
    DepositControl dpc;
    @Override
    public void runOpMode() throws InterruptedException {

        r = new Robot(hardwareMap, telemetry);
        ic = new IntakeControl(r, gamepad1, gamepad2);
        dc = new DriveControl(r, gamepad1, gamepad2);
        dpc = new DepositControl(r, gamepad1, gamepad2);


        while(opModeInInit()){

        }

        while(opModeIsActive()){


            dc.update();
            r.update();
        }
    }
}
