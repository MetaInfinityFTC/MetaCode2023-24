package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.deposit.Slides;


@TeleOp
@Config
public class SlidesTuner extends LinearOpMode {

    public static double goalPos = 0;

    public void runOpMode() throws InterruptedException {
        Slides s = new Slides(hardwareMap);

        Telemetry t = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        while(opModeInInit()){
            telemetry.update();
        }
        while(opModeIsActive()){
            s.setPidTarget(goalPos);

            t.addData("Goal pos", goalPos);
            t.addData("Current pos", s.getPos());
            s.update();
            t.update();
        }
    }
}
