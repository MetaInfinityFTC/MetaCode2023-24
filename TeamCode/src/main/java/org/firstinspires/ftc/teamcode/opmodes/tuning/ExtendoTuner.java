package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.deposit.Slides;
import org.firstinspires.ftc.teamcode.subsystem.extendo.Extendo;

/*
Use this to tune the extendo PID
 */

//TODO: lemme know if this doesnt work, cause i can make a version that doesnt access the subsystem directly

@TeleOp
@Config
public class ExtendoTuner extends OpMode {

    Slides slides;

    public static double TargetPos = 0.0;

    @Override
    public void init() {
        slides = new Slides(hardwareMap);
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        slides.setPidTarget(TargetPos);
        slides.updatePID();
        telemetry.addData("extendoPos", slides.getPos());
        telemetry.addData("targetPos", TargetPos);
        telemetry.update();
    }
}
