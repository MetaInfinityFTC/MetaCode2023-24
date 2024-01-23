package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.extendo.Extendo;

/*
Use this to tune the extendo PID
 */

//TODO: lemme know if this doesnt work, cause i can make a version that doesnt access the subsystem directly

@TeleOp
@Config
public class ExtendoTuner extends OpMode {

    Extendo extendo;

    public static Extendo.Extension_States states = Extendo.Extension_States.first;

    @Override
    public void init() {
        extendo = new Extendo(hardwareMap);
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        extendo.setState(states);
        extendo.update();
        telemetry.addData("extendoPos", extendo.getPos());
        telemetry.addData("targetPos", states.getNumVal());
        telemetry.update();
    }
}
