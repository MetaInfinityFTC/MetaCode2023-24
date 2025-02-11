package org.firstinspires.ftc.teamcode.opmodes.tuning;

import static org.firstinspires.ftc.teamcode.subsystem.intake.Virtual4Bar.v4bPreTransfer;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.extendo.Extendo;
import org.firstinspires.ftc.teamcode.subsystem.intake.Virtual4Bar;

/*
Use this to tune the extendo PID
 */

//TODO: lemme know if this doesnt work, cause i can make a version that doesnt access the subsystem directly

@TeleOp
@Config
public class ExtendoTuner extends OpMode {

    Extendo slides;
    Virtual4Bar v4b;

    public static Extendo.Extension_States states = Extendo.Extension_States.retracted;

    public static int TargetPos = 0;

    @Override
    public void init() {
        slides = new Extendo(hardwareMap);
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        slides.setState(states);
        slides.update();
        telemetry.addData("extendoPos", slides.getPos());
        telemetry.addData("targetPos", states.getTicks());
        telemetry.update();
    }


}
