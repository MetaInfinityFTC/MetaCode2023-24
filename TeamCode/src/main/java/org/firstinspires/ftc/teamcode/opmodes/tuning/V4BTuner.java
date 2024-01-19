package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.intake.Virtual4Bar;

/*
Use this to find positions and set the positions in the subsystem
 */

@TeleOp
@Config
public class V4BTuner extends OpMode {

    Virtual4Bar v4b;
    public static double clawPos = 0, v4bPos = 0;

    @Override
    public void init() {
        v4b = new Virtual4Bar(hardwareMap);
    }

    @Override
    public void loop() {
        v4b.setClaw(clawPos);
        v4b.setV4b(v4bPos);
    }
}
