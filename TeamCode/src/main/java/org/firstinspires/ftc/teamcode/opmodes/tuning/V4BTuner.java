package org.firstinspires.ftc.teamcode.opmodes.tuning;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit;
import org.firstinspires.ftc.teamcode.subsystem.deposit.Slides;
import org.firstinspires.ftc.teamcode.subsystem.extendo.Extendo;
import org.firstinspires.ftc.teamcode.subsystem.intake.ActiveIntake;
import org.firstinspires.ftc.teamcode.subsystem.intake.Virtual4Bar;

/*
Use this to find positions and set the positions in the subsystem
 */

@TeleOp
@Config
public class V4BTuner extends OpMode {

    Virtual4Bar v4b;
    Deposit deposit;
    ActiveIntake intake;
    public static double clawPos = 0.2, v4bPos = 0.6, wristPos = 0.2, fingerPos = 0, armPos = 0.7;

    @Override
    public void init() {
        //ntake = new ActiveIntake(hardwareMap);
    }

    @Override
    public void loop() {
        /*deposit.setWrist(wristPos);
        deposit.setArm(armPos);
        deposit.setFinger(fingerPos);
        v4b.setV4b(v4bPos);
        v4b.setClaw(clawPos);*/
        //intake.neutral();
        //intake.setIntake(gamepad1.right_trigger - gamepad1.left_trigger);
    }
}
