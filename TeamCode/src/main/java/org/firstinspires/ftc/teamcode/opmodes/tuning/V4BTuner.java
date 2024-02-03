package org.firstinspires.ftc.teamcode.opmodes.tuning;

import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.armPreTransfer;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.armTransfer;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.bothPixels;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.wristTransfer;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit;
import org.firstinspires.ftc.teamcode.subsystem.deposit.Slides;
import org.firstinspires.ftc.teamcode.subsystem.extendo.Extendo;
import org.firstinspires.ftc.teamcode.subsystem.intake.Virtual4Bar;

/*
Use this to find positions and set the positions in the subsystem
 */

@TeleOp
@Config
public class V4BTuner extends OpMode {

    Virtual4Bar v4b;
    Deposit deposit;
    public static double clawPos = 0, v4bPos = 0, wristPos = 0.2, fingerPos = bothPixels, armPos = 0.5;

    @Override
    public void init() {
        v4b = new Virtual4Bar(hardwareMap);
        deposit = new Deposit(hardwareMap);
    }

    @Override
    public void loop() {
        deposit.setWrist(wristPos);
        deposit.setArm(armPos);
        deposit.setFinger(fingerPos);
    }
}
