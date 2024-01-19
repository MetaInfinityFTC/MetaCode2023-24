package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/*
Use this to find positions and set the positions in the subsystem
 */

@TeleOp
public class ExtendoPosFinder extends OpMode {

    public DcMotor right;

    @Override
    public void init() {
        right = hardwareMap.dcMotor.get("rightEx");
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void loop() {
        telemetry.addData("extendo pos in ticks", right.getCurrentPosition());
        telemetry.update();
    }
}
