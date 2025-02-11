package org.firstinspires.ftc.teamcode.telecontrol;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.internal.stellaris.FlashLoaderCommand;
import org.firstinspires.ftc.teamcode.FallingEdge;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystem.deposit.NewDeposit;
import org.firstinspires.ftc.teamcode.subsystem.deposit.Slides;

public class DepositControl implements Control{

    public Robot r;
    Gamepad gp1, gp2;

    public DepositControl(Robot r, Gamepad one, Gamepad two){
        this.r = r;
        gp1 = one;
        gp2 = two;
    }

    public boolean low = true;

    FallingEdge dropL = new FallingEdge(() -> r.d.dropL());
    FallingEdge dropR = new FallingEdge(() -> r.d.dropR());
    FallingEdge drop = new FallingEdge(() -> r.d.clawDrop());

    FallingEdge swivelL = new FallingEdge(() -> r.d.swivelPos++);
    FallingEdge swivelR = new FallingEdge(() -> r.d.swivelPos--);

    FallingEdge rowUp = new FallingEdge(() -> r.s.row++);
    FallingEdge rowDown = new FallingEdge(() -> r.s.row--);

    FallingEdge switchOuttake = new FallingEdge(() -> low = !low);


    @Override
    public void update() {

        r.d.setSwivel();
        if(low){
            r.d.setArm(NewDeposit.armDeposit);
            r.d.setWrist(NewDeposit.wrist30degree);
        }
        else{
            r.d.setArm(NewDeposit.armHighDeposit);
            r.d.setWrist(NewDeposit.wrist90Degree);
        }
        switchOuttake.update(gp1.a);
        swivelL.update(gp1.left_bumper);
        swivelR.update(gp1.right_bumper);
        drop.update(gp1.x);
        dropL.update(gp1.left_trigger > 0);
        dropR.update(gp1.right_trigger > 0);

        rowUp.update(gp1.dpad_up);
        rowDown.update(gp1.dpad_down);

        r.s.setPosition();
    }
}
