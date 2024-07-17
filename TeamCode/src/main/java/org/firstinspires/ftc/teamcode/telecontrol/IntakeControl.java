package org.firstinspires.ftc.teamcode.telecontrol;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.FallingEdge;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystem.extendo.Extendo;

public class IntakeControl implements Control{

    Robot r;
    Gamepad gp1, gp2;

    public IntakeControl(Robot r, Gamepad one, Gamepad two){
        this.r = r;
        gp1 = one;
        gp2 = two;
    }

    FallingEdge ground = new FallingEdge(() -> r.i.ground());
    FallingEdge mid = new FallingEdge(() -> r.i.mid());
    FallingEdge high = new FallingEdge(() -> r.i.high());

    FallingEdge in = new FallingEdge(() -> r.e.setState(Extendo.Extension_States.retracted));
    FallingEdge p1 = new FallingEdge(() -> r.e.setState(Extendo.Extension_States.first));
    FallingEdge p2 = new FallingEdge(() -> r.e.setState(Extendo.Extension_States.second));
    FallingEdge out = new FallingEdge(() -> r.e.setState(Extendo.Extension_States.extended));

    @Override
    public void update() {

        r.i.setIntake(gp1.left_trigger - gp1.right_trigger);

        ground.update(gp1.a);
        mid.update(gp1.y);
        high.update(gp1.b);

        in.update(gp1.dpad_down);
        p1.update(gp1.dpad_left);
        p2.update(gp1.dpad_right);
        out.update(gp1.dpad_up);




    }
}
