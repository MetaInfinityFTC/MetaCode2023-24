package org.firstinspires.ftc.teamcode.telecontrol;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.FallingEdge;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystem.Hang;
import org.firstinspires.ftc.teamcode.subsystem.deposit.Slides;
import org.firstinspires.ftc.teamcode.subsystem.extendo.Extendo;

public class EndgameControl implements Control{

    public Gamepad gp1, gp2;

    Robot r;


    public EndgameControl(Robot r, Gamepad one, Gamepad two){
        this.r = r;

        gp1 = one;
        gp2 = two;
    }

    FallingEdge launch = new FallingEdge(() -> r.dr.launch());
    FallingEdge hang = new FallingEdge(() -> {
       if(Hang.rHang.getPosition() == Hang.down){
           r.h.up();
           r.e.setState(Extendo.Extension_States.hung);
       }
       else{
           r.h.down();
           r.e.setState(Extendo.Extension_States.retracted);
       }
    });

    @Override
    public void update(){
        launch.update(gp1.a);

        hang.update(gp1.b);

    }
}
