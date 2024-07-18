package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Hang implements Subsystem {

    public static Servo lHang, rHang;

    public static double down = 0.9;
    public static double up  = 0.5;

    HardwareMap h;

    public Hang(HardwareMap h){
        this.h = h;

        lHang = h.get(Servo.class, "lHang");
        rHang = h.get(Servo.class, "rHang");

        lHang.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void update() {

    }

    @Override
    public void init() {
        down();
    }

    public void down(){
        setHang(down);
    }

    public void up(){
        setHang(up);
    }


    public void setHang(double pos){
        lHang.setPosition(pos+0.05);
        rHang.setPosition(pos);
    }
}
