package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Drone implements Subsystem{

    Servo drone;

    HardwareMap h;

    public static double noLaunch = 0;
    public static double launch = 0.5;

    public Drone(HardwareMap h){
        this.h = h;
        drone = h.get(Servo.class, "drone");
    }

    public void setDrone(double d){
        drone.setPosition(d);
    }

    @Override
    public void update() {

    }

    @Override
    public void init() {
        setDrone(noLaunch);
    }

    public void launch(){
        setDrone(launch);
    }


}
