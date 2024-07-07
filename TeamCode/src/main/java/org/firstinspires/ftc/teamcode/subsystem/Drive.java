package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class Drive implements Subsystem{

    public HardwareMap hardwareMap;
    public SampleMecanumDrive drive;

    public Drive(HardwareMap h){
        hardwareMap = h;
        drive = new SampleMecanumDrive(h);
    }

    @Override
    public void update() {
        drive.update();
    }

    @Override
    public void init() {

    }
}
