package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.Drive;
import org.firstinspires.ftc.teamcode.subsystem.Drone;
import org.firstinspires.ftc.teamcode.subsystem.Subsystem;
import org.firstinspires.ftc.teamcode.subsystem.Hang;
import org.firstinspires.ftc.teamcode.subsystem.deposit.NewDeposit;
import org.firstinspires.ftc.teamcode.subsystem.extendo.Extendo;
import org.firstinspires.ftc.teamcode.subsystem.intake.NewActiveIntake;
import org.firstinspires.ftc.teamcode.subsystem.deposit.Slides;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Robot {

    public NewDeposit d;
    public Extendo e;
    public NewActiveIntake i;
    public Slides s;
    public Drive db;
    public Drone dr;
    public Hang h;

    public List<Subsystem> subsystems;

    public HardwareMap hardwareMap;
    Telemetry tele;

    public double startTime = 0;


    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.tele = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        d = new NewDeposit(hardwareMap);
        e = new Extendo(hardwareMap);
        i = new NewActiveIntake(hardwareMap);
        s = new Slides(hardwareMap);
        db = new Drive(hardwareMap);
        dr = new Drone(hardwareMap);
        h = new Hang(hardwareMap);

        subsystems = new ArrayList<>(Arrays.asList(d, e, i, s, db, dr, h));
        // define motors & servos in initialization
    }

    public void init(){
        for (Subsystem s : subsystems){
            s.init();
        }
    }

    public void update(){
        for (Subsystem s : subsystems){
            s.update();
        }
        tele.update();
    }


}
