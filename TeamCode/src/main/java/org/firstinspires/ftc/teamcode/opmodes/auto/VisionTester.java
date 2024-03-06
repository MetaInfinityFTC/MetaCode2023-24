package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.vision.Vision;

import java.util.List;

@TeleOp
public class VisionTester extends LinearOpMode {
    Vision vision;
    @Override
    public void runOpMode(){
        vision = new Vision(hardwareMap,telemetry,false);

        while (!isStarted()){
            telemetry.update();
            //checks for red prop
        }

        waitForStart();
        vision.disablePropProcessor();
        while(opModeIsActive()){
            List<Pose2d> relocalization = vision.getTagLocalizationData();
            for(Pose2d p : relocalization){
                telemetry.addLine(p.getX() + " " + p.getY() + " " + p.getHeading());
            }
            telemetry.update();
        }
    }
}
