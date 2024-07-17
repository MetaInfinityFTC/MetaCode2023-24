package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp
@Config
public class AllServoTest extends LinearOpMode {

    public static double clawL = 0;
    public static double clawR = 0;
    public static double wrist = 0;
    public static double arm = 0;
    public static double swivel = 0;

    public static double linkage = 0;

    public static double drone = 0;

    public static double hang = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        Robot r = new Robot(hardwareMap, telemetry);

        while(opModeIsActive()){
            r.d.setWrist(wrist);
            r.d.setArm(arm);
            r.d.setClawL(clawL);
            r.d.setClawR(clawR);
            r.d.setSwivel(swivel);

            r.i.setLinkage(linkage);

            r.dr.setDrone(drone);

            r.h.setHang(hang);

        }

    }
}
