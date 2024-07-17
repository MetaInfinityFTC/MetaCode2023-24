package org.firstinspires.ftc.teamcode.telecontrol;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class DriveControl implements Control{
    Robot r;
    Gamepad gp1, gp2;

    public DriveControl(Robot r, Gamepad one, Gamepad two) {
        this.r = r;
        gp1 = one;
        gp2 = two;
    }
    @Override
    public void update() {
        double y = -gp1.left_stick_y; // Remember, Y stick value is reversed
        double x = gp1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gp1.right_stick_x;

        r.db.drive.setWeightedDrivePower(new Pose2d(y, x, -rx));
    }
}
