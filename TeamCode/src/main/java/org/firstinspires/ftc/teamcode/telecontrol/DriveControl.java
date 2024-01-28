package org.firstinspires.ftc.teamcode.telecontrol;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class DriveControl implements Control{
    SampleMecanumDrive drive;

    public DriveControl(SampleMecanumDrive drive) {
        this.drive = drive;
    }
    @Override
    public void update(Gamepad gamepad1, Gamepad gamepad2) {
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        drive.setWeightedDrivePower(new Pose2d(y, x, rx));
    }
}
