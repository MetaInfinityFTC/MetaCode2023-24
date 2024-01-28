package org.firstinspires.ftc.teamcode.telecontrol;

import com.qualcomm.robotcore.hardware.Gamepad;

public interface Control {
    void update(Gamepad gp1, Gamepad gp2);
}
