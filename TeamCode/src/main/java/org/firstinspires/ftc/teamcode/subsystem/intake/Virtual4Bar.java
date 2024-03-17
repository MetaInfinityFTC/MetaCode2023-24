package org.firstinspires.ftc.teamcode.subsystem.intake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Virtual4Bar {

    private Servo v4b, claw;
    private DigitalChannel breakBeam;

    //TODO: Tune
    //Can be accessed from outside the class for easy setting without a big enum/easy tuning
    public static double clawOpen = 0.5, clawClose = 0.15, transferOpen = 0.3;
    public static double v4bGround = 0.945, v4bTransfer = 0.45, v4bPreTransfer = 0.6, v4bStackHigh = 0.905, v4bStackMid = 0.92;
    //too many vars for a fancy enum lmao

    //TODO: Set string names to config names
    public Virtual4Bar(HardwareMap hardwareMap) {
        v4b = hardwareMap.servo.get("v4b");
        claw = hardwareMap.servo.get("claw");
        //v4b.setDirection(Servo.Direction.REVERSE);
        //breakBeam = hardwareMap.digitalChannel.get("breakbeam");
    }

    //functions for manual override just in case
    public void setClaw(double pos) {
        claw.setPosition(pos);
    }

    public void setV4b(double pos) {
        v4b.setPosition(pos);
    }

    /**
     * runs to the transfer ready position
     */
    public void transfer() {
        setClaw(clawClose);
        setV4b(v4bTransfer);
    }

    /**
     * runs to a position that is ready to intake
     * @param v4bPosition the position you want to intake from
     */
    public void intake(double v4bPosition) {
        setClaw(clawOpen);
        setV4b(v4bPosition);
    }

    public boolean doWeHaveAPixel() {
        return breakBeam.getState();
    }

}
