package org.firstinspires.ftc.teamcode.subsystem.drivetrain.butterfly;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class ButterflyModule {

    Servo moduleServo;
    DcMotorEx moduleMotor;

    private ButterflyState state;

    private boolean isRaised = false;
    private boolean isLocked = false;

    public ButterflyModule(Servo moduleServo, DcMotorEx moduleMotor) {
        this.moduleMotor = moduleMotor;
        this.moduleServo = moduleServo;
    }

    void lock() {
        setState(ButterflyState.IS_LOCKED, true);
        setState(ButterflyState.IS_RAISED, false);
        update();
    }

    void update() {
        if (isRaised && moduleServo.getPosition() != 1) {
            moduleServo.setPosition(1);
        } else {
            moduleServo.setPosition(0);
        }
    }

    void setPower(double power) {
        moduleMotor.setPower(power);
    }

    private void setState(ButterflyState state, boolean status) {
        if (state == ButterflyState.IS_RAISED) {
            isRaised = status;
        } else if (state == ButterflyState.IS_LOCKED) {
            isLocked = status;
        }
    }

    public boolean getLocked() {
        return isLocked;
    }

    public boolean getRaised() {
        return isRaised;
    }
}
