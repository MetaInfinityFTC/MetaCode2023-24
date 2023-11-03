package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;

public abstract class MetaSubsystem extends SubsystemBase {
    public abstract void periodic();
    public abstract void read();
    public abstract void write();
    public abstract void reset();
}
