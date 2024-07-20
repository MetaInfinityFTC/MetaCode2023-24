package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.FunctionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystem.deposit.NewDeposit;

public class GlobalCommands{

    public static double GRAB_DELAY = 1;
    public static double ARM_DELAY = 1;
    public static double LIFT_DELAY = 0.6;

    public static SequentialCommandGroup getTransfer1(Robot r){
        return new SequentialCommandGroup(new FunctionalCommand(() -> {r.startTime = System.nanoTime();}, ()->{r.d.setArm(NewDeposit.armTransfer);r.d.clawDrop();r.d.setSwivel(NewDeposit.swivelTransfer);}, (Boolean ended) -> {}, () -> System.nanoTime() - r.startTime > ARM_DELAY * 1e9),
                new FunctionalCommand(() -> {r.startTime = System.nanoTime();}, ()->{r.d.clawGrab();}, (Boolean ended) -> {}, () -> System.nanoTime() - r.startTime > GRAB_DELAY * 1e9)/*,
                new FunctionalCommand(() -> {r.startTime = System.nanoTime();}, ()->{r.d.setArm(NewDeposit.armPreTransfer);}, (Boolean ended) -> {}, () -> System.nanoTime() - r.startTime > LIFT_DELAY * 1e9)*/);

        }
}

