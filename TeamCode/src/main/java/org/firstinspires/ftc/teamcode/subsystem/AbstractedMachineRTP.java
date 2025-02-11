package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.zeroPixel;
import static org.firstinspires.ftc.teamcode.subsystem.extendo.Extendo.Extension_States.retracted;
import static org.firstinspires.ftc.teamcode.subsystem.intake.Virtual4Bar.clawClose;
import static org.firstinspires.ftc.teamcode.subsystem.intake.Virtual4Bar.v4bPreTransfer;
import static org.firstinspires.ftc.teamcode.subsystem.intake.Virtual4Bar.v4bStackHigh;
import static org.firstinspires.ftc.teamcode.subsystem.intake.Virtual4Bar.v4bTransfer;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit;
import org.firstinspires.ftc.teamcode.subsystem.extendo.Extendo;
import org.firstinspires.ftc.teamcode.subsystem.intake.Virtual4Bar;

@Deprecated
public class AbstractedMachineRTP {

    public enum Transfer {
        GRAB, TRANSFER, FINISHED, CLAW_OPEN, TRANSFER2, ARMDOWN, BOOM, PRE_TRANSFER

    }

    public enum Drop {
        DROP, SECOND, RESET
    }


    public static StateMachine getTransferMachine(Virtual4Bar virtual4Bar, Extendo extendo, Deposit deposit) {
        return  new StateMachineBuilder()
                .state(Transfer.GRAB)
                .onEnter(() -> virtual4Bar.setClaw(clawClose))
                .transitionTimed(0.3) // .2 seconds to grab before putting everything in transfer

                .state(Transfer.PRE_TRANSFER)
                .onEnter(() -> {
                    virtual4Bar.setV4b(v4bPreTransfer); //raise off ground
                    extendo.extendosetPidTarget(0, 0.5);
                })
                .transition(extendo::isAtTarget) //0.2 seconds to move everything before bringing v4b back
                .onExit(() -> {
                    virtual4Bar.setV4b(v4bPreTransfer);
                })
                .waitState(1)
                .state(Transfer.TRANSFER)
                .onEnter(() -> {
                    virtual4Bar.setV4b(v4bTransfer);
                })
                .transitionTimed(0.5)
                .state(Transfer.TRANSFER2)
                .onEnter(() -> {
                    deposit.setFinger(zeroPixel);
                })/*
                .transitionTimed(0.35)
                .state(Transfer.CLAW_OPEN)
                .onEnter(()-> {
                    deposit.setArm(armTransfer);
                })*/
                .transitionTimed(0.5)
                .state(Transfer.BOOM)
                .onEnter(() -> {
                    deposit.setArm(0.85);
                })
                .transitionTimed(0.5)
                .state(Transfer.ARMDOWN)
                .onEnter(() -> {
                    virtual4Bar.setClaw(0.4); //open slightly so pixels can come out
                    deposit.setArm(0.85);
                })
                .state(Transfer.FINISHED) // end state

                .build();

    }

    public static StateMachine dropMachine(Deposit deposit, Virtual4Bar virtual4Bar) {
        return new StateMachineBuilder()
                .waitState(1)
                .state(Drop.DROP)
                .transitionTimed(0.3)
                .state(Drop.SECOND)
                .onEnter(()-> {
                    deposit.setFinger(zeroPixel);
                })
                .transitionTimed(0.5)
                .state(Drop.RESET)
                .onEnter(()->{
                })
                .build();
    }
}
