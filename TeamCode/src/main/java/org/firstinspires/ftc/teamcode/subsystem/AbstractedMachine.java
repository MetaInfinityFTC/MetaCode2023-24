package org.firstinspires.ftc.teamcode.subsystem;

import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;
import org.firstinspires.ftc.teamcode.opmodes.DaTele;
import org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit;
import org.firstinspires.ftc.teamcode.subsystem.extendo.Extendo;
import org.firstinspires.ftc.teamcode.subsystem.intake.Virtual4Bar;

import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.*;
import static org.firstinspires.ftc.teamcode.subsystem.deposit.Deposit.armTransfer;
import static org.firstinspires.ftc.teamcode.subsystem.extendo.Extendo.Extension_States.retracted;
import static org.firstinspires.ftc.teamcode.subsystem.intake.Virtual4Bar.clawClose;
import static org.firstinspires.ftc.teamcode.subsystem.intake.Virtual4Bar.v4bPreTransfer;
import static org.firstinspires.ftc.teamcode.subsystem.intake.Virtual4Bar.v4bTransfer;

public class AbstractedMachine {

    public enum Transfer {
        GRAB, TRANSFER, FINISHED, CLAW_OPEN, TRANSFER2, ARMDOWN, BOOM, PRE_TRANSFER

    }

    public static StateMachine getTransferMachine(Virtual4Bar virtual4Bar, Extendo extendo, Deposit deposit) {
        return  new StateMachineBuilder()
                .state(Transfer.GRAB)
                .onEnter(() -> virtual4Bar.setClaw(clawClose))
                .transitionTimed(.2) // .2 seconds to grab before putting everything in transfer

                .state(Transfer.PRE_TRANSFER)
                .onEnter(() -> {
                    virtual4Bar.setV4b(0.8); //raise off ground
                    extendo.setState(retracted);
                    deposit.setWrist(wristTransfer);
                    deposit.setArm(armPreTransfer);
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
                .transitionTimed(1)
                .state(Transfer.TRANSFER2)
                .onEnter(() -> {
                    deposit.setFinger(zeroPixel);
                    deposit.setArm(armTransfer);
                })
                .transitionTimed(0.5)
                .state(Transfer.CLAW_OPEN)
                .onEnter(()-> {
                    deposit.setArm(armTransfer);
                })
                .transitionTimed(0.5)
                .state(Transfer.BOOM)
                .onEnter(() -> {
                    deposit.setFinger(bothPixels);
                    deposit.setArm(1);
                })
                .transitionTimed(1)
                .state(Transfer.ARMDOWN)
                .onEnter(() -> {
                    virtual4Bar.setClaw(0.45); //open slightly so pixels can come out
                    deposit.setArm(1);
                })
                .state(Transfer.FINISHED) // end state

                .build();

    }
}
