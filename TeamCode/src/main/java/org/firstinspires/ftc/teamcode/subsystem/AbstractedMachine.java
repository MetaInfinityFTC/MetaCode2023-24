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
import static org.firstinspires.ftc.teamcode.subsystem.intake.Virtual4Bar.v4bTransfer;

public class AbstractedMachine {

    public enum Transfer {
        GRAB, TRANSFER, FINISHED, PRE_TRANSFER

    }

    public static StateMachine getTransferMachine(Virtual4Bar virtual4Bar, Extendo extendo, Deposit deposit) {
        return  new StateMachineBuilder()
                .state(Transfer.GRAB)
                .onEnter(() -> virtual4Bar.setClaw(clawClose))
                .transitionTimed(.2) // .2 seconds to grab before putting everything in transfer

                .state(Transfer.PRE_TRANSFER)
                .onEnter(() -> {
                    virtual4Bar.setV4b(0.9); //raise off ground
                    extendo.setState(retracted);
                    deposit.setWrist(wristTransfer);
                    deposit.setArm(armPreTransfer);
                })
                .transition(extendo::isAtTarget) //0.2 seconds to move everything before bringing v4b back
                .onExit(() -> {
                    virtual4Bar.setV4b(v4bTransfer);
                })

                .state(Transfer.TRANSFER)
                .onEnter(() -> {
                    deposit.setFinger(bothPixels);
                    deposit.setArm(armTransfer);
                })
                .transitionTimed(0.2) //wait before going to Depo Pos
                .onExit(() -> {
                    virtual4Bar.setClaw(0.34); //open slightly so pixels can come out
                })

                .state(Transfer.FINISHED) // end state

                .build();

    }
}
