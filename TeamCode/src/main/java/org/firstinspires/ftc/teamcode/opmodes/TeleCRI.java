package org.firstinspires.ftc.teamcode.opmodes;

import android.app.VoiceInteractor;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.RobotTeleopPOV_Linear;
import org.firstinspires.ftc.teamcode.FallingEdge;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystem.Drive;
import org.firstinspires.ftc.teamcode.subsystem.GlobalCommands;
import org.firstinspires.ftc.teamcode.subsystem.deposit.NewDeposit;
import org.firstinspires.ftc.teamcode.telecontrol.DepositControl;
import org.firstinspires.ftc.teamcode.telecontrol.DriveControl;
import org.firstinspires.ftc.teamcode.telecontrol.EndgameControl;
import org.firstinspires.ftc.teamcode.telecontrol.IntakeControl;

enum State{NEUTRAL, INTAKE, DEPOSIT, TRANSFER, ENDGAME}



@TeleOp
@Config
public class TeleCRI extends LinearOpMode {


    Robot r;
    IntakeControl ic;
    DriveControl dc;
    DepositControl dpc;
    EndgameControl ec;

    public State s = State.NEUTRAL;

    public boolean transfering = false;
    @Override
    public void runOpMode() throws InterruptedException {

        r = new Robot(hardwareMap, telemetry);
        ic = new IntakeControl(r, gamepad1, gamepad2);
        dc = new DriveControl(r, gamepad1, gamepad2);
        dpc = new DepositControl(r, gamepad1, gamepad2);
        ec = new EndgameControl(r, gamepad1, gamepad2);


        r.init();

        while(opModeInInit()){

        }

        ElapsedTime t = new ElapsedTime();

        CommandScheduler cs = CommandScheduler.getInstance();
        Command transfer = GlobalCommands.getTransfer(r);

        FallingEdge endgameTrigger = new FallingEdge(() -> {if(s != State.ENDGAME && s != State.TRANSFER) {
            s = State.ENDGAME;
            r.d.collapse();
            r.s.setPosition(0);
        } else s = State.NEUTRAL;});


        while(opModeIsActive()){

            endgameTrigger.update(gamepad1.left_stick_button);

            switch (s){
                case NEUTRAL:
                    r.s.hang = false;
                    r.d.setWrist(NewDeposit.wristTransfer);
                    r.d.setArm(NewDeposit.armPreTransfer);
                    r.d.setSwivel(NewDeposit.swivelTransfer);
                    if(gamepad1.right_trigger > 0 || gamepad1.left_trigger > 0){
                        s = State.INTAKE;
                    }
                    if(gamepad1.x){
                        s = State.TRANSFER;
                    }
                    if(gamepad1.y){
                        s = State.DEPOSIT;
                    }
                    break;
                case INTAKE:
                    ic.update();
                    if(gamepad1.x){
                        s = State.TRANSFER;
                    }
                    break;
                case TRANSFER:
                    if(!cs.isScheduled(transfer)){
                        if(transfering){
                            transfering = false;
                            s = State.NEUTRAL;
                        }
                        else {
                            cs.schedule(transfer);
                            transfering = true;
                        }
                    }
                    break;
                case DEPOSIT:
                    dpc.update();
                    if(NewDeposit.lclaw.getPosition() == NewDeposit.lDrop && NewDeposit.rclaw.getPosition() == NewDeposit.rDrop){
                        s = State.NEUTRAL;
                        r.s.setPosition(0);
                    }
                    break;

                case ENDGAME:
                    ec.update();
                    break;
                default:
                    s = State.NEUTRAL;
            }

            r.tele.addData("LClaw", NewDeposit.lclaw.getPosition());
            r.tele.addData("RClaw", NewDeposit.rclaw.getPosition());
            r.tele.addData("State", s.name());

            cs.run();
            dc.update();
            r.update();
        }
    }
}
