package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

public class Blinkdin {

    RevBlinkinLedDriver.BlinkinPattern target;
    RevBlinkinLedDriver led;

    public Blinkdin(RevBlinkinLedDriver led){
        this.led = led;
    }

    public Blinkdin changePattern(RevBlinkinLedDriver.BlinkinPattern setting){
        this.target = setting;
        return this;
    }

    public RevBlinkinLedDriver.BlinkinPattern getPattern(){
        return target;
    }
    public void update(){
        this.led.setPattern(target);
    }

}