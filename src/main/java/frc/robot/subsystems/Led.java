package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Map;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Led extends SubsystemBase {

    public boolean hasCoral = false;
    public boolean rotLock = false;
    public boolean aligningReef = false;
    public boolean homing = false;
    public boolean climbMode = false;
    public boolean homed = false;
    public boolean posAlign = false;
    public boolean rotAlign = false;
    public boolean affectorInPos = false;
    public boolean alignInPos = false;

    private boolean intakeRunning = false;

    private AddressableLED led = new AddressableLED(0);
    private AddressableLEDBuffer buffer = new AddressableLEDBuffer(50);

    private LEDPattern pattern;

    private LoggedNetworkBoolean rainbow = new LoggedNetworkBoolean("LED override", false);

    public Led() {
        led.setLength(buffer.getLength());

        led.start();
    }
    
    @Override
    public void periodic() {
        LEDPattern status = LEDPattern.solid(Color.kBlack);
        LEDPattern state  = LEDPattern.solid(Color.kBlack);
        LEDPattern stateMask = LEDPattern.steps(Map.of(0, Color.kBlack, 0.5, Color.kWhite));
        LEDPattern statusMask  = LEDPattern.steps(Map.of(0, Color.kWhite, 0.5, Color.kBlack));
        
        if(affectorInPos){
            status = LEDPattern.solid(new Color(0, 0, 0));
        }
        // if(posAlignMode){
            status = LEDPattern.solid(Color.kYellow);
        // }
        // if(rotAlignMode){
        //     status = LEDPattern.solid(Color.kOrange);
        // }
        if(homing){
            state = LEDPattern.solid(new Color(255, 0, 0));
            state = state.mask(LEDPattern.steps(Map.of(0, Color.kBlack, 0.45, Color.kWhite, 0.55, Color.kBlack)));
            state = state.scrollAtRelativeSpeed(Percent.per(Second).of(0.25));
        }
        if(alignInPos){
            status = LEDPattern.solid(Color.kTeal);
        }
        if(hasCoral){
            state = LEDPattern.solid(new Color(0, 255, 0));
        }
        if(rotLock){
            state = LEDPattern.solid(new Color(255, 255, 255));
        }
        if(!homed){
            status = LEDPattern.solid(new Color(255, 0, 0));
            status = status.breathe(Seconds.of(1));
        }
        if(aligningReef){
            state = LEDPattern.solid(new Color(0, 0, 255));
        }
        if(climbMode){
            state = LEDPattern.solid(Color.kMagenta);
            status = LEDPattern.solid(Color.kMagenta);
        }


        status = status.mask(statusMask);
        state  = state.mask(stateMask);

        pattern = status.overlayOn(state);

        if(intakeRunning){
            pattern = pattern.blink(Seconds.of(.125));
        }

        if(rainbow.get()){
            pattern = LEDPattern.rainbow(255, 255).scrollAtRelativeSpeed(Percent.per(Second).of(25));
        }

        pattern.applyTo(buffer);

        led.setData(buffer);

        Logger.recordOutput("led status", buffer.getLED(0).toHexString());
        Logger.recordOutput("led state" , buffer.getLED(49).toHexString());

        Logger.recordOutput("led/hasCoral", hasCoral);
        Logger.recordOutput("led/rotAlign", rotAlign);
        Logger.recordOutput("led/posAlign", posAlign);
        Logger.recordOutput("led/rotLock", rotLock);
        Logger.recordOutput("led/aligningReef", aligningReef);
        Logger.recordOutput("led/homing", homing);
        Logger.recordOutput("led/climbing", climbMode);
        Logger.recordOutput("led/homed", homed);
        Logger.recordOutput("led/affectorInPos", affectorInPos);
        Logger.recordOutput("led/alignInPos", alignInPos);

        for(int i=0; i<buffer.getLength(); i++){
            Logger.recordOutput("leds/"+i , buffer.getLED(i).toHexString());
        }
    }

    public void setRunning(boolean in){
        this.intakeRunning = in;
    }
    // public void setColor(Color c){
    //     this.c = c;
    // }
    // public void setState(LEDState state){
    //     // if(state.getPriority() > currentState.getPriority()){
    //         this.currentState = state;
    //     // }
    // }
    // public void setStatus(LEDStatus status){
    //     // if(status.getPriority() > currentStatus.getPriority()){
    //         this.currentStatus = status;
    //     // }
    // }
}