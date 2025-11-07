package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.LEDAnim.LEDAnim;
import frc.utils.LEDAnim.RainbowAnim;

public class Led extends SubsystemBase {

    public boolean hasCoral = false;
    public boolean rotLock = false;
    public boolean aligningReef = false;
    public boolean homing = false;
    public boolean climbMode = false;
    public boolean homed = false;
    public boolean affectorInPos = false;
    public boolean alignInPos = false;
    public boolean intakeSensorFault = false;

    private boolean intakeRunning = false;

    private AddressableLED led = new AddressableLED(0);
    private AddressableLEDBuffer buffer = new AddressableLEDBuffer(50);

    private LoggedNetworkBoolean rainbow = new LoggedNetworkBoolean("LED override", false);

    private LEDAnim b = new RainbowAnim(50);

    public Led() {
        led.setLength(buffer.getLength());

        led.start();
    }

    @Override
    public void periodic() {
        Color status = Color.kBlack;
        Color state = Color.kBlack;

        // if(affectorInPos){
        // status = Color.kBlack;
        // }
        if (homing) {
            state = new Color(255, 0, 0);
            // state = state.mask(LEDPattern.steps(Map.of(0, Color.kBlack, 0.45,
            // Color.kWhite, 0.55, Color.kBlack)));
            // state = state.scrollAtRelativeSpeed(Percent.per(Second).of(0.25));
        }
        if (hasCoral) {
            state = new Color(0, 255, 0);
        } else {
            state = new Color(255, 50, 0);
            alignInPos = false;
        }
        if (intakeSensorFault) {
            state = new Color(0, 0, 255);
        }
        if (intakeRunning) {
            state = new Color(255, 255, 0);
        }

        if (rotLock) {
            status = new Color(255, 255, 255);
            alignInPos = false;
        }
        if (aligningReef) {
            status = new Color(0, 0, 255);
            alignInPos = false;
        }
        if (alignInPos) {
            status = new Color(0, 255, 255);
        }
        if (!homed) {
            state = new Color(255, 0, 0);
            // status = status.breathe(Seconds.of(1));
        }
        if (climbMode) {
            state = Color.kMagenta;
            status = Color.kMagenta;
        }

        // if(intakeRunning){
        // pattern = pattern.blink(Seconds.of(.125));
        // }

        if (rainbow.get()) {
            for (int i = 0; i < buffer.getLength(); i++) {
                buffer.setLED(i, b.getLEDs()[i]);
            }
        } else {
            for (int i = 0; i < buffer.getLength(); i++) {
                buffer.setLED(i, i < buffer.getLength() / 2 ? status : state);// overlayOn is broken, so we use this
            }
        }

        led.setData(buffer);

        Logger.recordOutput("led status", buffer.getLED(0).toHexString());
        Logger.recordOutput("led state", buffer.getLED(49).toHexString());

        Logger.recordOutput("led/hasCoral", hasCoral);
        Logger.recordOutput("led/rotLock", rotLock);
        Logger.recordOutput("led/aligningReef", aligningReef);
        Logger.recordOutput("led/homing", homing);
        Logger.recordOutput("led/climbing", climbMode);
        Logger.recordOutput("led/homed", homed);
        Logger.recordOutput("led/affectorInPos", affectorInPos);
        Logger.recordOutput("led/alignInPos", alignInPos);
        Logger.recordOutput("led/intakerunning", intakeRunning);
        Logger.recordOutput("led/Sensor fault", intakeSensorFault);

        for (int i = 0; i < buffer.getLength(); i++) {
            Logger.recordOutput("leds/" + i, buffer.getLED(i).toHexString());
        }
    }

    public void setRunning(boolean in) {
        this.intakeRunning = in;
    }
    // public void setColor(Color c){
    // this.c = c;
    // }
    // public void setState(LEDState state){
    // // if(state.getPriority() > currentState.getPriority()){
    // this.currentState = state;
    // // }
    // }
    // public void setStatus(LEDStatus status){
    // // if(status.getPriority() > currentStatus.getPriority()){
    // this.currentStatus = status;
    // // }
    // }
}