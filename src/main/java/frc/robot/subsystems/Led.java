package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Led extends SubsystemBase {

    private Color c = Color.kTeal;

    private boolean intaking = false;
    private boolean homed = false;

    private AddressableLED led = new AddressableLED(0);
    private AddressableLEDBuffer buffer = new AddressableLEDBuffer(49);

    private LEDPattern elevPos;

    private LoggedNetworkBoolean gayMode = new LoggedNetworkBoolean("gay mode", false);

    public Led() {
        led.setLength(buffer.getLength());

        led.start();
    }
    
    @Override
    public void periodic() {
        if(!homed){
            elevPos = LEDPattern.solid(Color.kRed).breathe(Seconds.of(2));
        } else {

            elevPos = LEDPattern.solid(c);
            if(intaking){
                elevPos = elevPos.blink(Seconds.of(.25));
            }
        }
        elevPos = elevPos.atBrightness(Percent.of(50));
        if(gayMode.get()){
            elevPos = LEDPattern.rainbow(255, 255).scrollAtRelativeSpeed(Percent.per(Second).of(25));
        }

        elevPos.applyTo(buffer);

        led.setData(buffer);
    }

    public void setIntaking(boolean in){
        this.intaking = in;
    }
    public void setHomed(boolean homed){
        this.homed = homed;
    }  
    public void setColor(Color c){
        this.c = c;
    }  
}