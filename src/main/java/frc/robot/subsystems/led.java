package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Seconds;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class led extends SubsystemBase {

    private double pos;
    private Color c = Color.kTeal;

    private boolean intaking = false;
    private boolean holding = false;
    @AutoLogOutput
    private boolean homed = false;

    private AddressableLED led = new AddressableLED(0);
    private AddressableLEDBuffer buffer = new AddressableLEDBuffer(49);

    private LEDPattern elevPos = LEDPattern.solid(Color.kTeal).mask(LEDPattern.progressMaskLayer(() -> pos));

    public led() {
        led.setLength(buffer.getLength());

        led.start();
    }
    
    @Override
    public void periodic() {
        if(!homed){
            elevPos = LEDPattern.solid(Color.kRed).breathe(Seconds.of(2));
        } else {
            if(holding){
                c = Color.kGreen;
            } else {
                c = Color.kYellow;        }
            elevPos = LEDPattern.solid(c).mask(LEDPattern.progressMaskLayer(() -> pos));
            if(intaking && !holding){
                elevPos.blink(Seconds.of(.25));
            }
        }
        elevPos = elevPos.atBrightness(Percent.of(50));

        elevPos.applyTo(buffer);

        led.setData(buffer);
    }

    public void setpos(double pos){
        this.pos = pos;
    }
    public void setHolding(boolean holding){
        this.holding = holding;
    }
    public void setIntaking(boolean in){
        this.intaking = in;
    }
    public void setHomed(boolean homed){
        this.homed = homed;
    }
    
}