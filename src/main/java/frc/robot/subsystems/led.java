package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import java.util.stream.Stream;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class led extends SubsystemBase {

    private boolean pattOn = true;
    private double pos;

    private AddressableLED led = new AddressableLED(1);
    private AddressableLEDBuffer buffer = new AddressableLEDBuffer(100);

    private LEDPattern patt = LEDPattern.gradient(GradientType.kContinuous, Color.kRed, Color.kBlue)
            .scrollAtRelativeSpeed(Percent.per(Second).of(25))
            .synchronizedBlink(RobotController::getRSLState);
    private LEDPattern alliance;

    private Color[] colors = new Color[buffer.getLength()];


    public led() {
        led.setLength(buffer.getLength());

        led.start();
    }
    
    @Override
    public void periodic() {
        if(pattOn){
            patt.applyTo(buffer);
        } else {
            alliance = LEDPattern.solid(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? Color.kBlue : Color.kRed)
            .mask(LEDPattern.progressMaskLayer(() -> pos));

            alliance.applyTo(buffer);
        }
        led.setData(buffer);

        for(int i = 0; i < buffer.getLength(); i++) {
            colors[i] = buffer.getLED(i);
        }
        Logger.recordOutput("led/buffer", Stream.of(colors).map(t -> t.toHexString()).toArray(String[]::new));
    }

    public void togglePattern() {
        pattOn = !pattOn;
    }
    public void setpos(double pos){
        this.pos = pos;
    }
    
}
