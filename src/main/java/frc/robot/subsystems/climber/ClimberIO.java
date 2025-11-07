package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

/**
 * climber IO interface
 * ,
 * <p>
 * all hardware interactions should be through the IO interface, control should
 * be done through functions and inputs should be sent through the inputs class
 */
public interface ClimberIO {

    /**
     * update all sensor inputs, acts as a periodic function
     * 
     * @param in object to store all sensor inputs to
     */
    public default void updateInputs(ClimberIOInputs in) {
    }

    /**
     * set voltage for motor
     * 
     * @param v voltage to set motor to in volts
     */
    public default void setVoltage(double v) {
    }

    /**
     * object to store all sensor inputs, all values are logged automatically
     */
    @AutoLog
    public class ClimberIOInputs {
        public double appliedVolts = 0;
        public double currentDraw = 0;
        public double motorTemp = 0;
    }
}
