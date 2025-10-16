package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

/**
 * interface for intake IO, all hardware interfaec will be done through this
 * interface, control is done through functions and inputs are sent through the
 * inputs class
 */
public interface IntakeIO {

    /**
     * update all sensor inputs, acts as a periodic function
     * 
     * @param inputs
     */
    public default void updateInputs(IntakeIOInputs inputs) {
    }

    /**
     * set brake mode for motor, when brake is enabled, motor will have much more
     * resistance to being moved when not driven
     * 
     * @param brake true to enable brake mode
     */
    public default void setBrakeMode(boolean brake) {
    }

    /**
     * set motor voltage
     * 
     * @param voltage output voltage in volts
     */
    public default void setVoltage(double voltage) {
    }

    /**
     * object that stores all sensor inputs, all values are logged automatically
     */
    @AutoLog
    public class IntakeIOInputs {
        public double motorVoltage = 0.0;
        public double motorCurrent = 0.0;
        public double motorTemperature = 0.0;
        public double motorVelocityRPM = 0.0;
        public boolean holding = false;
    }
}
