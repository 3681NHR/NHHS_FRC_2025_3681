package frc.robot.subsystems.affector.wrist;

import org.littletonrobotics.junction.AutoLog;

/**
 * interface for wrist IO
 * <p> all hardware interface will be done through this interface, control is done through commands and inputs are sent through the inputs class
 */
public interface WristIO {
    
    public default void updateInputs(WristIOInputs in) {}

    /**
     * set brake mode on motor
     * <p>motor braking will increse resistance to being moved when no power is applied
     * @param brake if true, brake mode is enabled
     */
    public default void setBrake(boolean brake) {}

    /**
     * set output voltage to the motor
     * @param v voltage to set motor to in volts
     */
    public default void setVoltage(double v) {}

    /**
     * object that stores all sensor inputs, all values are logged automatically
     */
    @AutoLog
    public class WristIOInputs{
        public double pos = 0.0;
        public double vel = 0.0;

        public double motorCurrentAmps = 0.0;
        public double motoroutVolts = 0.0;
        public double motorTemp = 0.0;
    }
}
