package frc.robot.subsystems.physButtons;

import org.littletonrobotics.junction.AutoLog;

/**
 * interface for physical button IO, all hardware interactions should be through
 * this interface, control should be done through functions and inputs should be
 * sent through the inputs class
 */
public interface ButtonIO {
    /**
     * update all sensor inputs, acts as a periodic function
     * 
     * @param inputs
     */
    public default void updateInputs(ButtonIOInputs inputs) {
    }

    /**
     * object that stores all sensor inputs, all values are logged automatically
     */
    @AutoLog
    public class ButtonIOInputs {
        public boolean pressed = false;
    }
}
