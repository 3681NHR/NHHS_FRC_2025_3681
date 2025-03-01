package frc.robot.subsystems.physButtons;

import org.littletonrobotics.junction.AutoLog;

public interface ButtonIO {
    public default void updateInputs(ButtonIOInputs inputs) {}

    @AutoLog
    public class ButtonIOInputs {
        public boolean pressed = false;
    }
}
