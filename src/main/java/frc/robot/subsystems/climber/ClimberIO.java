package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    public default void updateInputs(ClimberIOInputs in) {}

    public default void setVoltage(double v) {}

    @AutoLog
    public class ClimberIOInputs{
        public double appliedVolts = 0;
        public double currentDraw = 0;
        public double motorTemp = 0;
    }
}
