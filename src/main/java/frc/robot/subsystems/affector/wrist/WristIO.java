package frc.robot.subsystems.affector.wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
    
    public default void updateInputs(WristIOInputs in) {}

    public default void setBrake(boolean brake) {}

    public default void setVoltage(double v) {}
    @AutoLog
    public class WristIOInputs{
        public double pos = 0.0;
        public double vel = 0.0;

        public double motorCurrentAmps = 0.0;
        public double motoroutVolts = 0.0;
        public double motorTemp = 0.0;
    }
}
