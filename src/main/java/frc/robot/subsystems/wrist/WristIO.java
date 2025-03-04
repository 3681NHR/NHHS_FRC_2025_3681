package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
    
    public default void updateInputs(WristIOInputs in) {}

    public default void setPos(double posRad) {}

    public default void setBrake(boolean brake) {}

    public default void setVoltage(double v) {}
    @AutoLog
    public class WristIOInputs{
        public double posRad = 0.0;
        public double velRadPerSec = 0.0;

        public double motorCurrentAmps = 0.0;
        public double motoroutVolts = 0.0;
        public double motorTemp = 0.0;
    }
}
