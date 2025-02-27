package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    
    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void moveOpenLoop(double voltage) {}

    public default void setNeutralMode(boolean brake) {}

    public default void resetposition(double posMeters) {}

    public default void setVoltage(double voltage) {}

    public default void setBrakeMode(boolean brake) {}
    
    public default void setVelocity(double velocityRPM) {}

    @AutoLog
    public class IntakeIOInputs {
        public double motorVoltage = 0.0;
        public double motorCurrent = 0.0;
        public double motorTemperature = 0.0;
        public double motorVelocityRPM = 0.0;
        public double motorPositionRotations = 0.0;
    }
}
