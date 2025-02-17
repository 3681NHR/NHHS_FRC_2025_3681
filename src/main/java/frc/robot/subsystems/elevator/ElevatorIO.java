package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    
    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void setTargetLocation(double targetMeters) {}

    public default void moveOpenLoop(double voltage) {}

    public default void setNutralMode(boolean brake) {}

    public default void resetposition(double posMeters) {}

    @AutoLog
    public class ElevatorIOInputs{
        public double positionMeters = 0;
        public double velocityMetersPerSec = 0;

        public double motor1CurrentAmps = 0;
        public double motor1Voltage = 0;
        public double motor1TempC = 0;

        public double motor2Current = 0;
        public double motor2Voltage = 0;
        public double motor2TempC = 0;

    }
}
