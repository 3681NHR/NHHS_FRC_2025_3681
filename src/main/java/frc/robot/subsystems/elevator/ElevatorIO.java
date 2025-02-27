package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    
    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void setElevatorTargetLocation(double targetMeters) {}

    public default void moveElevatorOpenLoop(double voltage) {}

    public default void setElevatorNeutralMode(boolean brake) {}

    public default void resetElevatorPosition(double posMeters) {}

    @AutoLog
    public class ElevatorIOInputs{
        public double elevatorPositionMeters = 0;
        public double elevatorVelocityMetersPerSec = 0;

        public double motor1CurrentAmps = 0;
        public double motor1Voltage = 0;
        public double motor1TempC = 0;

        public double motor2CurrentAmps = 0;
        public double motor2Voltage = 0;
        public double motor2TempC = 0;
    }
}
