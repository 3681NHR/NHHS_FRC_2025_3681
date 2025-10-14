package frc.robot.subsystems.affector.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    
    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void setVoltage(double voltage) {}

    public default void setBrake(boolean brake) {}

    public default void resetPos(double posMeters) {}

    @AutoLog
    public class ElevatorIOInputs{
        public double pos = 0;
        public double vel = 0;

        public double motor1CurrentAmps = 0;
        public double motor1Voltage = 0;
        public double motor1TempC = 0;

        public double motor2CurrentAmps = 0;
        public double motor2Voltage = 0;
        public double motor2TempC = 0;
    }
}
