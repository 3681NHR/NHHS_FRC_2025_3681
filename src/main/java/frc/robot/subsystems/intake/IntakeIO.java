package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public class IntakeIOInputs {
        public double velocityMetersPerSec = 0;

        public double motorCurrentAmps = 0;
        public double motorVoltage = 0;
        public double motorTempC = 0;
    }
}
