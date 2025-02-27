package frc.robot.subsystems.intake;
import static frc.robot.constants.IntakeConstants.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    
    private boolean useVelocityControl = false;
    
    public Intake(IntakeIO io) {
        this.io = io;
    }
    
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        if (DriverStation.isDisabled()) {
            stop();
        }
    }
    
    public void setVoltage(double volts) {
        useVelocityControl = false;
        io.setVoltage(volts);
    }

    public void stop() {
            setVoltage(0.0);
        
    }
    
    public void setBrakeMode(boolean enable) {
        io.setBrakeMode(enable);
    }
}
