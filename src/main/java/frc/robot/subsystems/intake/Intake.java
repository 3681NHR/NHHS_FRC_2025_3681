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
    
    public void setVelocity(double rpm) {
        useVelocityControl = true;
        io.setVelocity(rpm);
    }
    public void stop() {
        if (useVelocityControl) {
            setVelocity(0.0);
        } else {
            setVoltage(0.0);
        }
    }
    
    public void setBrakeMode(boolean enable) {
        io.setBrakeMode(enable);
    }
    
    public double getVelocityRPM() {
        return inputs.motorVelocityRPM;
    }
    
    public double getVoltage() {
        return inputs.motorVoltage;
    }
    public double getCurrent() {
        return inputs.motorCurrent;
    }
    public boolean isRunning() {
        return Math.abs(inputs.motorVelocityRPM) > MOTOR_RUNNING_THRESHOLD;
    }
}
