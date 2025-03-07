package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.constants.IntakeConstants.MOTOR_RUNNING_THRESHOLD;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    
    private LoggedNetworkBoolean holdLock = new LoggedNetworkBoolean("overrides/holdLock", true);
    
    public Intake(IntakeIO io) {
        this.io = io;
    }
    
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        Logger.recordOutput("Intake/CurrentCommand", getCurrentCommand() != null ? getCurrentCommand().getName() : "none");

        if (DriverStation.isDisabled()) {
            stop();
        }
    }
    
    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }

    public void stop() {
        setVoltage(0.0);
    }
    public boolean isHolding() {
        return inputs.holding;
    }
    
    public void setBrakeMode(boolean enable) {
        io.setNeutralMode(enable);
    }
    public boolean getHoldLock() {
        return holdLock.get();
    }
    public boolean isMoving(){
        return Math.abs(inputs.motorVoltage) > MOTOR_RUNNING_THRESHOLD;
    }
    public boolean isIntaking(){
        return inputs.motorVoltage > MOTOR_RUNNING_THRESHOLD;
    }
}
