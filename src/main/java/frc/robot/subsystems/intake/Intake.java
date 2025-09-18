package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;

import static frc.robot.constants.IntakeConstants.MOTOR_RUNNING_THRESHOLD;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class Intake extends SubsystemBase {

    public enum WantedIntakeState {
        INTAKE,
        OUTTAKE,
        STOP,
        MANUAL
    }
    public enum CurrentIntakeState {
        INTAKING,
        OUTAKING,
        STOPPED,
        MANUAL
    }
    private WantedIntakeState wantedState = WantedIntakeState.STOP;
    private CurrentIntakeState currentState = CurrentIntakeState.STOPPED;
    private CurrentIntakeState previousState = CurrentIntakeState.STOPPED;

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

        previousState = currentState;

        stateTransition();
        applyStates();

        Logger.recordOutput("Intake/previousState", previousState);
        Logger.recordOutput("Intake/currentState", currentState);
        Logger.recordOutput("Intake/wantedState", wantedState);


        Logger.recordOutput("Intake/CurrentCommand", getCurrentCommand() != null ? getCurrentCommand().getName() : "none");

        if (DriverStation.isDisabled()) {
            setWantedState(WantedIntakeState.STOP);
        }
    }

    public void stateTransition(){
        switch(wantedState){
            case INTAKE:
                currentState = CurrentIntakeState.INTAKING;
                break;
            case OUTTAKE:
                currentState = CurrentIntakeState.OUTAKING;
                break;
            case STOP:
                currentState = CurrentIntakeState.STOPPED;
                break;
            case MANUAL:
                currentState = CurrentIntakeState.MANUAL;
                break;
        }
    }
    public void applyStates(){
        switch (currentState) {
            case INTAKING:
            io.setVoltage(IntakeConstants.SPEED);
            break;
            case OUTAKING:
            io.setVoltage(-IntakeConstants.SPEED);
            break;
            case STOPPED:
                io.setVoltage(0);
            break;
            case MANUAL:
            break;
        }
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
    public void setWantedState(WantedIntakeState state){
        wantedState = state;
    }
    public void setWantedState(WantedIntakeState state, double volt){
        wantedState = state;
        if(state == WantedIntakeState.MANUAL){
            io.setVoltage(volt);
        }
    }
}
