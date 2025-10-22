package frc.robot.subsystems.intake;

import frc.utils.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;

import static frc.robot.constants.IntakeConstants.MOTOR_RUNNING_THRESHOLD;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

/**
 * Intake subsystem using a state machine and IO abstraction.
 * <p>
 * voltage is linearly related to speed, and there is no position control, so
 * the intake is controlled by voltage only
 */
public class Intake extends SubsystemBase {

    public enum WantedIntakeState {
        INTAKE, // same dir as score, but slower to account for sensor lag
        SCORE, // same dir as intake, but faster
        OUTTAKE, // opposite dir as intake and score, used only for L1
        OUTTAKE_SLOW,
        STOP,
        MANUAL// set voltage manually
    }

    public enum CurrentIntakeState {
        INTAKING,
        SCORING,
        OUTAKING,
        OUTAKING_SLOW,
        STOPPED,
        MANUAL
    }

    private WantedIntakeState wantedState = WantedIntakeState.STOP;
    private CurrentIntakeState currentState = CurrentIntakeState.STOPPED;
    private CurrentIntakeState previousState = CurrentIntakeState.STOPPED;

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    private boolean wasHolding = false;

    private LoggedNetworkBoolean holdLock = new LoggedNetworkBoolean("overrides/use intake sensor", true);
    private final Alert intakeSensorAlert = new Alert("Intake sensor disbled", Alert.AlertType.kError);

    public Intake(IntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        wasHolding = inputs.holding;

        // update alert
        intakeSensorAlert.set(!holdLock.get());

        // update and log IO inputs
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        previousState = currentState;

        stateTransition();
        applyStates();

        // logging
        Logger.recordOutput("Intake/previousState", previousState);
        Logger.recordOutput("Intake/currentState", currentState);
        Logger.recordOutput("Intake/wantedState", wantedState);

        Logger.recordOutput("Intake/CurrentCommand",
                getCurrentCommand() != null ? getCurrentCommand().getName() : "none");

        if (DriverStation.isDisabled()) {
            // stop intake if robot is disabled
            setWantedState(WantedIntakeState.STOP);
        }
    }

    /**
     * handles state transitions based on wanted state
     */
    public void stateTransition() {
        switch (wantedState) {
            case INTAKE:
                currentState = CurrentIntakeState.INTAKING;
                break;
            case SCORE:
                currentState = CurrentIntakeState.SCORING;
                break;
            case OUTTAKE:
                currentState = CurrentIntakeState.OUTAKING;
                break;
            case OUTTAKE_SLOW:
                currentState = CurrentIntakeState.OUTAKING_SLOW;
                break;
            case STOP:
                currentState = CurrentIntakeState.STOPPED;
                break;
            case MANUAL:
                currentState = CurrentIntakeState.MANUAL;
                break;
        }
    }

    /**
     * do logic based on current state
     */
    public void applyStates() {
        // set motor voltage to presets based on current state
        switch (currentState) {
            case INTAKING:
                io.setVoltage(IntakeConstants.SPEED_INTAKE);
                break;
            case SCORING:
                io.setVoltage(IntakeConstants.SPEED_SCORE);
                break;
            case OUTAKING:
                io.setVoltage(-IntakeConstants.SPEED_SCORE);
                break;
            case OUTAKING_SLOW:
                io.setVoltage(-IntakeConstants.SPEED_INTAKE);
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

    /**
     * get if the intake was holding a game piece last update
     */
    public boolean wasHolding() {
        return wasHolding;
    }

    /**
     * sets the motor brake mode, when brake mode is enabled, the motor will have
     * more resistance to being moved when no power is applied
     * 
     * @param enable if true, brake mode is enabled
     */
    public void setBrakeMode(boolean enable) {
        io.setBrakeMode(enable);
    }

    /**
     * get if the sensor is overriden
     * 
     * @return true if the sensor is enabled, false if it is overriden
     */
    public boolean getSensorEnabled() {
        return holdLock.get();
    }

    /**
     * get if the intake voltage is higher than a threshold
     * 
     * @return true if the intake is moving, false if it is not
     */
    public boolean isMoving() {
        return Math.abs(inputs.motorVoltage) > MOTOR_RUNNING_THRESHOLD;
    }

    /**
     * get if the intake is intaking
     * 
     * @return true if the intake is intaking, false if it is not moving or if it is
     *         outtaking
     */
    public boolean isIntaking() {
        return inputs.motorVoltage > MOTOR_RUNNING_THRESHOLD;
    }

    /**
     * set wanted state of the intake
     * 
     * @param state wanted state
     */
    public void setWantedState(WantedIntakeState state) {
        wantedState = state;
    }

    /**
     * set wanted state of the intake and set voltage if manual
     * 
     * @param state
     * @param volt
     */
    public void setWantedState(WantedIntakeState state, double volt) {
        wantedState = state;
        if (state == WantedIntakeState.MANUAL) {
            io.setVoltage(volt);
        }
    }

}
