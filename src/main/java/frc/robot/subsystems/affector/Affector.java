package frc.robot.subsystems.affector;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.OperatorConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.WristConstants;
import frc.robot.subsystems.affector.elevator.ElevatorIO;
import frc.robot.subsystems.affector.wrist.WristIO;
import frc.robot.subsystems.affector.elevator.ElevatorIOInputsAutoLogged;
import frc.robot.subsystems.affector.wrist.WristIOInputsAutoLogged;
import frc.utils.ArmFF;
import frc.utils.ElevatorFF;
import frc.utils.ExtraMath;
import frc.utils.ProfiledPID;

/**
 * subsystem to handle elevator and wrist control using a state machine and IO
 * abstraction for hardware control and logging
 */
public class Affector extends SubsystemBase {

    public static class AffectorPosition {
        public double elev;
        public double wrist;

        public AffectorPosition(double elev, double wrist) {
            this.elev = elev;
            this.wrist = wrist;
        }
    }

    public enum WantedAffectorState {
        HOME, // homing elevator
        OFF, // stop everything
        SYSID, // system identification mode
        POSITION// position control
    }

    private enum CurrentAffectorState {
        HOME,
        OFF,
        SYSID,
        POSITION
    }

    private WantedAffectorState wantedState = WantedAffectorState.OFF;
    private CurrentAffectorState currentState = CurrentAffectorState.OFF;
    private CurrentAffectorState previousState = CurrentAffectorState.OFF;

    private ElevatorIO elevIO;
    private ElevatorIOInputsAutoLogged elevInputs = new ElevatorIOInputsAutoLogged();
    private double elevPosSet = 0.0;

    private WristIO wristIO;
    private WristIOInputsAutoLogged wristInputs = new WristIOInputsAutoLogged();
    private double wristPosSet = 0.0;

    @AutoLogOutput(key = "Affector/Elevator/Homing/IsHomed")
    public boolean elevHomed = false;

    private ProfiledPID elevPID = new ProfiledPID(
            RobotBase.isReal() ? ElevatorConstants.POS_PID : ElevatorConstants.POS_PID_SIM);
    private ElevatorFF elevFF = new ElevatorFF(
            RobotBase.isReal() ? ElevatorConstants.POS_FF : ElevatorConstants.POS_FF_SIM);

    private ProfiledPID wristPID = new ProfiledPID(
            RobotBase.isReal() ? WristConstants.POS_PID : WristConstants.POS_PID_SIM);
    private ArmFF wristFF = new ArmFF(RobotBase.isReal() ? WristConstants.POS_FF : WristConstants.POS_FF_SIM);

    private Alert elevNotHomed = new Alert("Elevator is not homed!", AlertType.kError);
    private Alert elevNoLim = new Alert("Elevator limits not enforced", AlertType.kWarning);

    private Alert wristNoLim = new Alert("Wrist limits not enforced", AlertType.kWarning);

    private LoggedNetworkBoolean elevLimitOverride = new LoggedNetworkBoolean("overrides/elevatorLimits", true);
    private LoggedNetworkBoolean wristLimitOverride = new LoggedNetworkBoolean("overrides/wristLimits", true);

    double elevVout = 0.0;
    double wristVout = 0.0;

    private SysIdRoutine elevSysID;
    private SysIdRoutine wristSysID;

    @AutoLogOutput(key = "Affector/Elevator/BrakeEnabled")
    private boolean elevBrake = true;

    @AutoLogOutput(key = "Affector/Wrist/BrakeEnabled")
    private boolean wristBrake = true;

    private double elevPIDOut = 0.0;
    private double elevFFOut = 0.0;

    private double wristPIDOut = 0.0;
    private double wristFFOut = 0.0;

    @AutoLogOutput(key = "Affector/Elevator/Homing/Timestamp")
    private double homingZeroTimeStamp = Double.NaN;

    private XboxController operatorController;

    public Affector(ElevatorIO elevio, WristIO wristio, XboxController controller) {
        this.elevIO = elevio;
        this.wristIO = wristio;
        this.operatorController = controller;

        // configure sysid
        elevSysID = new SysIdRoutine(new Config(
                ElevatorConstants.VRAMP,
                ElevatorConstants.VSTEP,
                ElevatorConstants.TIMEOUT,
                (state) -> Logger.recordOutput("Affector/ElevSysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                        this::sysId,
                        null,
                        this));
        wristSysID = new SysIdRoutine(new Config(
                WristConstants.VRAMP,
                WristConstants.VSTEP,
                WristConstants.TIMEOUT,
                (state) -> Logger.recordOutput("Affector/WristSysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                        this::sysId,
                        null,
                        this));
    }

    @Override
    public void periodic() {
        // get and log inputs from elevator and wristIO
        elevIO.updateInputs(elevInputs);
        Logger.processInputs("Affector/Elevator", elevInputs);

        wristIO.updateInputs(wristInputs);
        Logger.processInputs("Affector/Wrist", wristInputs);

        if (DriverStation.isDisabled()) {
            // ensure no sudden motion on enable
            setWantedState(WantedAffectorState.POSITION, getPosition());
        }

        previousState = currentState;

        stateTransitions();
        applyStates();

        // calculate PID and FF outputs
        elevPIDOut = elevPID.calculate(elevInputs.pos, elevPosSet);
        elevFFOut = elevFF.calculate(elevPID.getSetpoint().velocity);

        wristPIDOut = wristPID.calculate(wristInputs.pos, wristPosSet);
        wristFFOut = wristFF.calculate(wristInputs.pos, wristPID.getSetpoint().velocity);

        // log outputs
        Logger.recordOutput("Affector//previousState", previousState.toString());
        Logger.recordOutput("Affector//currentState", currentState.toString());
        Logger.recordOutput("Affector//wantedState", wantedState.toString());
        Logger.recordOutput("Affector//CurrentCommand",
                getCurrentCommand() != null ? getCurrentCommand().getName() : "none");

        Logger.recordOutput("Affector/Elevator/Control/PID goal", elevPosSet);
        Logger.recordOutput("Affector/Elevator/Control/PID setpoint pos", elevPID.getSetpoint().position);
        Logger.recordOutput("Affector/Elevator/Control/PID setpoint vel", elevPID.getSetpoint().velocity);
        Logger.recordOutput("Affector/Elevator/Control/PID applied", elevPIDOut);
        Logger.recordOutput("Affector/Elevator/Control/FF aplied", elevFFOut);

        Logger.recordOutput("Affector/Wrist/Control/PID goal", wristPosSet);
        Logger.recordOutput("Affector/Wrist/Control/PID setpoint pos", wristPID.getSetpoint().position);
        Logger.recordOutput("Affector/Wrist/Control/PID setpoint vel", wristPID.getSetpoint().velocity);
        Logger.recordOutput("Affector/Wrist/Control/PID applied", wristPIDOut);
        Logger.recordOutput("Affector/Wrist/Control/FF aplied", wristFFOut);

        // send voltage commands to IOs tp move elevator and wrist
        elevIO.setVoltage(elevVout);
        wristIO.setVoltage(wristVout);

        // alerts
        elevNotHomed.set(!elevHomed);
        elevNoLim.set(!elevHomed || currentState == CurrentAffectorState.HOME
                || currentState == CurrentAffectorState.SYSID || !elevLimitOverride.get());

        wristNoLim.set(currentState == CurrentAffectorState.HOME || currentState == CurrentAffectorState.SYSID
                || !wristLimitOverride.get());
    }

    /**
     * detemines the current state based on the wanted state
     */
    private void stateTransitions() {
        switch (wantedState) {
            case HOME:
                currentState = CurrentAffectorState.HOME;
                break;
            case OFF:
                currentState = CurrentAffectorState.OFF;
                break;
            case SYSID:
                currentState = CurrentAffectorState.SYSID;
                break;
            case POSITION:
                currentState = CurrentAffectorState.POSITION;
                break;
        }
    }

    /**
     * applies the logic for each state
     */
    private void applyStates() {
        // reset PIDs when entering position state to prevent windup or unwanted motion
        if (currentState == CurrentAffectorState.POSITION && previousState != CurrentAffectorState.POSITION) {
            elevPID.reset(elevInputs.pos, elevInputs.vel);
            wristPID.reset(wristInputs.pos, wristInputs.vel);
        }
        // set conditions for homing
        if (currentState == CurrentAffectorState.HOME && previousState != CurrentAffectorState.HOME) {
            elevHomed = false;
            homingZeroTimeStamp = Double.NaN;
        }
        switch (currentState) {
            case HOME:
                if (elevHomed) {
                    // homing complete, reset position and enter position mode
                    homingZeroTimeStamp = Double.NaN;
                    elevVout = 0;
                    elevIO.resetPos(ElevatorConstants.HOME_POS);
                    setWantedState(WantedAffectorState.POSITION,
                            new AffectorPosition(ElevatorConstants.HOME_POS, Constants.Affector.STOW_POSITION.wrist));
                } else {
                    // still homing, movef elevator down
                    elevVout = ElevatorConstants.HOME_VOLTAGE;
                    if (Math.abs(getVelocity().elev) < ElevatorConstants.HOME_MIN_VEL) {
                        // elevator has stopped, assume it has hit the bottom stop
                        if (!Double.isFinite(homingZeroTimeStamp)) {
                            homingZeroTimeStamp = Logger.getTimestamp();
                        } else {
                            Logger.recordOutput("Affector/Elevator/Homing/time",
                                    (Logger.getTimestamp() - homingZeroTimeStamp) / 1000000);
                            // require elevator fully stopped for some time to ensure it has hit a stop
                            elevHomed = Logger.getTimestamp() - homingZeroTimeStamp >= ElevatorConstants.HOME_STOP_TIME
                                    * 1000000;

                        }
                    } else {
                        // still moving
                        homingZeroTimeStamp = Double.NaN;
                        Logger.recordOutput("Affector/Elevator/Homing/time", Double.NaN);
                    }
                }
                break;
            case OFF:
                elevVout = 0;
                wristVout = 0;
                break;
            case SYSID:
                // sysid is handled by a command, do nothing in sysid state
                break;
            case POSITION:
                // position control
                wristPosSet += ExtraMath.processInput(operatorController.getRightY(),
                        -0.02 * WristConstants.POS_PID.maxSpeed(), 1.0, 0.05);
                elevPosSet += (operatorController.getRightTriggerAxis() - operatorController.getLeftTriggerAxis())
                        * OperatorConstants.ELEVATOR_MAN_SENS;

                // enforce software limit unless override is used
                if (elevHomed && elevLimitOverride.get()) {
                    elevPosSet = MathUtil.clamp(elevPosSet, ElevatorConstants.MIN_POS, ElevatorConstants.MAX_POS);
                }
                if (wristLimitOverride.get()) {
                    wristPosSet = MathUtil.clamp(wristPosSet, WristConstants.MIN_POS, WristConstants.MAX_POS);
                }
                // set output voltage to PIDF output
                elevVout = elevPIDOut + elevFFOut;
                wristVout = wristPIDOut + wristFFOut;
                break;
        }
    }

    /**
     * gets if the elevator has been homed
     * 
     * @return true if elevator is homed, false if not
     */
    public boolean isElevHomed() {
        return elevHomed;
    }

    /**
     * set elevator voltage, only works in home or sysid states
     * 
     * @param voltage voltage to set elevator motors to in volts
     */
    public void setElevVoltage(double voltage) {
        // only set voltage if in a state that allows it
        if (currentState == CurrentAffectorState.HOME || currentState == CurrentAffectorState.SYSID) {
            elevVout = voltage;
        }
    }

    /**
     * set wrist voltage, only works in sysid state
     * 
     * @param voltage voltage to set wrist motors to in volts
     */
    public void setWristVoltage(double voltage) {
        if (currentState == CurrentAffectorState.SYSID) {
            wristVout = voltage;
        }
    }

    /**
     * get the current set position of the affector
     * 
     * @return AffectorPosition object with current set positions
     */
    public AffectorPosition getPositionSet() {
        return new AffectorPosition(elevPosSet, wristPosSet);
    }

    /**
     * get the current position of the affector
     * 
     * @return AffectorPosition object with current positions
     */
    public AffectorPosition getPosition() {
        return new AffectorPosition(elevInputs.pos, wristInputs.pos);
    }

    /**
     * get the current velocity of the affector
     * 
     * @return AffectorPosition object with current velocities
     */
    public AffectorPosition getVelocity() {
        return new AffectorPosition(elevInputs.vel, wristInputs.vel);
    }

    /**
     * get if the elevator motors are in brake mode
     * 
     * @return true if brake mode is enabled, false if in coast mode
     */
    public boolean getElevBrake() {
        return elevBrake;
    }

    /**
     * set elevator brake mode, recomended to use true unless manual movement is
     * needed
     * 
     * @param brake true to enable brake mode, false to set to coast mode
     */
    public void setElevBrake(boolean brake) {
        elevIO.setBrake(brake);
        this.elevBrake = brake;
    }

    /**
     * get if the wrist motor is in brake mode
     * 
     * @return true if brake mode is enabled, false if in coast mode
     */
    public boolean getWristBrake() {
        return wristBrake;
    }

    /**
     * set wrist brake mode, recomended to use true unless manual movement is needed
     * 
     * @param brake true to enable brake mode, false to set to coast mode
     */
    public void setWristBrake(boolean brake) {
        wristIO.setBrake(brake);
        this.wristBrake = brake;
    }

    /**
     * get if the elevator and wrist are within tolerance of their setpoints
     * 
     * @return true if both elevator and wrist are within tolerance, false if either
     *         is out of tolerance
     */
    public boolean atSetpoint() {
        return Math.abs(elevInputs.pos - elevPosSet) < ElevatorConstants.POS_TOLERANCE
                && Math.abs(wristInputs.pos - wristPosSet) < WristConstants.POS_TOLERANCE;
    }

    /**
     * method to be used by sysid command to set elevator voltage
     * 
     * @param v voltage to set elevator motors
     */
    public void sysId(Voltage v) {
        setElevVoltage(v.in(Volts));
    }

    /**
     * calculate the pose of the innermost stage of the elevator
     * 
     * @param pos elevator position
     * @return {@link Pose3d} of the innermost stage of the elevator
     */
    public Pose3d calculatePoseElevInnerStage(double pos) {
        return new Pose3d(new Translation3d(0, 0, pos), new Rotation3d());
    }

    /**
     * calculate the pose of the middle stage of the elevator
     * 
     * @param pos elevator position
     * @return {@link Pose3d} of the middle stage of the elevator
     */
    public Pose3d calculatePoseElevMiddleStage(double pos) {
        return new Pose3d(new Translation3d(0, 0, pos * 0.544561), new Rotation3d());
    }

    /**
     * colculate the pose of the wrist based on elevator and wrist position
     * 
     * @param pos            wrist angle
     * @param elevatorHeight elevator position
     * @return {@link Pose3d} of the wrist
     */
    public Pose3d calculatePoseWrist(double pos, double elevatorHeight) {
        return new Pose3d(WristConstants.WRIST_POS.plus(new Translation3d(0, 0, elevatorHeight)),
                new Rotation3d(pos - (Math.PI / 2.0), 0, 0));
    }

    /**
     * stop elevator and wrist, enters position mode and holds current position
     */
    public void stop() {
        setWantedState(WantedAffectorState.POSITION, getPosition());
    }

    /**
     * set the wanted state of the affector
     * 
     * @param w wanted state
     */
    public void setWantedState(WantedAffectorState w) {
        wantedState = w;
    }

    /**
     * set the wanted state of the affector and the position to move to if in
     * position mode
     * 
     * @param w   wanted state
     * @param pos target position - only works in position mode
     */
    public void setWantedState(WantedAffectorState w, AffectorPosition pos) {
        wantedState = w;
        elevPosSet = pos.elev;
        wristPosSet = pos.wrist;
    }

    /**
     * set if the elevator has been homed
     * <p>
     * this does not home the elevator, it only sets the state
     * 
     * @param homed true if elevator is homed, false if not
     */
    public void setElevHomed(boolean homed) {
        elevHomed = homed;
    }

    /**
     * tare the elevator position to a given position
     * 
     * @param pos current position of the elevator
     */
    public void resetElevPos(double pos) {
        elevIO.resetPos(pos);
    }

    /** Returns a command to run a quasistatic test in the specified direction. */
    public Command elevSysIdQuasistatic(SysIdRoutine.Direction direction) {
        setWantedState(WantedAffectorState.SYSID);
        return elevSysID.quasistatic(direction);
    }

    /** Returns a command to run a dynamic test in the specified direction. */
    public Command elevSysIdDynamic(SysIdRoutine.Direction direction) {
        setWantedState(WantedAffectorState.SYSID);
        return elevSysID.dynamic(direction);
    }

    public Command wristSysIdQuasistatic(SysIdRoutine.Direction direction) {
        setWantedState(WantedAffectorState.SYSID);
        return wristSysID.quasistatic(direction);
    }

    /** Returns a command to run a dynamic test in the specified direction. */
    public Command wristSysIdDynamic(SysIdRoutine.Direction direction) {
        setWantedState(WantedAffectorState.SYSID);
        return wristSysID.dynamic(direction);
    }

}
