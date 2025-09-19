package frc.robot.subsystems.affector;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
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
import frc.robot.constants.AffectorPosition;
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

public class Affector extends SubsystemBase {

    public enum WantedAffectorState{
        HOME,
        OFF,
        SYSID,
        POSITION
    }
    private enum CurrentAffectorState{
        HOME,
        OFF,
        SYSID,
        POSITION
    }

    private WantedAffectorState  wantedState   =  WantedAffectorState.OFF;
    private CurrentAffectorState currentState  = CurrentAffectorState.OFF;
    private CurrentAffectorState previousState = CurrentAffectorState.OFF;

    private ElevatorIO elevIO;
    private ElevatorIOInputsAutoLogged elevInputs = new ElevatorIOInputsAutoLogged();
    private double elevPosSet = 0.0;

    private WristIO wristIO;
    private WristIOInputsAutoLogged wristInputs = new WristIOInputsAutoLogged();
    private double wristPosSet = 0.0;

    @AutoLogOutput(key="Elevator/IsHomed")
    private boolean elevHomed = false;

    private ProfiledPID elevPID = new ProfiledPID(RobotBase.isReal() ? ElevatorConstants.POS_PID : ElevatorConstants.POS_PID_SIM);
    private ElevatorFF elevFF = new ElevatorFF(RobotBase.isReal() ? ElevatorConstants.POS_FF : ElevatorConstants.POS_FF_SIM);

    private ProfiledPID wristPID = new ProfiledPID(RobotBase.isReal() ? WristConstants.POS_PID : WristConstants.POS_PID_SIM);
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

    @AutoLogOutput(key="Affector/Elevator/BrakeEnabled")
    private boolean elevBrake = true;

    @AutoLogOutput(key="Affector/Wrist/BrakeEnabled")
    private boolean wristBrake = true;


    private double elevPIDOut = 0.0;
    private double elevFFOut = 0.0;

    private double wristPIDOut = 0.0;
    private double wristFFOut = 0.0;

    private double homingZeroTimeStamp = Double.NaN;

    private XboxController operatorController;
    
    public Affector(ElevatorIO elevio, WristIO wristio, XboxController controller){
        this.elevIO = elevio;
        this.wristIO = wristio;
        this.operatorController = controller;

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
    public void periodic(){
        elevIO.updateInputs(elevInputs);
        Logger.processInputs("Affector/Elevator", elevInputs);

        wristIO.updateInputs(wristInputs);
        Logger.processInputs("Affector/Wrist", wristInputs);

        if(DriverStation.isDisabled()){
            setWantedState(WantedAffectorState.POSITION, getPosition());
        }

        previousState = currentState;

        stateTransitions();
        applyStates();
        
        
        elevPIDOut = elevPID.calculate(elevInputs.pos, elevPosSet);
        elevFFOut = elevFF.calculate(elevPID.getSetpoint().velocity);

        wristPIDOut = wristPID.calculate(wristInputs.pos, wristPosSet);
        wristFFOut = wristFF.calculate(wristInputs.pos, wristPID.getSetpoint().velocity);
        
        Logger.recordOutput("Affector//previousState", previousState.toString());
        Logger.recordOutput("Affector//currentState", currentState.toString());
        Logger.recordOutput("Affector//wantedState", wantedState.toString());
        Logger.recordOutput("Affector//CurrentCommand", getCurrentCommand() != null ? getCurrentCommand().getName() : "none");

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

        elevIO.setVoltage(elevVout);
        wristIO.setVoltage(wristVout);
        
        elevNotHomed.set(!elevHomed);
        elevNoLim.set(!elevHomed || currentState == CurrentAffectorState.HOME || currentState == CurrentAffectorState.SYSID || !elevLimitOverride.get());

        wristNoLim.set(currentState == CurrentAffectorState.HOME || currentState == CurrentAffectorState.SYSID || !wristLimitOverride.get());
    }

    private void stateTransitions(){
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
    private void applyStates(){
        if(currentState == CurrentAffectorState.POSITION && previousState != CurrentAffectorState.POSITION){
            elevPID.reset(elevInputs.pos, elevInputs.vel);
            wristPID.reset(wristInputs.pos, wristInputs.vel);
        }
        switch (currentState) {
            case HOME:
                if(previousState != CurrentAffectorState.HOME){
                    elevHomed = false;
                }
                if(elevHomed){
                    elevVout = 0;
                    elevIO.resetPos(ElevatorConstants.HOME_POS);
                    setWantedState(WantedAffectorState.POSITION, new AffectorPosition(ElevatorConstants.HOME_POS, Constants.Affector.STOW_POSITION.wrist));
                } else {
                    elevVout = ElevatorConstants.HOME_VOLTAGE;
                    if (Math.abs(getVelocity().elev) < ElevatorConstants.HOME_MIN_VEL) {
                        if (!Double.isFinite(homingZeroTimeStamp)) {
                            homingZeroTimeStamp = Logger.getTimestamp();
                        } else {
                            elevHomed = Logger.getTimestamp() - homingZeroTimeStamp >= Units.secondsToMilliseconds(ElevatorConstants.HOME_STOP_TIME);
                        }
                    } else {
                        homingZeroTimeStamp = Double.NaN;
                    }
                }
            break;
            case OFF:
                elevVout = 0;
                wristVout = 0;
            break;
            case SYSID:
            break;
            case POSITION:

            wristPosSet += ExtraMath.processInput(operatorController.getRightY(), -0.02 * WristConstants.POS_PID.maxSpeed(), 1.0, 0.05);
            elevPosSet += (operatorController.getRightTriggerAxis()-operatorController.getLeftTriggerAxis())*OperatorConstants.ELEVATOR_MAN_SENS;
           
            // if(!elevHomed && elevInputs.pos < 0){
            //         elevIO.resetPos(0);
            // }
            if(elevHomed && elevLimitOverride.get()){
                elevPosSet = MathUtil.clamp(elevPosSet, ElevatorConstants.MIN_POS, ElevatorConstants.MAX_POS);
            }
            if(wristLimitOverride.get()){
                wristPosSet = MathUtil.clamp(wristPosSet, WristConstants.MIN_POS, WristConstants.MAX_POS);
            }
            elevVout = elevPIDOut + elevFFOut;
            wristVout = wristPIDOut + wristFFOut;
            break;
        }
    }

    public boolean isElevHomed(){
        return elevHomed;
    }

    public void setElevVoltage(double voltage){
        if(currentState == CurrentAffectorState.HOME || currentState == CurrentAffectorState.SYSID){
            elevVout = voltage;
        }
    }
    public void setWristVoltage(double voltage){
        if(currentState == CurrentAffectorState.SYSID){
            wristVout = voltage;
        }
    }

    public AffectorPosition getPositionSet(){
        return new AffectorPosition(elevPosSet, wristPosSet);
    }
    public AffectorPosition getPosition(){
        return new AffectorPosition(elevInputs.pos, wristInputs.pos);
    }
    public AffectorPosition getVelocity() {
        return new AffectorPosition(elevInputs.vel, wristInputs.vel);
    }
    public boolean getElevBrake(){
        return elevBrake;
    }
    public void setElevBrake(boolean brake){
        elevIO.setBrake(brake);
        this.elevBrake = brake;
    }
    public boolean getWristBrake(){
        return wristBrake;
    }
    public void setWristBrake(boolean brake){
        wristIO.setBrake(brake);
        this.wristBrake = brake;
    }
    
    public boolean atSetpoint(){
        return Math.abs(elevInputs.pos - elevPosSet) < ElevatorConstants.POS_TOLERANCE && Math.abs(wristInputs.pos - wristPosSet) < WristConstants.POS_TOLERANCE;
    }

    public void sysId(Voltage v){
        setElevVoltage(v.in(Volts));
    }

    public Pose3d calculatePoseElevInnerStage(double pos){
        return new Pose3d(new Translation3d(0, 0, pos), new Rotation3d());
    }
    public Pose3d calculatePoseElevMiddleStage(double pos){
        return new Pose3d(new Translation3d(0, 0, pos*0.544561), new Rotation3d());
    }
    public Pose3d calculatePoseWrist(double pos, double elevatorHeight){
        return new Pose3d(WristConstants.WRIST_POS.plus(new Translation3d(0, 0, elevatorHeight)), new Rotation3d(pos - (Math.PI/2.0), 0, 0));
    }

    public void stop(){
        setWantedState(WantedAffectorState.POSITION, getPosition());
    }

    public void setWantedState(WantedAffectorState w){
        wantedState = w;
    }
    public void setWantedState(WantedAffectorState w, AffectorPosition pos){
        wantedState = w;
        elevPosSet = pos.elev;
        wristPosSet = pos.wrist;
    }

    public void setElevHomed(boolean homed){
        elevHomed = homed;
    }
    public void resetElevPos(double pos){
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
