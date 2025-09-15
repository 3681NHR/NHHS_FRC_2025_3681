package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.ElevatorConstants.*;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.constants.AffectorPosition;
import frc.robot.constants.ElevatorConstants;
import frc.utils.ElevatorFF;
import frc.utils.ProfiledPID;

public class Elevator extends SubsystemBase {

    public enum WantedState{
        HOME,
        OFF,
        SYSID,
        POSITION
    }
    private enum CurrentState{
        HOME,
        OFF,
        SYSID,
        POSITION
    }

    private ElevatorIO io;
    private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private double posSet = 0.0;

    @AutoLogOutput(key="Elevator/IsHomed")
    private boolean homed = false;

    private ProfiledPID pid = new ProfiledPID(RobotBase.isReal() ? POS_PID : POS_PID_SIM);
    private ElevatorFF ff = new ElevatorFF(RobotBase.isReal() ? POS_FF : POS_FF_SIM);

    private Alert notHomed = new Alert("Elevator is not homed!", AlertType.kError);
    private Alert noLim = new Alert("Elevator limits not enforced", AlertType.kWarning);

    private LoggedNetworkBoolean limits = new LoggedNetworkBoolean("overrides/elevatorLimits", true);

    double volt = 0.0;

    private SysIdRoutine sysid;
    @AutoLogOutput
    private boolean brake = true;

    private WantedState wantedState = WantedState.OFF;
    private CurrentState currentState = CurrentState.OFF;
    private CurrentState previousState = CurrentState.OFF;

    private double pidOut = 0.0;
    private double ffOut = 0.0;

    private double zeroTimeStamp = Double.NaN;
    
    public Elevator(ElevatorIO io){
        this.io = io;

        sysid = new SysIdRoutine(new Config(
            VRAMP,
            VSTEP,
            TIMEOUT,
            (state) -> Logger.recordOutput("Elevator/SysIdState", state.toString())), 
            new SysIdRoutine.Mechanism(
            this::sysId, 
            null, 
            this));
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        previousState = currentState;

        stateTransitions();
        applyStates();

        Logger.recordOutput("Elevator/previousState", previousState.toString());
        Logger.recordOutput("Elevator/currentState", currentState.toString());
        Logger.recordOutput("Elevator/wantedState", wantedState.toString());

        if(DriverStation.isDisabled()){
            wantedState = WantedState.OFF;
            posSet = inputs.elevatorPositionMeters;
        }

        Logger.recordOutput("Elevator/CurrentCommand", getCurrentCommand() != null ? getCurrentCommand().getName() : "none");

        
        pidOut = pid.calculate(inputs.elevatorPositionMeters, posSet);
        ffOut = ff.calculate(pid.getSetpoint().velocity);
        
        Logger.recordOutput("Elevator/Control/PID goal", posSet);
        Logger.recordOutput("Elevator/Control/PID setpoint pos", pid.getSetpoint().position);
        Logger.recordOutput("Elevator/Control/PID setpoint vel", pid.getSetpoint().velocity);
        Logger.recordOutput("Elevator/Control/PID applied", pidOut);
        Logger.recordOutput("Elevator/Control/FF aplied", ffOut);

        
        io.setVoltage(volt);
        
        notHomed.set(!homed);
        noLim.set(!homed || currentState == CurrentState.HOME || currentState == CurrentState.SYSID || !limits.get());
    }

    private void stateTransitions(){
        switch (wantedState) {
            case HOME:
                currentState = CurrentState.HOME;
            break;
            case OFF:
                currentState = CurrentState.OFF;
            break;
            case SYSID:
                currentState = CurrentState.SYSID;
            break;
            case POSITION:
                currentState = CurrentState.POSITION;
            break;
        }
    }
    private void applyStates(){
        switch (currentState) {
            case HOME:
                if(previousState != CurrentState.HOME){
                    homed = false;
                }
                if(homed){
                    volt = 0;
                    resetPos(HOME_POS);
                    setWantedState(WantedState.POSITION, new AffectorPosition(HOME_POS, 0.0));
                } else {
                    volt = HOME_VOLTAGE;
                    if (Math.abs(getVelocity()) < ElevatorConstants.HOME_MIN_VEL) {
                        if (!Double.isFinite(zeroTimeStamp)) {
                            zeroTimeStamp = Logger.getTimestamp();
                        } else {
                            homed = Logger.getTimestamp() - zeroTimeStamp >= Units.secondsToMilliseconds(ElevatorConstants.HOME_STOP_TIME);
                        }
                    } else {
                        zeroTimeStamp = Double.NaN;
                    }
                }
            break;
            case OFF:
                volt = 0;
            break;
            case SYSID:
            break;
            case POSITION:
                if(!homed && inputs.elevatorPositionMeters < 0){
                    io.resetElevatorPosition(0);
                }
                if(homed && limits.get()){
                    posSet = MathUtil.clamp(posSet, MIN_POS, MAX_POS);
                }
                volt = pidOut + ffOut;
            break;
        }
    }

    public void setHomed(boolean homed){
        this.homed = homed;
    }
    public boolean isHomed(){
        return homed;
    }

    public void setVoltage(double voltage){
        if(currentState == CurrentState.HOME || currentState == CurrentState.SYSID){
            volt = voltage;
        }
    }

    public double getPositionSet(){
        return posSet;
    }
    public double getPosition(){
        return inputs.elevatorPositionMeters;
    }
    public double getVelocity() {
        return inputs.elevatorVelocityMetersPerSec;
    }
    public void resetPos(double pos){
        io.resetElevatorPosition(pos);
    }

    public boolean getBrake(){
        return brake;
    }
    public void setBrake(boolean brake){
        io.setElevatorNeutralMode(brake);
        this.brake = brake;
    }
    
    public boolean atSetpoint(){
        return Math.abs(inputs.elevatorPositionMeters - posSet) < POS_TOLERANCE;
    }

    public void sysId(Voltage v){
        setVoltage(v.in(Volts));
    }

    public Pose3d getAScopePoseInnerStage(double pos){
        return new Pose3d(new Translation3d(0, 0, pos), new Rotation3d());
    }
    public Pose3d getAScopePoseMiddleStage(double pos){
        return new Pose3d(new Translation3d(0, 0, pos*0.544561), new Rotation3d());
    }

    public void stop(){
        setWantedState(WantedState.POSITION, new AffectorPosition(getPosition(), 0.0));
    }
    /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    setWantedState(WantedState.SYSID);
    return sysid.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    setWantedState(WantedState.SYSID);
    return  sysid.dynamic(direction);
  }

  public void setWantedState(WantedState w){
    wantedState = w;
  }
  public void setWantedState(WantedState w, AffectorPosition pos){
    wantedState = w;
    posSet = pos.elev;
  }
}
