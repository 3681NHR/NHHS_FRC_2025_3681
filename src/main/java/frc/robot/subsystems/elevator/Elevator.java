package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.ElevatorConstants.MAX_POS;
import static frc.robot.constants.ElevatorConstants.MIN_POS;
import static frc.robot.constants.ElevatorConstants.POS_FF;
import static frc.robot.constants.ElevatorConstants.POS_FF_SIM;
import static frc.robot.constants.ElevatorConstants.POS_PID;
import static frc.robot.constants.ElevatorConstants.POS_PID_SIM;
import static frc.robot.constants.ElevatorConstants.POS_TOLERANCE;
import static frc.robot.constants.ElevatorConstants.TIMEOUT;
import static frc.robot.constants.ElevatorConstants.VRAMP;
import static frc.robot.constants.ElevatorConstants.VSTEP;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.utils.ElevatorFF;
import frc.utils.ProfiledPID;

public class Elevator extends SubsystemBase {

    private ElevatorIO io;
    private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private double posSet = 0.0;

    @AutoLogOutput(key="Elevator/IsHomed")
    private boolean homed = false;
    @AutoLogOutput(key="Elevator/Openloop")
    private boolean openloop = false;

    private ProfiledPID pid = new ProfiledPID(RobotBase.isReal() ? POS_PID : POS_PID_SIM);
    private ElevatorFF ff = new ElevatorFF(RobotBase.isReal() ? POS_FF : POS_FF_SIM);

    private Alert notHomed = new Alert("Elevator is not homed!", AlertType.kError);
    private Alert noLim = new Alert("Elevator limits not enforced", AlertType.kWarning);

    private LoggedNetworkBoolean limits = new LoggedNetworkBoolean("overrides/elevatorLimits", true);

    double volt = 0.0;

    private SysIdRoutine sysid;
    @AutoLogOutput
    private boolean brake = true;
    
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

        if(DriverStation.isDisabled()){
            posSet = inputs.elevatorPositionMeters;
        }

        Logger.recordOutput("Elevator/CurrentCommand", getCurrentCommand() != null ? getCurrentCommand().getName() : "none");

        notHomed.set(!homed);

        double pidOut = pid.calculate(inputs.elevatorPositionMeters, posSet);
        double ffOut = ff.calculate(pid.getSetpoint().velocity);

        Logger.recordOutput("Elevator/Control/PID goal", posSet);
        Logger.recordOutput("Elevator/Control/PID setpoint pos", pid.getSetpoint().position);
        Logger.recordOutput("Elevator/Control/PID setpoint vel", pid.getSetpoint().velocity);
        Logger.recordOutput("Elevator/Control/PID applied", pidOut);
        Logger.recordOutput("Elevator/Control/FF aplied", ffOut);

        if(!openloop){
            if(!homed && inputs.elevatorPositionMeters < 0){
                io.resetElevatorPosition(0);
            }
            if(homed && limits.get()){
                posSet = MathUtil.clamp(posSet, MIN_POS, MAX_POS);
            }

            volt = pidOut + ffOut;
        } else {
            posSet = inputs.elevatorPositionMeters;
        }
        io.setVoltage(volt);

        noLim.set(!homed || openloop || !limits.get());
    }

    public void setHomed(boolean homed){
        this.homed = homed;
    }
    public boolean isHomed(){
        return homed;
    }

    public void setVoltage(double voltage){
        openloop = true;
        volt = voltage;
        io.setVoltage(voltage);
    }

    public void setTargetPos(double pos){
        this.posSet = pos;
        openloop = false;
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

    public Command man(DoubleSupplier change){
        return run(() -> {
            posSet += change.getAsDouble();
        }).beforeStarting(() -> {
            //reset pos on start to avoid jumping
            posSet = inputs.elevatorPositionMeters;
        }).withName("man");
    }

    public void toggleBrake(){
        io.setElevatorNeutralMode(brake);
        brake = !brake;
    }
    public void setBrake(boolean brake){
        io.setElevatorNeutralMode(brake);
        this.brake = brake;
    }
    
    public boolean inPosition(){
        return Math.abs(inputs.elevatorPositionMeters - posSet) < POS_TOLERANCE;
    }

    public void sysId(Voltage v){
        openloop = true;
        setVoltage(v.in(Volts));
    }

    public Pose3d getAScopePoseInnerStage(double pos){
        return new Pose3d(new Translation3d(0, 0, pos), new Rotation3d());
    }
    public Pose3d getAScopePoseMiddleStage(double pos){
        return new Pose3d(new Translation3d(0, 0, pos*0.544561), new Rotation3d());
    }

    public void stop(){
        setTargetPos(getPosition());
    }
    /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysid.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return  sysid.dynamic(direction);
  }
}
