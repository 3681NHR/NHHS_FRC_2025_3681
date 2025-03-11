package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.constants.WristConstants;
import frc.utils.ArmFF;
import frc.utils.ProfiledPID;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.WristConstants.*;

import java.util.function.DoubleSupplier;

public class Wrist extends SubsystemBase {

    private WristIO io;
    private WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

    private double posSet;
    @AutoLogOutput(key="Wrist/BrakeMode")
    private boolean brake;

    private ProfiledPID pid = new ProfiledPID(RobotBase.isReal() ? POS_PID : POS_PID_SIM);
    private ArmFF ff = new ArmFF(RobotBase.isReal() ? POS_FF : POS_FF_SIM);

    private Alert noLim = new Alert("wrist limits not enforced", AlertType.kWarning);
    private LoggedNetworkBoolean limits = new LoggedNetworkBoolean("overrides/wristLimits", true);

    private SysIdRoutine sysid;

    public Wrist(WristIO io){
        this.io = io;

        sysid = new SysIdRoutine(
            new Config(
                VRAMP,
                VSTEP,
                TIMEOUT,
                (s) -> Logger.recordOutput("Wrist/sysisState", s.toString())
            ), 
            new Mechanism((v) -> setVoltage(v.in(Volts)), null, this));
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("wrist", inputs);

        if(DriverStation.isDisabled()){
            posSet = inputs.posRad;
        }

        Logger.recordOutput("Wrist/CurrentCommand", getCurrentCommand() != null ? getCurrentCommand().getName() : "none");

        noLim.set(!limits.get());
        if(limits.get()){
            posSet = MathUtil.clamp(posSet, MIN_POS, MAX_POS);
        }
        double pidOut = pid.calculate(inputs.posRad, posSet);
        double ffOut = ff.calculate(inputs.posRad, pid.getSetpoint().velocity);

        Logger.recordOutput("Wrist/Control/PID goal", posSet);
        Logger.recordOutput("Wrist/Control/PID setpoint pos", pid.getSetpoint().position);
        Logger.recordOutput("Wrist/Control/PID setpoint vel", pid.getSetpoint().velocity);
        Logger.recordOutput("Wrist/Control/PID applied", pidOut);
        Logger.recordOutput("Wrist/Control/FF aplied", ffOut);

        io.setVoltage(pidOut + ffOut);
    }

    public void setPosSet(double pos){
        this.posSet = pos;
    }
    public double getPos(){
        return inputs.posRad;
    }
    public double getPosSet(){
        return posSet;
    }
    public Command man(DoubleSupplier move){
        return run(() -> {
            setPosSet(getPosSet() + move.getAsDouble());
        });
    }
    public void setBrake(boolean brake){
        this.brake = brake;
        io.setBrake(brake);
    }
    public void toggleBrake(){
        this.brake = !brake;
        setBrake(brake);
    }

    public boolean inPosition(){
        return Math.abs(inputs.posRad - posSet) < POS_TOLERANCE;
    }

    public Pose3d getAScopePoseWrist(double pos, double elevatorHeight){
        return new Pose3d(WristConstants.WRIST_POS.plus(new Translation3d(0, 0, elevatorHeight)), new Rotation3d(pos - (Math.PI/2.0), 0, 0));
    }

    public void stop(){
        setPosSet(getPos());
    }

    public void setVoltage(double v){
        io.setVoltage(v);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction dir){
        return sysid.quasistatic(dir);
    }
    public Command sysIdDynamic(SysIdRoutine.Direction dir){
        return sysid.dynamic(dir);
    }
}
