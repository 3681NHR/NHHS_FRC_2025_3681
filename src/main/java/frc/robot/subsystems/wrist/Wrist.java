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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.WristConstants;

import static frc.robot.constants.WristConstants.*;

import java.util.function.DoubleSupplier;

public class Wrist extends SubsystemBase {

    private WristIO io;
    private WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

    @AutoLogOutput
    private double posSet;
    @AutoLogOutput
    private boolean brake;

    private Alert noLim = new Alert("wrist limits not enforced", AlertType.kWarning);
    private LoggedNetworkBoolean limits = new LoggedNetworkBoolean("overrides/wristLimits", true);

    public Wrist(WristIO io){
        this.io = io;
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("wrist", inputs);

        if(DriverStation.isDisabled()){
            posSet = inputs.posRad;
        }

        noLim.set(!limits.get());
        if(limits.get()){
            posSet = MathUtil.clamp(posSet, MIN_POS, MAX_POS);
        }

        io.setPos(posSet);
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

    public Pose3d getAScopePoseWrist(double elevatorHeight){
        return new Pose3d(WristConstants.WRIST_POS.plus(new Translation3d(0, 0, elevatorHeight)), new Rotation3d(getPos()- (Math.PI/2.0), 0, 0));
    }
    public void stop(){
        setPosSet(getPos());
    }
}
