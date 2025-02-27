package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.constants.WristConstants.*;

public class Wrist extends SubsystemBase {

    private WristIO io;
    private WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

    private double pos;

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
            pos = inputs.posRad;
        }

        noLim.set(!limits.get());
        if(limits.get()){
            pos = MathUtil.clamp(pos, MIN_POS, MAX_POS);
        }

        io.setPos(pos);
    }

    public void setPos(double pos){
        this.pos = pos;
    }
    public double getPos(){
        return inputs.posRad;
    }
    public double getPosSet(){
        return pos;
    }
}
