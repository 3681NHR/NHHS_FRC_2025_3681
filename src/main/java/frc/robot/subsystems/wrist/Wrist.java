package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.constants.WristConstants.*;

public class Wrist extends SubsystemBase {

    private WristIO io;
    private WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

    private double pos;

    public Wrist(WristIO io){
        this.io = io;
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("wrist", inputs);

        //pos = MathUtil.clamp(pos, MIN_POS, MAX_POS);

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
