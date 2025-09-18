package frc.robot.subsystems.affector.wrist;

import static frc.robot.constants.WristConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.Constants;

public class WristIOSim implements WristIO {

    private double vout = 0.0;

    
    private double pos = Constants.Affector.STOW_POSITION.wrist;
    private double vel;

    public WristIOSim(){
    }

    public void setBrake(boolean brake){}
    @Override
    public void updateInputs(WristIOInputs in){

        double f = (MathUtil.clamp(vout, -12, 12)*40);
        f -= (vel*40);
        double a = f/(3.5);//f/m
        a -= Math.cos(pos)*(9.81*Units.inchesToMeters(14)); // gravity
        vel += a * 0.02;
        pos += vel * 0.02;
        pos = MathUtil.clamp(pos, MIN_POS, MAX_POS);
        if(pos == MIN_POS || pos == MAX_POS){
            vel = 0;
        }

        in.pos = pos;
        in.vel = vel;

        in.motoroutVolts = vout;
        in.motorTemp = 0.0;
    }

    public void setVoltage(double v) {
        vout = v;
    }
}
