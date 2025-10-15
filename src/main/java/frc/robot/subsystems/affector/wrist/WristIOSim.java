package frc.robot.subsystems.affector.wrist;

import static frc.robot.constants.WristConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.Constants;

/**
 * simulated wrist IO
 * <p>generated realistic input values based on outputs from the subsystem
 */
public class WristIOSim implements WristIO {

    private double vout = 0.0;

    private double pos = Constants.Affector.STOW_POSITION.wrist;
    private double vel;

    public WristIOSim(){
    }

    public void setBrake(boolean brake){}
    @Override
    public void updateInputs(WristIOInputs in){

        double f = (MathUtil.clamp(vout, -12, 12)*40);//motor force
        f -= (vel*40); //friction
        double a = f/(3.5); // acceleration
        a -= Math.cos(pos)*(9.81*Units.inchesToMeters(14)); // gravity
        vel += a * 0.02;// velocity
        pos += vel * 0.02;// position
        pos = MathUtil.clamp(pos, MIN_POS, MAX_POS);//hard stop
        if(pos == MIN_POS || pos == MAX_POS){
            vel = 0;//simulate hitting hard stop
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
