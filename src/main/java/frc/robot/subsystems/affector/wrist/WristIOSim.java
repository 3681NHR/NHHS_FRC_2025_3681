package frc.robot.subsystems.affector.wrist;

import static frc.robot.constants.WristConstants.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.utils.BatteryVoltageSim;
import frc.utils.ExtraMath.Derrivitive;

public class WristIOSim implements WristIO {

    private double vout = 0.0;

    // private SingleJointedArmSim arm = new SingleJointedArmSim(
    //     LinearSystemId.createSingleJointedArmSystem(DCMotor.getNEO(1), SingleJointedArmSim.estimateMOI(Units.inchesToMeters(8), Units.lbsToKilograms(8.635)), 24),
    //     DCMotor.getNEO(1),
    //     24,
    //     Units.inchesToMeters(12),
    //     MIN_POS,
    //     MAX_POS,
    //     true,
    //     Math.PI/2,
    //     0, 0
    // );
    
    private double pos;
    private double vel;

    private Derrivitive velDeriv = new Derrivitive();

    public WristIOSim(){
        // BatteryVoltageSim.getInstance().addCurrentSource(()-> arm.getCurrentDrawAmps());
    }

    public void setBrake(boolean brake){}
    @Override
    public void updateInputs(WristIOInputs in){
        // pos = MathUtil.inputModulus(arm.getAngleRads(), -Math.PI, Math.PI);
        // vel = velDeriv.calculate(pos, 0.02);
        // arm.setInputVoltage(vout);
        // arm.update(0.02);

        double f = (MathUtil.clamp(vout, -12, 12)*12);
        f -= (vel*50);
        double a = f/(3);//f/m
        a -= Math.cos(pos)*(9.81*Units.inchesToMeters(12)); // gravity
        vel += a * 0.02;
        pos += vel * 0.02;
        pos = MathUtil.clamp(pos, MIN_POS, MAX_POS);
        if(pos == MIN_POS || pos == MAX_POS){
            vel = 0;
        }
        Logger.recordOutput("sim/wrist/f", f);
        Logger.recordOutput("sim/wrist/a", a);

        in.pos = pos;
        in.vel = vel;

        // in.motorCurrentAmps = arm.getCurrentDrawAmps();
        in.motoroutVolts = vout;
        in.motorTemp = 0.0;
    }

    public void setVoltage(double v) {
        vout = v;
    }
}
