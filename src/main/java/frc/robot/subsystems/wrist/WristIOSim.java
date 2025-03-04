package frc.robot.subsystems.wrist;

import static frc.robot.constants.WristConstants.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.utils.ArmFF;
import frc.utils.BatteryVoltageSim;
import frc.utils.ExtraMath.Derrivitive;
import frc.utils.ProfiledPID;

public class WristIOSim implements WristIO {

    private SingleJointedArmSim arm = new SingleJointedArmSim(
        LinearSystemId.createSingleJointedArmSystem(DCMotor.getNEO(1), SingleJointedArmSim.estimateMOI(Units.inchesToMeters(8), Units.lbsToKilograms(8.635)), 24),
        DCMotor.getNEO(1),
        24,
        Units.inchesToMeters(12),
        MIN_POS,
        MAX_POS,
        true,
        Math.PI/2,
        0, 0
    );

    
    private double pos;
    private double vel;
    private double posSet;

    private ArmFF ff = new ArmFF(POS_FF_SIM);

    private ProfiledPID pid = new ProfiledPID(POS_PID_SIM);

    private Derrivitive velDeriv = new Derrivitive();

    public WristIOSim(){
        BatteryVoltageSim.getInstance().addCurrentSource(()-> arm.getCurrentDrawAmps());
    }

    @Override
    public void setPos(double pos){
        posSet = pos;
    }
    public void setBrake(boolean brake){}
    @Override
    public void updateInputs(WristIOInputs in){
        pos = MathUtil.inputModulus(arm.getAngleRads(), -Math.PI, Math.PI);
        vel = velDeriv.calculate(pos, 0.02);

        double pidOut = pid.calculate(pos, posSet);
        double ffOut = ff.calculate(pos, pid.getSetpoint().velocity);

        Logger.recordOutput("Wrist/PID goal", posSet);
        Logger.recordOutput("Wrist/PID set", pid.getSetpoint().position);
        Logger.recordOutput("Wrist/PID out", pidOut);
        Logger.recordOutput("Wrist/FF out", ffOut);

        arm.setInputVoltage(ffOut + pidOut);
        arm.update(0.02);

        in.posRad = pos;
        in.velRadPerSec = vel;

        in.motorCurrentAmps = arm.getCurrentDrawAmps();
        in.motoroutVolts = ffOut + pidOut;
        in.motorTemp = 0.0;
    }

    public void setVoltage(double v) {

    }
}
