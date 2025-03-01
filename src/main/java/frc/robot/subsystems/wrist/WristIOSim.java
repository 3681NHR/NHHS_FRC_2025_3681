package frc.robot.subsystems.wrist;

import static frc.robot.constants.WristConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class WristIOSim implements WristIO {

    private SingleJointedArmSim arm = new SingleJointedArmSim(
        LinearSystemId.createSingleJointedArmSystem(DCMotor.getNEO(1), 1, 24),
        DCMotor.getNEO(1),
        24,
        Units.inchesToMeters(12),
        MIN_POS,
        MAX_POS,
        true,
        0,
        0.1
    );

    @Override
    public void setPos(double pos){

    }
    public void setBrake(boolean brake){}
    @Override
    public void updateInputs(WristIOInputs in){
        in.posRad = 0.0;
        in.velRadPerSec = 0.0;

        in.motorCurrentAmps = 0.0;
        in.motoroutVolts = 0.0;
        in.motorTemp = 0.0;
    }
}
