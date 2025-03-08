package frc.robot.subsystems.elevator;

import static frc.robot.constants.ElevatorConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.utils.BatteryVoltageSim;

public class ElevatorIOSim implements ElevatorIO{

    private double pos = 0.0;
    private double vel = 0.0;
    
    private double voltsOut = 0.0;

    private ElevatorSim sim = new ElevatorSim(
        LinearSystemId.identifyPositionSystem(POS_FF.kV(), POS_FF.kA()),
        DCMotor.getNEO(2),
        MIN_POS,
        MAX_POS,
        true,
        0,
        0, 0
    );

    public ElevatorIOSim(){
        BatteryVoltageSim.getInstance().addCurrentSource(()-> sim.getCurrentDrawAmps());
    }
    
    
    public void updateInputs(ElevatorIOInputs inputs) {

        sim.update(0.02);

        pos = sim.getPositionMeters();
        vel = sim.getVelocityMetersPerSecond();
        
        sim.setInputVoltage(voltsOut);

        inputs.elevatorPositionMeters = pos;
        inputs.elevatorVelocityMetersPerSec = vel;

        inputs.motor1Voltage = voltsOut/2;
        inputs.motor1TempC = -1;

        inputs.motor2CurrentAmps = inputs.motor1CurrentAmps;
        inputs.motor2Voltage = inputs.motor1Voltage;
        inputs.motor2TempC = inputs.motor1TempC;

    }
    
    
    public void setVoltage(double voltage) {
        voltsOut = voltage;
    }
    
    public void setElevatorNeutralMode(boolean brake) {
        //not used with sim
    }
    
    public void resetElevatorPosition(double posMeters) {
        //not used
    }

}
