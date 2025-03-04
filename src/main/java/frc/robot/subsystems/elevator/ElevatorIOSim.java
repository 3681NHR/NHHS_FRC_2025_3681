package frc.robot.subsystems.elevator;

import static frc.robot.constants.ElevatorConstants.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.utils.BatteryVoltageSim;
import frc.utils.ElevatorFF;
import frc.utils.ProfiledPID;

public class ElevatorIOSim implements ElevatorIO{

    private double posOffset = 0.0;
    private double pos = 0.0;
    private double lastPos = 0.0;
    private double vel = 0.0;
    private double posSetpoint = 0.0;
    
    private double voltsOut = 0.0;
    private boolean openloop = false;
    
    private ProfiledPID pid = new ProfiledPID(POS_PID_SIM);
    private ElevatorFF ff = new ElevatorFF(POS_FF_SIM);

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

        posSetpoint = MathUtil.clamp(posSetpoint, MIN_POS, MAX_POS);

        double pidOut = pid.calculate(pos, posSetpoint);
        double ffOut = ff.calculate(pos, pid.getSetpoint().velocity);
        Logger.recordOutput("Elevator/pidOut", pidOut);
        Logger.recordOutput("Elevator/target", posSetpoint);
        Logger.recordOutput("Elevator/setpoint", pid.getSetpoint().position);
        if(!openloop){
            sim.setInputVoltage(ffOut + pidOut);
        } else {
            sim.setInputVoltage(voltsOut);
        }

        inputs.elevatorPositionMeters = pos;
        inputs.elevatorVelocityMetersPerSec = vel;

        inputs.motor1Voltage = voltsOut/2;
        inputs.motor1TempC = -1;

        inputs.motor2CurrentAmps = inputs.motor1CurrentAmps;
        inputs.motor2Voltage = inputs.motor1Voltage;
        inputs.motor2TempC = inputs.motor1TempC;

    }
    
    public void setElevatorTargetLocation(double targetMeters) {
        posSetpoint = targetMeters;
        openloop = false;
    }
    
    public void moveElevatorOpenLoop(double voltage) {
        openloop = true;
        voltsOut = voltage;
    }
    
    public void setElevatorNeutralMode(boolean brake) {
        //not used with sim
    }
    
    public void resetElevatorPosition(double posMeters) {
        posOffset = posMeters;
    }

}
