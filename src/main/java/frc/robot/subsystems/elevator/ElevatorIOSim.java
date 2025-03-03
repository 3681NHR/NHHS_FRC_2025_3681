package frc.robot.subsystems.elevator;

import static frc.robot.constants.ElevatorConstants.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
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

    public ElevatorIOSim(){
    }
    
    
    public void updateInputs(ElevatorIOInputs inputs) {
        pos = pid.getSetpoint().position;
        vel = (pos-lastPos)/0.02;
        lastPos = pos;

        posSetpoint = MathUtil.clamp(posSetpoint, MIN_POS, MAX_POS);

        double pidOut = pid.calculate(pos, posSetpoint);
        Logger.recordOutput("Elevator/pidOut", pidOut);
        Logger.recordOutput("Elevator/target", posSetpoint);
        Logger.recordOutput("Elevator/setpoint", pid.getSetpoint().position);


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
