package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.HomeElevator;

public class Elevator extends SubsystemBase {

    private ElevatorIO io;
    private ElevatorIOInputsAutoLogged inputs;
    private double pos = 0.0;

    private boolean homed = false;
    private boolean openloop = false;
    
    public Elevator(ElevatorIO io){
        this.io = io;

        if(!homed){
            CommandScheduler.getInstance().schedule(new HomeElevator(this));
        }
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("elevator", inputs);

        if(!openloop){
            io.setTargetLocation(pos);
        }
    }

    public void setHomed(boolean homed){
        this.homed = homed;
    }
    public boolean isHomed(){
        return homed;
    }

    public void setVoltage(double voltage){
        io.moveOpenLoop(voltage);
        openloop = true;
    }

    public void setTargetPos(double pos){
        this.pos = pos;
        openloop = false;
    }

    public double getPosition(){
        return pos;
    }
    public double getVelocity() {
        return inputs.velocityMetersPerSec;
    }
    public void resetPos(double pos){
        io.resetposition(pos);
    }
}
