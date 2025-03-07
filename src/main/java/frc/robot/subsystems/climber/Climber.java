package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase{

    private ClimberIO io;

    private ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    private double vout = 0.0;

    public Climber(ClimberIO io){
         this.io = io;
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);

        io.setVoltage(vout);
    }
    public void setVoltage(double vout){
        this.vout = vout;
    }
}
