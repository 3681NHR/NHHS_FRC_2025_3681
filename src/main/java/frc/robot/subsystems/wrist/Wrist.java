package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {

    private WristIO io;
    private WristIOInputsAutoLogged inputs;

    private double pos;

    public Wrist(WristIO io){
        this.io = io;
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("wrist", inputs);

        io.setPos(pos);
    }

    public void setPos(double pos){
        this.pos = pos;
    }
}
