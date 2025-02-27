package frc.robot.subsystems.physButtons;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.physButtons.ButtonIO.ButtonIOInputs;

public class Buttons extends SubsystemBase{

    private final ButtonIO[] ios;
    private final ButtonIOInputsAutoLogged[] inputs;

    public Buttons(ButtonIO... buttons) {
        this.ios = buttons;
        this.inputs = new ButtonIOInputsAutoLogged[buttons.length];

        for(ButtonIOInputsAutoLogged input : inputs){
            input = new ButtonIOInputsAutoLogged();
        }
    }

    @Override
    public void periodic() {
        for(int i = 0; i < ios.length; i++){
            ios[i].updateInputs(inputs[i]);
            Logger.processInputs("buttons/"+i, inputs[i]);
        }
    }

    public boolean get(int index) {
        return inputs[index].pressed;
    }
}
