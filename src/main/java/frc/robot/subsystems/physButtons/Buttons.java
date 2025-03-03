package frc.robot.subsystems.physButtons;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Buttons extends SubsystemBase{

    private final ButtonIO[] ios;
    private final ButtonIOInputsAutoLogged[] inputs;

    public Buttons(ButtonIO... buttons) {
        this.ios = buttons;
        this.inputs = new ButtonIOInputsAutoLogged[buttons.length];

        for(ButtonIOInputsAutoLogged i : inputs){
            i = new ButtonIOInputsAutoLogged();
        }
    }

    @Override
    public void periodic() {
        for(int i = 0; i < ios.length; i++){
            if(inputs[i] == null){
                inputs[i] = new ButtonIOInputsAutoLogged();
            }
            ios[i].updateInputs(inputs[i]);
            Logger.processInputs("buttons/"+i, inputs[i]);
        }
    }

    public boolean get(int index) {
        if(index < inputs.length){
            if(inputs[index] == null){
                inputs[index] = new ButtonIOInputsAutoLogged();
            }
            return inputs[index].pressed;
        } else {
            return false;
        }
    }
}
