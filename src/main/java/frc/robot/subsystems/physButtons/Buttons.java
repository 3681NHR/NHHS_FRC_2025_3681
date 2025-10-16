package frc.robot.subsystems.physButtons;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * subsystem to handle physical buttons, using an array of ButtonIO
 * implementations to handle multiple buttons
 */
public class Buttons extends SubsystemBase {

    private final ButtonIO[] ios;
    private final ButtonIOInputsAutoLogged[] inputs;

    public Buttons(ButtonIO... buttons) {
        this.ios = buttons;
        this.inputs = new ButtonIOInputsAutoLogged[buttons.length];

        for (@SuppressWarnings("unused")
        ButtonIOInputsAutoLogged i : inputs) {
            i = new ButtonIOInputsAutoLogged();
        }
    }

    @Override
    public void periodic() {
        // update and log all buttons
        for (int i = 0; i < ios.length; i++) {
            if (inputs[i] == null) {
                inputs[i] = new ButtonIOInputsAutoLogged();
            }
            ios[i].updateInputs(inputs[i]);
            Logger.processInputs("button" + i, inputs[i]);
        }
    }

    /**
     * get the pressed state of a button at a given index, returns false if index is
     * out of bounds
     * 
     * @param index the index of the button to get
     * @return the pressed state of the button at the given index
     */
    public boolean get(int index) {
        if (index < inputs.length) {
            if (inputs[index] == null) {
                inputs[index] = new ButtonIOInputsAutoLogged();
            }
            return inputs[index].pressed;
        } else {
            return false;
        }
    }
}
