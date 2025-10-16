package frc.robot.subsystems.physButtons;

import java.util.function.BooleanSupplier;

/**
 * Button IO implementation for simulation, takes in a boolean supplier to
 * simulate button presses
 */
public class ButtonIOSim implements ButtonIO {

    private final BooleanSupplier io;

    public ButtonIOSim(BooleanSupplier io) {
        this.io = io;
    }

    @Override
    public void updateInputs(ButtonIOInputs inputs) {
        // get supplier for button value
        inputs.pressed = io.getAsBoolean();
    }

}
