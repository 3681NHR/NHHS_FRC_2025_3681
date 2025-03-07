package frc.robot.subsystems.physButtons;

import java.util.function.BooleanSupplier;


public class ButtonIOSim implements ButtonIO {

    private final BooleanSupplier io;


    public ButtonIOSim(BooleanSupplier io) {
        this.io = io;
    }

    @Override
    public void updateInputs(ButtonIOInputs inputs) {
        inputs.pressed = io.getAsBoolean();
    }
    
}
