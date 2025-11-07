package frc.robot.subsystems.physButtons;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;

/**
 * Button IO implementation for Digital Input on real robot
 */
public class ButtonIODIO implements ButtonIO {

    private final DigitalInput dio;

    private final Debouncer debouncer = new Debouncer(0.05, Debouncer.DebounceType.kBoth);

    public ButtonIODIO(int channel) {
        dio = new DigitalInput(channel);
    }

    @Override
    public void updateInputs(ButtonIOInputs inputs) {
        // debounce button press to account for noise
        inputs.pressed = debouncer.calculate(dio.get());
    }

}
