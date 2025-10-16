package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * subsystem for the climber
 * <p>
 * all hardware interface is done through climber
 * <p>
 * there is no sensor on the climber, and since it is driven through a winch,
 * builtin encoder readings are inaccurate. As such, the climber is controlled
 * through voltage control only
 */
public class Climber extends SubsystemBase {

    private ClimberIO io;

    private ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    private double vout = 0.0;

    public Climber(ClimberIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        // update IO and log inputs
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);

        // set voltage
        io.setVoltage(vout);
    }

    public void setVoltage(double vout) {
        this.vout = vout;
    }
}
