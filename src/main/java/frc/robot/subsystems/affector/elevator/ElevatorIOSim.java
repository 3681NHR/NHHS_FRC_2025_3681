package frc.robot.subsystems.affector.elevator;

import static frc.robot.constants.ElevatorConstants.*;

import edu.wpi.first.math.MathUtil;

/**
 * elevator IO simulation
 * <p>
 * this class generates realistic inputs for the elevator based on given outputs
 * from the subsystem
 */
public class ElevatorIOSim implements ElevatorIO {

    private double pos = 0.0;
    private double vel = 0.0;

    private double voltsOut = 0.0;

    public ElevatorIOSim() {
    }

    public void updateInputs(ElevatorIOInputs inputs) {

        // simple physics simulation, there are classes builtin that are better, but
        // this allows fine tuning for closer response without needing a full model
        double f = (MathUtil.clamp(voltsOut, -12, 12) * 24);// applied motor force
        f -= (vel * 100); // friction
        double a = f / (4); // acceleraton
        vel += a * 0.02; // velocity
        pos += vel * 0.02;// position
        pos = MathUtil.clamp(pos, MIN_POS, MAX_POS);// hard limits
        if (pos == MIN_POS || pos == MAX_POS) {
            vel = 0;// simulate hitting hard stop
        }

        inputs.pos = pos;
        inputs.vel = vel;

        inputs.motor1Voltage = voltsOut / 2;
        inputs.motor1TempC = -1;

        inputs.motor2CurrentAmps = inputs.motor1CurrentAmps;
        inputs.motor2Voltage = inputs.motor1Voltage;
        inputs.motor2TempC = inputs.motor1TempC;

    }

    public void setVoltage(double voltage) {
        voltsOut = voltage;
        // voltage is calculated in update inputs
    }

    public void setBrake(boolean brake) {
        // not used with sim
    }

    public void resetPos(double posMeters) {
        // not used
    }

}
