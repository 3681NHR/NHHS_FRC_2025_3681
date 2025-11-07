package frc.utils;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class NetworkedSimpleFF extends SimpleMotorFeedforward {
    private LoggedNetworkNumber nt_kS;
    private LoggedNetworkNumber nt_kV;
    private LoggedNetworkNumber nt_kA;

    public NetworkedSimpleFF(double kS, double kV, double kA, String ntPath) {
        super(kS, kV, kA);
        nt_kS = new LoggedNetworkNumber("/Tuning" + ntPath + "/kS", kS);
        nt_kV = new LoggedNetworkNumber("/Tuning" + ntPath + "/kV", kV);
        nt_kA = new LoggedNetworkNumber("/Tuning" + ntPath + "/kA", kA);
    }

    public NetworkedSimpleFF(double kS, double kV, double kA, double dt, String ntPath) {
        super(kS, kV, kA, dt);
        nt_kS = new LoggedNetworkNumber("/Tuning" + ntPath + "/kS", kS);
        nt_kV = new LoggedNetworkNumber("/Tuning" + ntPath + "/kV", kV);
        nt_kA = new LoggedNetworkNumber("/Tuning" + ntPath + "/kA", kA);
    }

    public NetworkedSimpleFF(PIDGains.SimpleFF gains, String ntPath) {
        super(gains.kS(), gains.kV(), gains.kA());
        nt_kS = new LoggedNetworkNumber("/Tuning" + ntPath + "/kS", gains.kS());
        nt_kV = new LoggedNetworkNumber("/Tuning" + ntPath + "/kV", gains.kV());
        nt_kA = new LoggedNetworkNumber("/Tuning" + ntPath + "/kA", gains.kA());
    }

    /**
     * Calculates the feedforward from the gains and setpoints assuming continuous
     * control.
     *
     * @param velocity     The velocity setpoint.
     * @param acceleration The acceleration setpoint.
     * @return The computed feedforward.
     * @deprecated Use {@link #calculateWithVelocities(double, double)} instead.
     */
    public double calculate(double velocity, double acceleration) {
        this.setKs(nt_kS.get());
        this.setKv(nt_kV.get());
        this.setKa(nt_kA.get());

        return super.calculate(velocity);
    }

    /**
     * Calculates the feedforward from the gains and velocity setpoint assuming
     * continuous control
     * (acceleration is assumed to be zero).
     *
     * @param velocity The velocity setpoint.
     * @return The computed feedforward.
     */
    public double calculate(double velocity) {
        return this.calculate(velocity, 0);
    }

    /**
     * Calculates the feedforward from the gains and setpoints assuming discrete
     * control.
     *
     * <p>
     * Note this method is inaccurate when the velocity crosses 0.
     *
     * @param currentVelocity The current velocity setpoint.
     * @param nextVelocity    The next velocity setpoint.
     * @return The computed feedforward.
     */
    public double calculateWithVelocities(double currentVelocity, double nextVelocity) {
        this.setKs(nt_kS.get());
        this.setKv(nt_kV.get());
        this.setKa(nt_kA.get());

        return super.calculateWithVelocities(currentVelocity, nextVelocity);
    }
}
