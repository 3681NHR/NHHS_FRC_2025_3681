package frc.utils;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import edu.wpi.first.math.controller.PIDController;

public class NetworkedPID extends PIDController {
    private LoggedNetworkNumber nt_kP;
    private LoggedNetworkNumber nt_kI;
    private LoggedNetworkNumber nt_kD;

    public NetworkedPID(double kP, double kI, double kD, String tunePath) {
        super(kP, kI, kD);
        nt_kD = new LoggedNetworkNumber("/Tuning" + tunePath + "/kD", kD);
        nt_kI = new LoggedNetworkNumber("/Tuning" + tunePath + "/kI", kI);
        nt_kP = new LoggedNetworkNumber("/Tuning" + tunePath + "/kP", kP);
    }

    public NetworkedPID(double kP, double kI, double kD, double period, String tunePath) {
        super(kP, kI, kD, period);
        nt_kD = new LoggedNetworkNumber("/Tuning" + tunePath + "/kD", kD);
        nt_kI = new LoggedNetworkNumber("/Tuning" + tunePath + "/kI", kI);
        nt_kP = new LoggedNetworkNumber("/Tuning" + tunePath + "/kP", kP);
    }

    public NetworkedPID(PIDGains.PID gains, String tunePath) {
        super(gains.kP(), gains.kI(), gains.kD());
        nt_kD = new LoggedNetworkNumber("/Tuning" + tunePath + "/kD", gains.kD());
        nt_kI = new LoggedNetworkNumber("/Tuning" + tunePath + "/kI", gains.kI());
        nt_kP = new LoggedNetworkNumber("/Tuning" + tunePath + "/kP", gains.kP());
    }

    /**
     * Returns the next output of the PID controller.
     *
     * @param measurement The current measurement of the process variable.
     * @return The next controller output.
     */
    public double calculate(double measurement) {
        this.setPID(nt_kP.get(), nt_kI.get(), nt_kD.get());
        return super.calculate(measurement);
    }

    /**
     * Returns the next output of the PID controller.
     *
     * @param measurement The current measurement of the process variable.
     * @param setpoint    The new setpoint of the controller.
     * @return The next controller output.
     */
    public double calculate(double measurement, double setpoint) {
        setSetpoint(setpoint);
        return this.calculate(measurement);
    }
}
