package frc.utils;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.controller.ProfiledPIDController;

public class NetworkedProfiledPID extends ProfiledPIDController {
    private LoggedNetworkNumber nt_kP;
    private LoggedNetworkNumber nt_kI;
    private LoggedNetworkNumber nt_kD;
    private LoggedNetworkNumber nt_maxSpeed;
    private LoggedNetworkNumber nt_maxAccel;

    public NetworkedProfiledPID(double kP, double kI, double kD, Constraints constraints, String ntPath) {
        super(kP, kI, kD, constraints);
        nt_kD = new LoggedNetworkNumber(ntPath + "/kD", kD);
        nt_kI = new LoggedNetworkNumber(ntPath + "/kI", kI);
        nt_kP = new LoggedNetworkNumber(ntPath + "/kP", kP);
        nt_maxSpeed = new LoggedNetworkNumber(ntPath + "/maxSpeed", constraints.maxVelocity);
        nt_maxAccel = new LoggedNetworkNumber(ntPath + "/maxAccel", constraints.maxAcceleration);
    }

    public NetworkedProfiledPID(double kP, double kI, double kD, Constraints constraints, double period,
            String ntPath) {
        super(kP, kI, kD, constraints, period);
        nt_kD = new LoggedNetworkNumber(ntPath + "/kD", kD);
        nt_kI = new LoggedNetworkNumber(ntPath + "/kI", kI);
        nt_kP = new LoggedNetworkNumber(ntPath + "/kP", kP);
        nt_maxSpeed = new LoggedNetworkNumber(ntPath + "/maxSpeed", constraints.maxVelocity);
        nt_maxAccel = new LoggedNetworkNumber(ntPath + "/maxAccel", constraints.maxAcceleration);
    }

    public NetworkedProfiledPID(PIDGains.ProfiledPID gains, String ntPath) {
        super(gains.kP(), gains.kI(), gains.kD(), new Constraints(gains.maxSpeed(), gains.maxAccel()));
        nt_kD = new LoggedNetworkNumber(ntPath + "/kD", gains.kD());
        nt_kI = new LoggedNetworkNumber(ntPath + "/kI", gains.kI());
        nt_kP = new LoggedNetworkNumber(ntPath + "/kP", gains.kP());
        nt_maxSpeed = new LoggedNetworkNumber(ntPath + "/maxSpeed", gains.maxSpeed());
        nt_maxAccel = new LoggedNetworkNumber(ntPath + "/maxAccel", gains.maxAccel());
    }

    /**
     * Returns the next output of the PID controller.
     *
     * @param measurement The current measurement of the process variable.
     * @return The controller's next output.
     */
    public double calculate(double measurement) {
        this.setPID(nt_kP.get(), nt_kI.get(), nt_kD.get());
        this.setConstraints(new Constraints(nt_maxSpeed.get(), nt_maxAccel.get()));
        return super.calculate(measurement);
    }

    /**
     * Returns the next output of the PID controller.
     *
     * @param measurement The current measurement of the process variable.
     * @param goal        The new goal of the controller.
     * @return The controller's next output.
     */
    public double calculate(double measurement, TrapezoidProfile.State goal) {
        setGoal(goal);
        return this.calculate(measurement);
    }

    /**
     * Returns the next output of the PIDController.
     *
     * @param measurement The current measurement of the process variable.
     * @param goal        The new goal of the controller.
     * @return The controller's next output.
     */
    public double calculate(double measurement, double goal) {
        setGoal(goal);
        return this.calculate(measurement);
    }

    /**
     * Returns the next output of the PID controller.
     *
     * @param measurement The current measurement of the process variable.
     * @param goal        The new goal of the controller.
     * @param constraints Velocity and acceleration constraints for goal.
     * @return The controller's next output.
     */
    public double calculate(
            double measurement, TrapezoidProfile.State goal, TrapezoidProfile.Constraints constraints) {
        setConstraints(constraints);
        return this.calculate(measurement, goal);
    }
}
