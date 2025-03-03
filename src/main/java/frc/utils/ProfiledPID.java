package frc.utils;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class ProfiledPID extends ProfiledPIDController{
    public ProfiledPID(double kP, double kI, double kD, Constraints constraints){
        super(kP, kI, kD, constraints);
    }
    public ProfiledPID(double kP, double kI, double kD, Constraints constraints, double period){
        super(kP, kI, kD, constraints, period);
    }
    public ProfiledPID(PIDGains.ProfiledPID gains){
        super(gains.kP(), gains.kI(), gains.kD(), new Constraints(gains.maxSpeed(), gains.maxAccel()));
    }
}
