package frc.utils;

import edu.wpi.first.math.controller.PIDController;

public class PID extends PIDController{
    public PID(double kP, double kI, double kD){
        super(kP, kI, kD);
    }
    public PID(double kP, double kI, double kD, double period){
        super(kP, kI, kD, period);
    }
    public PID(PIDGains.PID gains){
        super(gains.kP(), gains.kI(), gains.kD());
    }
}
