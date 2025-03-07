package frc.utils;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class SimpleFF extends SimpleMotorFeedforward{
    public SimpleFF(double kS, double kV, double kA){
        super(kS, kV, kA);
    }
    public SimpleFF(double kS, double kV, double kA, double dt){
        super(kS, kV, kA, dt);
    }
    public SimpleFF(PIDGains.SimpleFF gains){
        super(gains.kS(), gains.kV(), gains.kA());
    }
}
