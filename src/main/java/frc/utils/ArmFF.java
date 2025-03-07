package frc.utils;

import edu.wpi.first.math.controller.ArmFeedforward;

public class ArmFF extends ArmFeedforward{
    public ArmFF(double kS, double kG, double kV, double kA){
        super(kS, kG, kV, kA);
    }
    public ArmFF(double kS, double kG, double kV, double kA, double dt){
        super(kS, kG, kV, kA, dt);
    }
    public ArmFF(PIDGains.GravityFF gains){
        super(gains.kS(), gains.kG(), gains.kV(), gains.kA());
    }
}
