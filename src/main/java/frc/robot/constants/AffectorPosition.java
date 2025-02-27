package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public enum AffectorPosition {
    STOW   (0.0, Units.degreesToRadians(90)),
    L1     (0.0, Units.degreesToRadians(90)),
    L2     (0.0, Units.degreesToRadians(35)),
    L3     (0.0, Units.degreesToRadians(35)),
    L4     (0.0, Units.degreesToRadians(0)),
    STATION(0.0, Units.degreesToRadians(-30));

    public double elev;
    public double wrist;

    private AffectorPosition(double pos, double wrist){
        this.elev = pos;
        this.wrist = wrist;
    }
}
