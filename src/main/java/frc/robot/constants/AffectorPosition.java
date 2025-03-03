package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public enum AffectorPosition {
    STOW   (0.0, Units.degreesToRadians(90)),
    L1     (0.0, Units.degreesToRadians(90)),
    L2     (0.118, Units.degreesToRadians(55)),
    L3     (0.53, Units.degreesToRadians(55)),
    L4     (1.467, Units.degreesToRadians(0)),
    STATION(0.445, Units.degreesToRadians(-50));

    public double elev;
    public double wrist;

    private AffectorPosition(double pos, double wrist){
        this.elev = pos;
        this.wrist = wrist;
    }

    public boolean isScoring(){
        return this == L1 || this == L2 || this == L3 || this == L4;
    }
}
