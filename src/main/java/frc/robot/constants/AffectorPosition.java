package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public enum AffectorPosition {
    STOW   (0.0, Units.degreesToRadians(90)),
    L1     (0/*.135*/, 0),//-1.066),
    L2     (0.195, 0.942),
    L3     (0.662, 0.942),
    L4     (1.6, 0.165),
    STATION(0.532, -0.873);

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
