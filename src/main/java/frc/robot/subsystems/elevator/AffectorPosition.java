package frc.robot.subsystems.elevator;

public enum AffectorPosition {
    STOW(0.0, 0.0),
    L1(0.0, 0.0),
    L2(0.0, 0.0),
    L3(0.0, 0.0),
    L4(0.0, 0.0),
    STATION(0.0, 0.0);

    public double elev;
    public double wrist;

    private AffectorPosition(double pos, double wrist){
        this.elev = pos;
        this.wrist = wrist;
    }
}
