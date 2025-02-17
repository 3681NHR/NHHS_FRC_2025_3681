package frc.robot.subsystems.elevator;

public enum ElevatorPosition {
    DOWN(0.0),
    L1(0.0),
    L2(0.0),
    L3(0.0),
    L4(0.0),
    MAN(0.0),
    STATION(0.0);

    public double pos;

    private ElevatorPosition(double pos){
        this.pos = pos;
    }
}
