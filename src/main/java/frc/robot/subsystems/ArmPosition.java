package frc.robot.subsystems;

public class ArmPosition {
    public double angle = 0;
    public double extention = 0;
    public double wrist = 0;
    public ArmPosition(){}
    public ArmPosition(double a, double e, double w){
        angle = a;
        extention = e;
        wrist = w;
    }
}
