package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;

public interface Elevator {
    default void updateInputs(ArmIOInputs inputs){}
    public void setAngleSetpoint(double setpoint);
    public void setExtentionSetpoint(double setpoint);
    public void setWristSetpoint(double setpoint);
    public void setPositionSetpoint(ElevatorPosition a);
    @AutoLog
    public class ArmIOInputs{
        public double angle;
        public double extention;
        public double wrist;

        public Pose3d armP1Pose;
        public Pose3d armP2Pose;
        public Pose3d armP3Pose;
        public Pose3d wristPose;

        public double angleMotor1Temp;
        public double angleMotor2Temp;
        public double extentionMotor1Temp;
        public double extentionMotor2Temp;
        public double wristMotorTemp;

        public double angleAppliedOut;
        public double angleCurrentDrawAmps;

        public double extentionAppliedOut;
        public double extentionCurrentDrawAmps;

        public double wristAppliedOut;
        public double wristCurrentDrawAmps;

    }
}
