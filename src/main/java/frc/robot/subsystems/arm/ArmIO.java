package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    default void updateInputs(ArmIOInputs inputs){}
    public void setAngleSetpoint(double setpoint);
    public void setExtentionSetpoint(double setpoint);
    public void setWristSetpoint(double setpoint);
    public void setPositionSetpoint(ArmPosition a);
    @AutoLog
    public class ArmIOInputs{
        public double angle;
        public double extention;
        public double wrist;

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
